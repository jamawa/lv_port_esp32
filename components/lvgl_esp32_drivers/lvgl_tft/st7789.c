/**
 * @file st7789.c
 *
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_log.h"

#include "st7789.h"

#include "disp_spi.h"
#include "driver/gpio.h"

#include "math.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "st7789"
/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void st7789_set_orientation(uint8_t orientation);

static void st7789_send_cmd(uint8_t cmd);
static void st7789_send_data(void *data, uint16_t length);
static void st7789_send_color(void *data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/
/* The ST7789 display controller can drive 320*240 displays. When using a smaller display it will
 * usually be placed in the center of this buffer, to make rotation and mirroring more intuitive.
 * The offset coordinates of the screen need to be taken into account during initialization and flushing, so they are stored.
 */
static uint16_t display_offset_x = 0;		// X offset of the display with respect to the ST7789 video buffer
static uint16_t display_offset_y = 0;		// Y offset of the display with respect to the ST7789 video buffer

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void st7789_init(void)
{
    lcd_init_cmd_t st7789_init_cmds[] = {
        {ST7789_COLMOD, {0x55}, 1},		// 65k RGB interface, 16bit/pixel
		{ST7789_INVON, {0}, 0},
        {ST7789_RAMWR, {0}, 0},
        {ST7789_SLPOUT, {0}, 0x80},
        {ST7789_DISPON, {0}, 0x80},
        {0, {0}, 0xff},
    };

    //Initialize non-SPI GPIOs
    gpio_set_direction(ST7789_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(ST7789_RST, GPIO_MODE_OUTPUT);
    
#if ST7789_ENABLE_BACKLIGHT_CONTROL
    gpio_set_direction(ST7789_BCKL, GPIO_MODE_OUTPUT);
#endif

    //Reset the display
    gpio_set_level(ST7789_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(ST7789_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    ESP_LOGI(TAG, "ST7789 initialization");

    //Send all the commands
    uint16_t cmd = 0;
    while (st7789_init_cmds[cmd].databytes!=0xff) {
        st7789_send_cmd(st7789_init_cmds[cmd].cmd);
        st7789_send_data(st7789_init_cmds[cmd].data, st7789_init_cmds[cmd].databytes&0x1F);
        if (st7789_init_cmds[cmd].databytes & 0x80) {
                vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    /* Determine offset of display with respect to ST7789 video buffer which is 240x320.
     * Assume the display is placed in the center of the buffer. If not; the function st7789_set_display_area can be called
     * after initialization with the correct offsets.
     */
#if (CONFIG_LVGL_DISPLAY_ORIENTATION_PORTRAIT)
    display_offset_x = round((240 - LV_HOR_RES_MAX) / 2.0);
    display_offset_y = round((320 - LV_VER_RES_MAX) / 2.0);
#elif (CONFIG_LVGL_DISPLAY_ORIENTATION_PORTRAIT_INVERTED)
    display_offset_x = round((240 - LV_HOR_RES_MAX) / 2.0 - 0.1);
    display_offset_y = round((320 - LV_VER_RES_MAX) / 2.0 - 0.1);
#elif (CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE)
    display_offset_x = round((320 - LV_HOR_RES_MAX) / 2.0);
    display_offset_y = round((240 - LV_VER_RES_MAX) / 2.0);
#elif (CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED)
    display_offset_x = round((320 - LV_HOR_RES_MAX) / 2.0 - 0.1);
    display_offset_y = round((240 - LV_VER_RES_MAX) / 2.0 - 0.1);
#endif

    ESP_LOGI(TAG, "Display origin (x,y): (%d,%d)", display_offset_x, display_offset_y);
	st7789_set_display_area(display_offset_x, display_offset_y);
	st7789_set_orientation(CONFIG_LVGL_DISPLAY_ORIENTATION);
	st7789_enable_backlight(true);
}

void st7789_enable_backlight(bool backlight)
{
#if ST7789_ENABLE_BACKLIGHT_CONTROL
    ESP_LOGI(TAG, "%s backlight", backlight ? "Enabling" : "Disabling");

    uint32_t tmp = 0;

#if (ST7789_BCKL_ACTIVE_LVL==1)
    tmp = backlight ? 1 : 0;
#else
    tmp = backlight ? 0 : 1;
#endif

    gpio_set_level(ST7789_BCKL, tmp);
#endif
}

void st7789_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    uint8_t data[4] = {0};
    ESP_LOGI(TAG, "Flushing area (%d,%d)-(%d,%d) at offset (%d,%d)",
    		area->x1, area->y1, area->x2, area->y2, display_offset_x, display_offset_y);

    // Adjust area for actual display location within the ST7789 video buffer
    uint16_t offsetx1 = display_offset_x + area->x1;
    uint16_t offsetx2 = display_offset_x + area->x2;
    uint16_t offsety1 = display_offset_y + area->y1;
    uint16_t offsety2 = display_offset_y + area->y2;

    /*Column addresses*/
    st7789_send_cmd(ST7789_CASET);
    data[0] = (offsetx1 >> 8) & 0xFF;
    data[1] = offsetx1 & 0xFF;
    data[2] = (offsetx2 >> 8) & 0xFF;
    data[3] = offsetx2 & 0xFF;
    st7789_send_data(data, 4);

    /*Page addresses*/
    st7789_send_cmd(ST7789_RASET);
    data[0] = (offsety1 >> 8) & 0xFF;
    data[1] = offsety1 & 0xFF;
    data[2] = (offsety2 >> 8) & 0xFF;
    data[3] = offsety2 & 0xFF;
    st7789_send_data(data, 4);

    /*Memory write*/
    st7789_send_cmd(ST7789_RAMWR);
    uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);
    st7789_send_color((void*)color_map, size * 2);
}

/* st7789_set_display_area configures the placement of the display with respect to the ST7789 internal video buffer of 240x320.
 * When using a smaller display it will usually be placed in the center of this buffer, to make rotation and mirroring more intuitive.
 * The function takes orientation and mirroring into account.
 */
void st7789_set_display_area(uint16_t offset_x, uint16_t offset_y)
{
	uint8_t data[4] = {0};

	// store screen offset values for use in f.e. flush
	display_offset_x = offset_x;
	display_offset_y = offset_y;

	// Calculate display area
    uint16_t offset_x2 = offset_x + LV_HOR_RES_MAX - 1;
    uint16_t offset_y2 = offset_y + LV_VER_RES_MAX - 1;

    /* Set column addresses*/
    st7789_send_cmd(ST7789_CASET);
    data[0] = (offset_x >> 8) & 0xFF;
    data[1] = offset_x & 0xFF;
    data[2] = (offset_x2 >> 8) & 0xFF;
    data[3] = offset_x2 & 0xFF;
    st7789_send_data(data, 4);

    /* Set row addresses*/
    st7789_send_cmd(ST7789_RASET);
    data[0] = (offset_y >> 8) & 0xFF;
    data[1] = offset_y & 0xFF;
    data[2] = (offset_y2 >> 8) & 0xFF;
    data[3] = offset_y2 & 0xFF;
    st7789_send_data(data, 4);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void st7789_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ST7789_DC, 0);
    disp_spi_send_data(&cmd, 1);
}

static void st7789_send_data(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ST7789_DC, 1);
    disp_spi_send_data(data, length);
}

static void st7789_send_color(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ST7789_DC, 1);
    disp_spi_send_colors(data, length);
}

static void st7789_set_orientation(uint8_t orientation)
{
    const char *orientation_str[] = {
        "PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"
    };

    ESP_LOGI(TAG, "Display orientation: %s", orientation_str[orientation]);

#if defined (CONFIG_LVGL_PREDEFINED_DISPLAY_NONE)
    uint8_t data[] = {0xC0, 0x00, 0x60, 0xA0};
#endif

    st7789_send_cmd(ST7789_MADCTL);
    st7789_send_data((void *) &data[orientation], 1);
}
