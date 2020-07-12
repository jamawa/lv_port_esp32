cmake_minimum_required(VERSION 3.5)

if (NOT DEFINED PROJECT_NAME)
	include($ENV{IDF_PATH}/tools/cmake/project.cmake)
endif (NOT DEFINED PROJECT_NAME)
list(APPEND EXTRA_COMPONENT_DIRS components/lvgl_esp32_drivers components/lvgl_esp32_drivers/lvgl_touch components/lvgl_esp32_drivers/lvgl_tft)

if (NOT DEFINED PROJECT_NAME)
	project(lvgl-demo)
endif (NOT DEFINED PROJECT_NAME)
