cmake_minimum_required(VERSION 3.16)
project(ui_preview C CXX)
# Builds the balancer UI against LVGL 9.5 on a PC and renders every screen.
# usage: cmake -S tools/ui_preview -B build/ui -DLVGL_DIR=/path/to/lvgl-9.5.0
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT LVGL_DIR)
  message(FATAL_ERROR "set -DLVGL_DIR to an LVGL v9.5.0 checkout")
endif()

file(GLOB_RECURSE LVGL_SOURCES ${LVGL_DIR}/src/*.c)
add_library(lvgl STATIC ${LVGL_SOURCES})
target_include_directories(lvgl PUBLIC ${LVGL_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(lvgl PUBLIC LV_CONF_INCLUDE_SIMPLE=1)

set(COMP ${CMAKE_CURRENT_SOURCE_DIR}/../../components/balancer)
add_executable(ui_preview
  main.cpp
  ${COMP}/ui.cpp
  ${COMP}/ui_core.cpp
  ${COMP}/dsp_revolution.cpp
  ${COMP}/dsp_measurement.cpp
  ${COMP}/dsp_balance.cpp
  ${COMP}/dsp_job.cpp
  ${COMP}/dsp_sim.cpp
)
target_include_directories(ui_preview PRIVATE ${COMP})
target_compile_options(ui_preview PRIVATE -Wall -Wextra)
target_link_libraries(ui_preview lvgl m)
