cmake_minimum_required(VERSION 3.16)
project(p4_balancer_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(COMP ${CMAKE_CURRENT_SOURCE_DIR}/../components/balancer)
set(DSP_SOURCES
  ${COMP}/dsp_revolution.cpp
  ${COMP}/dsp_measurement.cpp
  ${COMP}/dsp_balance.cpp
  ${COMP}/dsp_job.cpp
  ${COMP}/dsp_resample.cpp
  ${COMP}/dsp_sim.cpp
  ${COMP}/ui_core.cpp   # ModelCore: pure C++, shared with the firmware and ui_sim
)

add_library(balancer_dsp STATIC ${DSP_SOURCES})
target_include_directories(balancer_dsp PUBLIC ${COMP})
target_compile_options(balancer_dsp PRIVATE -Wall -Wextra -Werror)

add_executable(test_dsp test_dsp.cpp test_mpu.cpp)
target_include_directories(test_dsp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_dsp balancer_dsp)

add_executable(balance_cli ${CMAKE_CURRENT_SOURCE_DIR}/../tools/balance_cli.cpp)
target_link_libraries(balance_cli balancer_dsp)

# The touchscreen UI: plain C++, the same files the firmware builds
add_library(balancer_ui STATIC
  ${COMP}/ui.cpp
  ${COMP}/ui_screens.cpp
  ${COMP}/ui_kit.cpp
  ${COMP}/ui_canvas.cpp
  ${COMP}/ui_fonts_data.cpp
)
target_include_directories(balancer_ui PUBLIC ${COMP})
target_compile_options(balancer_ui PRIVATE -Wall -Wextra -Werror)
target_link_libraries(balancer_ui balancer_dsp)

add_executable(test_ui test_ui.cpp)
target_include_directories(test_ui PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../tools/ui_sim)
target_link_libraries(test_ui balancer_ui)

enable_testing()
add_test(NAME dsp COMMAND test_dsp)
add_test(NAME ui COMMAND test_ui)
