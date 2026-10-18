cmake_minimum_required(VERSION 3.16)
# Offline hardware test against the local driver and diagnostics checkout.
set(ENV{IDF_COMPONENT_MANAGER} "0")
set(COMPONENTS main)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../.."
                         "${CMAKE_CURRENT_LIST_DIR}/../../../esp_rt_diagnostics")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mt6701_dual_i2c_test)
