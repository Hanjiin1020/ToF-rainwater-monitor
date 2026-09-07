cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/external/esp-idf-sx126x/components/ra01s"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tof_height)
