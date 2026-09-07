cmake_minimum_required(VERSION 3.16)
project(tof_host_tests C)

enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(geometry_under_test
    ../../components/geometry/geometry.c
)
target_include_directories(geometry_under_test PUBLIC
    ../../components/geometry/include
)
target_link_libraries(geometry_under_test PUBLIC m)

foreach(test_name test_geometry test_height_est test_pipeline)
    add_executable(${test_name} ${test_name}.c)
    target_link_libraries(${test_name} PRIVATE geometry_under_test)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

add_executable(test_lora_protocol
    test_lora_protocol.c
    ../../components/lora_protocol/lora_protocol.c
)
target_include_directories(test_lora_protocol PRIVATE
    ../../components/lora_protocol/include
)
add_test(NAME test_lora_protocol COMMAND test_lora_protocol)
