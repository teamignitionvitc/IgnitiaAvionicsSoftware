# Test suite CMakeLists.txt

# Unity test framework (lightweight)
add_library(unity STATIC
    unity/unity.c
)
target_include_directories(unity PUBLIC unity)

# Test executables
add_executable(test_flight_state
    test_flight_state.c
    ${CMAKE_SOURCE_DIR}/src/modules/flight_state.c
)
target_include_directories(test_flight_state PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/modules
)
target_link_libraries(test_flight_state unity pico_stdlib)

add_executable(test_deployment
    test_deployment.c
)
target_include_directories(test_deployment PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/modules
    ${CMAKE_SOURCE_DIR}/src/drivers
)
target_link_libraries(test_deployment unity pico_stdlib hardware_pwm)

add_executable(test_filters
    test_filters.c
    ${CMAKE_SOURCE_DIR}/src/utils/filters.c
)
target_include_directories(test_filters PRIVATE
    ${CMAKE_SOURCE_DIR}/src/utils
)
target_link_libraries(test_filters unity)

add_executable(test_ring_buffer
    test_ring_buffer.c
    ${CMAKE_SOURCE_DIR}/src/utils/ring_buffer.c
)
target_include_directories(test_ring_buffer PRIVATE
    ${CMAKE_SOURCE_DIR}/src/utils
)
target_link_libraries(test_ring_buffer unity)
