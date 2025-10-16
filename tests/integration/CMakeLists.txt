# ==============================================================================
# Integration Tests (Multiple Components Together)
# ==============================================================================
# Tests that verify interaction between multiple firmware components.

function(add_integration_test TEST_NAME)
    add_executable(${TEST_NAME} ${ARGN})
    
    target_link_libraries(${TEST_NAME} PRIVATE
        gtest_main
        firmware_mocks
        firmware_lib
        hal_mocks          # HAL mocks for hardware simulation
    )
    
    target_compile_definitions(${TEST_NAME} PRIVATE
        UNIT_TEST
        PICO_PLATFORM=host
    )
    
    target_include_directories(${TEST_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/src/config
        ${CMAKE_CURRENT_SOURCE_DIR}/..
    )
    
    gtest_discover_tests(${TEST_NAME}
        PROPERTIES LABELS "integration"
        DISCOVERY_TIMEOUT 10
    )
endfunction()

# Define integration tests
add_integration_test(test_mecanum_kinematics test_mecanum_kinematics.cpp)
add_integration_test(test_odometry test_odometry.cpp)
add_integration_test(test_topic_names test_topic_names.cpp)
