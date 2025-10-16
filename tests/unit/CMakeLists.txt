# ==============================================================================
# Unit Tests (Isolated Component Tests)
# ==============================================================================
# Each test is a separate executable for:
# - Parallel execution
# - Individual debugging
# - Granular test control

# Helper function to add unit tests
function(add_unit_test TEST_NAME)
    # Create executable from provided source files
    add_executable(${TEST_NAME} ${ARGN})
    
    # Link dependencies
    target_link_libraries(${TEST_NAME} PRIVATE
        gtest_main         # GoogleTest with main()
        firmware_mocks     # Mock implementations
        firmware_lib       # Firmware code under test
    )
    
    # Compile definitions for tests
    target_compile_definitions(${TEST_NAME} PRIVATE
        UNIT_TEST
        PICO_PLATFORM=host
    )
    
    # Include directories
    target_include_directories(${TEST_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/src/config
        ${CMAKE_CURRENT_SOURCE_DIR}/..
    )
    
    # GoogleTest Discovery (automatically finds all TEST/TEST_F)
    gtest_discover_tests(${TEST_NAME}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        PROPERTIES LABELS "unit"
        DISCOVERY_TIMEOUT 10
    )
endfunction()

# Define unit tests
add_unit_test(test_motor_pid test_motor_pid.cpp)
add_unit_test(test_tb6612_motor_pid test_tb6612_motor_pid.cpp)
add_unit_test(test_basic_math test_basic_math.cpp)
