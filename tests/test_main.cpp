/**
 * @file test_main.cpp
 * @brief Shared GoogleTest main entry point
 * 
 * This replaces the manual main() in tests1.cpp.
 * All test executables link against this library.
 * 
 * Benefits:
 * - Consistent test setup across all test executables
 * - Access to all GoogleTest command-line flags
 * - Optional custom listeners/configuration
 */

#include <gtest/gtest.h>
#include <iostream>

/**
 * @brief Custom Test Event Listener (optional)
 * 
 * Provides minimal output during test runs.
 * Enable by uncommenting the listener registration below.
 */
class MinimalPrinter : public ::testing::EmptyTestEventListener {
public:
    void OnTestProgramStart(const ::testing::UnitTest& unit_test) override {
        std::cout << "\n";
        std::cout << "========================================\n";
        std::cout << "Running " << unit_test.test_to_run_count() 
                  << " tests from " << unit_test.test_suite_to_run_count() 
                  << " test suites.\n";
        std::cout << "========================================\n\n";
    }

    void OnTestEnd(const ::testing::TestInfo& test_info) override {
        if (test_info.result()->Passed()) {
            std::cout << "  ✓ " << test_info.test_suite_name() 
                      << "." << test_info.name() << "\n";
        }
    }

    void OnTestProgramEnd(const ::testing::UnitTest& unit_test) override {
        std::cout << "\n========================================\n";
        if (unit_test.failed_test_count() == 0) {
            std::cout << "✓ ALL TESTS PASSED\n";
        } else {
            std::cout << "✗ SOME TESTS FAILED\n";
        }
        std::cout << "========================================\n";
    }
};

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    // Optional: Custom Listener for minimal output
    // Uncomment to enable:
    
    ::testing::TestEventListeners& listeners =
        ::testing::UnitTest::GetInstance()->listeners();
    delete listeners.Release(listeners.default_result_printer());
    listeners.Append(new MinimalPrinter);
    
    
    return RUN_ALL_TESTS();
}
