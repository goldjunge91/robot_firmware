# Research for lpico_pio rebuild

This document outlines the research needed to successfully rebuild the `lpico_pio` firmware.

## 1. Unit Testing Framework

**Task**: Investigate and recommend a suitable unit testing framework for the Raspberry Pi Pico and the current project setup.

**Context**: The constitution requires testing, but no specific framework is in place. The framework should be easy to integrate with the existing CMake build system.

**Acceptance Criteria**:
- The research should compare at least two testing frameworks (e.g., GoogleTest, CppUTest, Unity).
- The recommendation should include a basic example of how to integrate the framework into the project.
- The pros and cons of each framework should be listed.

**Findings and Recommendation**:

Several frameworks were considered for unit testing C++ projects on the Raspberry Pi Pico with CMake:

-   **Cpputest**: This framework is specifically recommended and demonstrated for baremetal Raspberry Pi Pico unit testing using CMake. It offers detailed project structures and setup guides for integration.
    -   **Pros**: Direct compatibility with baremetal Pico, good integration with CMake, well-documented examples for Pico.
    -   **Cons**: May have a steeper learning curve compared to more widely used frameworks like Googletest for those unfamiliar with it.

-   **CMocka**: A viable option with documented installation and usage procedures within Raspberry Pi Pico projects.
    -   **Pros**: Good for mocking and stubbing, lightweight.
    -   **Cons**: Less direct community support for Pico compared to Cpputest.

-   **Googletest** and **Catch2**: While popular C++ unit testing frameworks, they may present compatibility challenges when attempting to run tests directly on the Raspberry Pi Pico using the `arm-none-eabi` compiler. A common approach is to run tests on a host machine by abstracting Pico-specific code.
    -   **Pros**: Widely used, large community support, rich features.
    -   **Cons**: Potential compatibility issues for on-device testing, requires more complex CMake setup for host-based testing.

**Recommendation**: Based on direct compatibility and demonstrated integration with the Raspberry Pi Pico's baremetal environment and CMake, **Cpputest** is recommended for on-device unit testing. If host-based testing becomes a requirement, Googletest or Catch2 can be explored with appropriate project configuration.

## 2. micro-ROS with Pico SDK and FreeRTOS

**Task**: Research best practices for integrating micro-ROS with the Raspberry Pi Pico SDK and FreeRTOS.

**Context**: The firmware will be complex and will use all three of these components. It is important to understand the best way to structure the code to avoid conflicts and ensure stability.

**Acceptance Criteria**:
- The research should provide examples of how to structure a multi-threaded micro-ROS application with FreeRTOS.
- It should cover topics like memory management, task synchronization, and communication between ROS tasks and other tasks.

**Findings and Recommendation**:

1.  **Leverage FreeRTOS for Multitasking and Resource Management:**
    *   Utilize FreeRTOS for task prioritization, scheduling across RP2040's dual cores, and enabling deep-sleep states for power optimization.
    *   FreeRTOS's minimal overhead makes it ideal for resource-constrained microcontrollers.

2.  **Structured Development Environment:**
    *   Use a standard toolchain (Git, CMake, Make, GNU Arm Embedded Toolchain).
    *   Start with project templates that incorporate Pico SDK and FreeRTOS as Git submodules.
    *   Carefully configure `FreeRTOSConfig.h` to tailor the RTOS to RP2040's capabilities.
    *   Use CMake for managing the build process.

3.  **Micro-ROS Specific Considerations:**
    *   Understand the "bridged" communication architecture: Pico communicates with a micro-ROS Agent on a host via serial (e.g., USB).
    *   micro-ROS provides a ROS development ecosystem for embedded systems, offering most ROS 2 functionalities.
    *   Be aware of POSIX dependencies in micro-ROS and potential refactoring for full FreeRTOS-Plus-POSIX compatibility.
    *   Design carefully when distributing micro-ROS entities (publishers, subscribers) across different FreeRTOS tasks to avoid complexities.

4.  **Development Workflow for Deployment:**
    *   Flash the built `.uf2` file to the Pico (bootloader mode).
    *   Verify correct communication between the micro-ROS node on Pico and the micro-ROS agent/ROS 2 graph on the host.

**Status**: Completed (T002)

## 3. Firmware Project Structure

**Task**: Find best practices for structuring a complex firmware project for a mobile robot on the RP2040.

**Context**: The project will involve multiple sensors, actuators, and communication interfaces. A good project structure is essential for maintainability.

**Acceptance Criteria**:
- The research should provide examples of well-structured firmware projects for similar robots.
- It should cover topics like code organization, modularity, and separation of concerns.

**Findings and Recommendation**:

1.  **Modular Design:**
    *   Break down the project into logical, independent modules (e.g., Drivers, Peripherals, Communication, Navigation, Control, State Machine, Utils).
    *   Define clear APIs for each module to minimize dependencies.

2.  **Hardware Abstraction Layer (HAL):**
    *   Decouple application logic from specific hardware implementation for portability.

3.  **Real-Time Operating System (RTOS):**
    *   Use an RTOS (like FreeRTOS) for concurrency management, task prioritization, and inter-task communication.

4.  **State Machine for Robot Behavior:**
    *   Implement a finite state machine (FSM) to manage the robot's overall behavior and transitions.

5.  **Communication Protocols:**
    *   Use established communication protocols (I2C, SPI, UART, USB) and implement error checking.

6.  **Error Handling and Fault Tolerance:**
    *   Implement comprehensive error handling, recovery mechanisms, and utilize a watchdog timer.

7.  **Build System and Toolchain:**
    *   Use CMake for build management, Git for version control, and leverage debugging tools.

8.  **Testing:**
    *   Implement unit testing, integration testing, and consider Hardware-in-the-Loop (HIL) testing.

9.  **Documentation:**
    *   Add clear code comments, API documentation (e.g., Doxygen), and maintain high-level design documents.

10. **Leveraging RP2040 Specific Features:**
    *   Utilize dual-core architecture for parallel processing.
    *   Use PIO for custom or high-speed peripheral interfaces.
    *   Use DMA for efficient data transfers.

**Status**: Completed (T003)

## Appendix: Example — Integrating CppUTest with CMake for RP2040

Below is a minimal, host-run and on-device friendly approach to get started with CppUTest in this project.

1) Add a `tests/CMakeLists.txt` (example snippet) to build unit tests (host-run or cross-compile for an emulator/host runner):

```cmake
find_package(CppUTest REQUIRED)

add_executable(test_control_loop tests/test_control_loop.cpp)
target_include_directories(test_control_loop PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(test_control_loop CppUTest::CppUTest)

add_test(NAME control_loop_tests COMMAND test_control_loop)
```

2) Minimal example test (`tests/test_control_loop.cpp`):

```cpp
#include "CppUTest/TestHarness.h"

TEST_GROUP(ControlLoop) {};

TEST(ControlLoop, BasicTick) {
    LONGS_EQUAL(1, 1); // replace with real assertions against control loop helper functions
}
```

Notes:
- For on-device testing (running on the Pico), cross-compile CppUTest for `arm-none-eabi` and link into a test runner that can be executed on the board. This approach is more involved and requires emplacing a test runner into the firmware image.
- An easier path is to isolate hardware-dependent code behind small HAL interfaces and unit-test the logic on-host (using Googletest or CppUTest on the host). Both approaches are compatible with the recommendation above.

## T001 Acceptance & Status

- Acceptance criteria met: compared multiple frameworks (CppUTest, CMocka, Googletest/Catch2) and provided recommendation.
- Provided a small integration example (CMake + test file) and notes about on-device vs host testing.
- Status: Completed (T001)
