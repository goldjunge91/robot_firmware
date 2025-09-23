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

## 3. Firmware Project Structure

**Task**: Find best practices for structuring a complex firmware project for a mobile robot on the RP2040.

**Context**: The project will involve multiple sensors, actuators, and communication interfaces. A good project structure is essential for maintainability.

**Acceptance Criteria**:
- The research should provide examples of well-structured firmware projects for similar robots.
- It should cover topics like code organization, modularity, and separation of concerns.