/*
 * Agent.h
 *
 * Abstract agent interface to an active agent object that runs as
 * FreeRTOS task
 *
 *  Created on: 15 Aug 2022
 *      Author: jondurrant
 */

#ifndef SRC_AGENT_H_
#define SRC_AGENT_H_

#define MAX_NAME_LEN 20

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief Abstract base class for FreeRTOS task-based agents
 * 
 * The Agent class provides a standardized interface for creating and managing
 * FreeRTOS tasks. Each agent encapsulates a specific functionality that runs
 * as an independent task with its own stack and priority.
 * 
 * Derived classes must implement:
 * - run() - The main task loop
 * - getMaxStackSize() - Stack size requirements
 * 
 * @note This class follows the Template Method pattern where the task lifecycle
 *       is managed by the base class while specific behavior is defined by subclasses.
 */
class Agent {
public:
	/**
	 * @brief Construct a new Agent object
	 * 
	 * Initializes the agent with no active task. The task is created when start() is called.
	 */
	Agent();

	/**
	 * @brief Destroy the Agent object
	 * 
	 * Ensures the FreeRTOS task is properly stopped before destruction.
	 * 
	 * @warning If the task is still running, it will be stopped during destruction.
	 */
	virtual ~Agent();

	/**
	 * @brief Start the agent as a FreeRTOS task
	 * 
	 * Creates and starts a new FreeRTOS task that will execute the run() method.
	 * The task name is limited to MAX_NAME_LEN (20) characters.
	 * 
	 * @param name Task name for debugging and monitoring (max 20 characters)
	 * @param priority FreeRTOS task priority (0 = idle priority, higher = more priority)
	 * 
	 * @return true if task was created successfully
	 * @return false if task creation failed (e.g., insufficient memory)
	 * 
	 * @pre The agent must not already have a running task
	 * @post If successful, the task is running and executing run()
	 * 
	 * @note The task stack size is determined by getMaxStackSize()
	 * @note Default priority is tskIDLE_PRIORITY (0)
	 */
	virtual  bool start(const char *name, UBaseType_t priority = tskIDLE_PRIORITY);

	/**
	 * @brief Stop the agent's FreeRTOS task
	 * 
	 * Terminates the FreeRTOS task associated with this agent. The task will
	 * be deleted and its resources freed.
	 * 
	 * @post The task handle is set to NULL
	 * @post The run() method will no longer be executing
	 * 
	 * @warning This is a forceful termination. Ensure the task is in a safe state
	 *          before calling stop() to avoid resource leaks.
	 */
	virtual void stop();


	/**
	 * @brief Get the stack high water mark for the task
	 * 
	 * Returns the minimum amount of stack space that has remained unused since
	 * the task started. This is useful for monitoring stack usage and detecting
	 * potential stack overflow conditions.
	 * 
	 * @return unsigned int Minimum free stack space in words (not bytes)
	 * 
	 * @note A value close to zero indicates high risk of stack overflow
	 * @note Monitor this value during development to tune getMaxStackSize()
	 * @note Returns 0 if the task is not running
	 * 
	 * @warning Stack overflow can cause system crashes and undefined behavior
	 */
	virtual unsigned int getStakHighWater();

	/**
	 * @brief Get the FreeRTOS task handle
	 * 
	 * Returns the underlying FreeRTOS task handle for this agent. This can be
	 * used with FreeRTOS API functions for advanced task management.
	 * 
	 * @return TaskHandle_t The FreeRTOS task handle, or NULL if task not started
	 * 
	 * @note The handle is valid only while the task is running
	 * @note Use with caution - direct manipulation may interfere with agent lifecycle
	 */
	virtual TaskHandle_t getTask();

protected:
	/**
	 * @brief Static entry point for FreeRTOS task
	 * 
	 * This static method is called by FreeRTOS when the task starts. It casts
	 * the pvParameters back to an Agent pointer and calls the virtual run() method.
	 * 
	 * @param pvParameters Pointer to the Agent object (passed during task creation)
	 * 
	 * @note This is an internal method used by the start() implementation
	 * @note Do not call this method directly
	 */
	static void vTask( void * pvParameters );

	/**
	 * @brief Main task execution loop (pure virtual)
	 * 
	 * This method contains the main logic for the agent and is executed continuously
	 * by the FreeRTOS task. Derived classes must implement this method to define
	 * the agent's behavior.
	 * 
	 * @note This method typically contains an infinite loop with vTaskDelay() calls
	 * @note The method should never return under normal operation
	 * @note Use vTaskDelay() or blocking operations to yield to other tasks
	 * 
	 * @warning Blocking indefinitely without yielding will starve other tasks
	 */
	virtual void run()=0;

	/**
	 * @brief Get the required stack size for this agent (pure virtual)
	 * 
	 * Derived classes must implement this method to specify the stack size
	 * required for their task. The value is in words (not bytes).
	 * 
	 * @return configSTACK_DEPTH_TYPE Stack size in words
	 * 
	 * @note Stack size is in words (typically 4 bytes per word on ARM Cortex-M)
	 * @note Use getStakHighWater() during development to tune this value
	 * @note Insufficient stack size will cause stack overflow and system crashes
	 * 
	 * @warning Always allocate more stack than the minimum to account for interrupts
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize()=0;

	/// FreeRTOS task handle (NULL if task not started)
	TaskHandle_t xHandle = NULL;

	/// Task name buffer (max MAX_NAME_LEN characters)
	char pName[MAX_NAME_LEN];


};


#endif /* SRC_AGENT_H_ */
