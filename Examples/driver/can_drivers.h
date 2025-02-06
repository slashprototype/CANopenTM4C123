/*
 * @file        can_drivers.h
 * @brief       Header file for CAN driver functions for TM4C123 microcontroller
 * 
 * @details     This file contains the declarations of functions and variables 
 *              required for initializing and running the CAN module on the 
 *              TM4C123 microcontroller. It provides hardware configuration 
 *              functions, CAN controller initialization, and CANopen application 
 *              initialization.
 * 
 * @project     CANopen TM4C123 Application
 * @date        1 Jan 2025
 * @version     1.0.0
 * @author      Slashprototype
 */

#ifndef CAN_DRIVERS_H_
#define CAN_DRIVERS_H_

#include "301/CO_driver.h"
#include "CO_app_TM4C123.h"

extern volatile uint32_t sys_timer;

/**
 * @brief Sets the CAN module clock based on the system clock.
 *
 * This function configures the CAN module clock to match the provided
 * system clock frequency.
 *
 * @param sys_clock The system clock frequency in Hz.
 */
void setCANClock(uint32_t sys_clock);

/**
 * @brief Initializes the hardware peripherals required for the CAN controller.
 *
 * This function sets up the necessary hardware peripherals, such as GPIO
 * and CAN modules, required for the CAN controller to operate.
 */
void canHwInit(void);

/**
 * @brief Initializes or resets the CAN controller.
 *
 * This function is called within CO_CANmodule_init to initialize or reset
 * the CAN controller with the provided node structure.
 *
 * @param p_node Pointer to the CANopen node structure.
 */
void canControllerInit(co_node_tm4c_t* p_node);

/**
 * @brief Initializes the CANopen application.
 *
 * This function sets up the necessary structures and variables for the
 * CANopen application, configuring it to operate with the provided system
 * clock rate.
 *
 * @param p_node Pointer to the CANopen node structure.
 * @param sys_clock_rate The system clock rate in Hz.
 * @return CO_ReturnError_t Error code indicating the result of the initialization.
 */
CO_ReturnError_t canopenInit(co_node_tm4c_t* p_node, uint32_t sys_clock_rate);

#endif /* CAN_DRIVERS_H_ */
