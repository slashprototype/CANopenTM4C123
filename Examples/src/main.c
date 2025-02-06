/*
 * @file        main.c
 * @brief       Example main file for CANopenNodeTM4C123 Library
 * 
 * @details     This file contains the main function for initializing and running
 *              the CANopenNode library on the TM4C123 microcontroller.
 * 
 * @project     CANopen TM4C123 Library
 * @date        1 Jan 2025
 * @version     1.0.0
 * @author      Slashprototype
 */

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driver/can_drivers.h"

int main(void)
{
    /* MCU Clock settings*/
    SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ | SYSCTL_USE_PLL | SYSCTL_SYSDIV_5); /*40 MHz*/

    /* Get the system clock value to configure CAN peripherals */
    uint32_t sys_clock_value = SysCtlClockGet();
    
    /* CANopenNode for TM4C123 instance*/
    co_node_tm4c_t co_node_tm4c;

    /* CAN driver initialization for Hardware controller and CANopen Stack*/
    canopenInit(&co_node_tm4c, sys_clock_value);

    /* Enable interrupts to the processor*/
    IntMasterEnable();
    
    /* Main loop for CANopen process*/
    while (true)
    {
        /* CANopen process, generic functionalities declared */
        CO_appProcess();
    }
}
