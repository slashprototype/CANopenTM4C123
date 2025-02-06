/*
 * @file        CO_app_TM4C123.c
 * @brief       Application file for CANopenNodeTM4C123 Library
 *
 * @details     This file contains the application functions for initializing and running
 *              the CANopenNode library on the TM4C123 microcontroller.
 *
 * @project     CANopen TM4C123 Application
 * @date        1 Jan 2025
 * @version     1.0.0
 * @author      Slashprototype
 * @license     MIT
 */

#include <stdbool.h>
#include <stdint.h>
#include "CO_app_TM4C123.h"
#include "driver/can_drivers.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "inc/hw_can.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/timer.h"
#include "tm4c_utils/timers.h"

uint8_t can_rx_data[8] = {0};
uint8_t can_tx_data[8] = {0};

tCANMsgObject can_rx_msg_obj;
tCANMsgObject can_tx_msg_obj;

timer_module_t can_timer;
uint32_t can_sys_clock;

volatile uint32_t sys_timer = 0;

static void CANIntHandler(void);

void canTimerIntHandler() {
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    sys_timer += 1;
    co_appInterrupt(sys_timer);
}

/* Configure CANopen Node instance according to your requirements*/
CO_ReturnError_t canopenInit(co_node_tm4c_t* p_node, uint32_t sys_clock_rate) {

    // CAN Hardware initialization
    can_sys_clock = sys_clock_rate;
    canHwInit();

    CO_ReturnError_t co_error;
    p_node->desired_node_id = CO_DESIRED_NODE_ID;
    p_node->can_base = CAN_BASE;
    p_node->can_bit_rate = CAN_BIT_RATE;
    p_node->co_bit_rate = CAN_BIT_RATE / 1000U;
    p_node->b_can_bus_error = true;
    p_node->b_can_bus_off = true;
    p_node->b_rx_flag = false;
    p_node->b_tx_flag = false;
    p_node->msg_count = 0;
    p_node->canHandleIsr = CANIntHandler;
    p_node->canInitFunction = &canControllerInit;
    p_node->b_can_tx_fifo_is_full = false;
    p_node->b_can_rx_fifo_is_full = false;
    p_node->can_rx_fifo_buffer_size = 16; /* CAN RX MSG Object FIFO Buffer, 16 CAN Msg Objects are implemented)*/
    p_node->can_tx_buffer_size = 16;      /* CAN TX MSG Object FIFO Buffer, 16 CAN Msg Objects are implemented)*/
    p_node->can_tx_buffer_status = 0x0000;

    co_error = CO_appInit(p_node);

    (co_error != CO_ERROR_NO) ? GPIOPinWrite(CAN_LEDS_PORT, CAN_RED_LED_PIN, 255U) : 0;

    return co_error;
}

#if CAN_LOOPBACK_MODE_ENABLED
static void canLoopBackModeEnable(uint32_t ui32Base) {
    uint32_t ui32CtlReg;
    uint32_t ui32TstReg;

    ui32CtlReg = HWREG(ui32Base + CAN_O_CTL);
    ui32CtlReg |= ((uint32_t)(1U) << 7U);
    HWREG(ui32Base + CAN_O_CTL) = ui32CtlReg;

    ui32TstReg = ((uint32_t)(1U) << 4U);
    HWREG(ui32Base + CAN_O_TST) = ui32TstReg;
}
#endif

void canHwInit(void) {
    /*SECTION: CANopen LEDs RED & GREEN */
    SysCtlPeripheralEnable(CAN_LEDS_PERIPH);
    while (!SysCtlPeripheralReady(CAN_LEDS_PERIPH))
        ;
    GPIOPinTypeGPIOOutput(CAN_LEDS_PORT, CAN_RED_LED_PIN | CAN_GREEN_LED_PIN);

    /* SECTION: Specific CAN Hardware*/
    SysCtlPeripheralEnable(CAN_SYSCTL_PERIPH_GPIO);
    SysCtlPeripheralEnable(CAN_SYSCTL_PERIPH);
    while (!SysCtlPeripheralReady(CAN_SYSCTL_PERIPH))
        ;
    GPIOPinTypeCAN(CAN_PORT, CAN_GPIO_PINS);
    GPIOPinConfigure(CAN_RX_PIN);
    GPIOPinConfigure(CAN_TX_PIN);

    // /* SECTION: Enable CANopen 1 ms Timer*/
    // SysTickPeriodSet(can_sys_clock / 1000);
    // SysTickIntEnable(); /* Enable the SysTick Interrupt */
    // SysTickEnable();    /* Enable the SysTick Timer */
    // SysTickIntRegister(&sysTickIntHandler);

    /* TIMER0 CONFIGURATION */
    can_timer.hw_timer_base = TIMER2_BASE;
    can_timer.sysctl_peripheral = SYSCTL_PERIPH_TIMER2;
    can_timer.timer_configure = TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC;
    can_timer.timer_name = TIMER_A;
    can_timer.timer_int_mode = TIMER_TIMA_TIMEOUT;
    can_timer.frequency_hz = 0;
    can_timer.prescaler = 0;
    can_timer.counter = 0;
    can_timer.isrFunction = &canTimerIntHandler;
    setupTimerModule(&can_timer);
    can_timer.configureFrequency(&can_timer, 1000);
    IntPrioritySet(INT_TIMER2A, 0xE0);
    can_timer.enable(&can_timer);
}

void canControllerInit(co_node_tm4c_t* p_node) {
    CANInit(p_node->can_base);
    CANBitRateSet(p_node->can_base, can_sys_clock, p_node->can_bit_rate);
    CANIntRegister(CAN_BASE, p_node->canHandleIsr); // if using dynamic vectors
    CANIntEnable(p_node->can_base, CAN_INT_MASTER | CAN_INT_STATUS | CAN_INT_ERROR);
    IntEnable(CAN_INT);
#if CAN_LOOPBACK_MODE_ENABLED
    canLoopBackModeEnable(p_node->can_base);
#endif
    CANEnable(p_node->can_base);

    /* CAN TX MSG Object initial definition*/
    can_tx_msg_obj.ui32MsgID = 0x00;
    can_tx_msg_obj.ui32MsgIDMask = 0x00;
    can_tx_msg_obj.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    can_tx_msg_obj.ui32MsgLen = 0x00;
    can_tx_msg_obj.pui8MsgData = can_tx_data;
    p_node->can_msg_tx = can_tx_msg_obj;

    /* CAN RX MSG Object Initial definitions*/
    can_rx_msg_obj.ui32MsgID = 0x00;
    can_rx_msg_obj.ui32MsgIDMask = 0x00;
    can_rx_msg_obj.ui32MsgLen = 0x00;
    can_rx_msg_obj.pui8MsgData = can_rx_data;
    p_node->can_msg_rx = can_rx_msg_obj;

    for (int i = 1; i <= p_node->can_rx_fifo_buffer_size; i++) {
        if (i <= p_node->can_rx_fifo_buffer_size - 1) {
            p_node->can_msg_rx.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER
                                           | MSG_OBJ_FIFO; /*MSG_OBJ_FIFO flag indicate is part of a FIFO*/
            CANMessageSet(p_node->can_base, i, &p_node->can_msg_rx, MSG_OBJ_TYPE_RX);
        } else {
            p_node->can_msg_rx.ui32Flags =
                MSG_OBJ_RX_INT_ENABLE
                | MSG_OBJ_USE_ID_FILTER; /* Not a MSG_OBJ_FIFO flag indicate is the last object of a FIFO*/
            CANMessageSet(p_node->can_base, i, &p_node->can_msg_rx, MSG_OBJ_TYPE_RX);
        }
    }
}

static void CANIntHandler(void) {
    uint32_t can_isr_status;
    uint32_t can_base = co_node_tm4c->can_base;
    can_isr_status = CANIntStatus(can_base, CAN_INT_STS_CAUSE); // Return 0x8000 for Status Interrupt in CAN_O_INT
    uint8_t can_msg_num_isr = can_isr_status; /*Just for better understand of CAN Msg Object Interruption*/

    /*TODO: Fix this logic and improve it*/
    if (can_isr_status == CAN_INT_INTID_STATUS) {
        can_isr_status = CANStatusGet(can_base, CAN_STS_CONTROL);
        if (can_isr_status & CAN_STATUS_BUS_OFF) { /*CAN transceiver is disconnected*/
            co_node_tm4c->can_tx_buffer_status = 0;
            co_node_tm4c->b_can_bus_off = true;
        }
    }

    /* Received CAN Message Interrupt if can is between 1-16*/
    else if (can_msg_num_isr <= co_node_tm4c->can_rx_fifo_buffer_size) {
        /* Receive CAN Message FIFO Buffer Algorithm 1*/
        for (uint8_t i = 0; i < co_node_tm4c->can_rx_fifo_buffer_size; i++) {
            if (can_msg_num_isr > co_node_tm4c->can_rx_fifo_buffer_size) {
                co_node_tm4c->b_can_rx_fifo_is_full = true;
                break;
            }
            co_node_tm4c->b_can_rx_fifo_is_full = false;
            CANMessageGet(co_node_tm4c->can_base, can_msg_num_isr, &co_node_tm4c->can_msg_rx, true);
            if (co_node_tm4c->can_msg_rx.ui32Flags & MSG_OBJ_NEW_DATA) {
                co_node_tm4c->can_msg_rx.ui32Flags &= ~MSG_OBJ_NEW_DATA;
                co_node_tm4c->msg_count += 1;
                co_processRxIsr(
                    (co_node_tm4c->canopen_stack)->CANmodule); /* CAN open function to process this RX interruption*/
                can_msg_num_isr += 1;
            } else {
                break; /* If no new message received just break the for loop */
            }
        }
        co_node_tm4c->b_rx_flag = true;        /*usefull flag, use it for development*/
        co_node_tm4c->b_can_bus_error = false; /* Turn off the can bus error flag*/
        co_node_tm4c->b_can_bus_off = false;
    }
    /* Transmit CAN Message Interrupt if can isr > 16 and <= 32*/
    else if (can_msg_num_isr > co_node_tm4c->can_tx_buffer_size && can_msg_num_isr <= 32) {
        CANIntClear(co_node_tm4c->can_base, can_msg_num_isr); /* Clear interrupt flag */
        /*Clear the tx buffer status 16 bits array corresponding to isr number*/
        uint8_t bit_number_shift = (can_msg_num_isr - (co_node_tm4c->can_tx_buffer_size + 1));
        co_node_tm4c->can_tx_buffer_status &= ~(1 << bit_number_shift);
        co_processTxIsr(
            (co_node_tm4c->canopen_stack)->CANmodule); /* CAN open function to process this TX interruption*/
        co_node_tm4c->b_tx_flag = true;                /*useful flag, use it for development*/
        co_node_tm4c->b_can_bus_error = false;         /* Turn off the can bus error flag*/
        co_node_tm4c->b_can_bus_off = false;
    }
}
