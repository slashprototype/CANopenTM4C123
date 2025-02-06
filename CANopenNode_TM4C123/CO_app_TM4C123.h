/*
 * @file        CO_app_TM4C123.h
 * @brief       Application file for CANopenNodeTM4C123 Library
 * 
 * @details     This file contains the application functions for initializing and running
 *              the CANopenNode library on the TM4C123 microcontroller.
 * 
 * @project     CANopen TM4C123 Application
 * @date        1 Jan 2025
 * @version     1.0.0
 * @author      Slashprototype
 */


#ifndef CO_APP_TM4C_H_
#define CO_APP_TM4C_H_

/* Let this definition here, is essential to activate master functionalities*/
#include "CO_driver_target.h"
#include "301/CO_NMT_Heartbeat.h"
#include "CANopen.h"
#include "OD.h"

#define FIRST_HB_TIME 500U
/* TODO: Define this value with your desired timeout in miliseconds for SDOs timeout */
#ifndef SDO_SRV_TIMEOUT_TIME
#define SDO_SRV_TIMEOUT_TIME 50000U
#endif
#ifndef SDO_CLI_TIMEOUT_TIME
#define SDO_CLI_TIMEOUT_TIME 50000U
#endif

#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS NULL
#define CO_FIFO_ENABLE true    /* Enable FIFO for CAN TX and RX */

uint16_t NMT_CONTROL;
/*
 * User defined structure that contains all the configurations and definitions to use MCU CAN controller
 * and CANopen Features, an instance of this object is received by some functions by the name of "void *CANptr"
 * Note: This is a pointer to save memory because an instance of this structure will consume a lot!!
 */
typedef struct
{
    /*Specific CAN controller vars*/
    uint32_t can_bit_rate;  /* Baud rate selected for MCU CAN controller initialization*/
    uint16_t co_bit_rate;   /* Baud rate selected for MCU CAN controller initialization*/
    uint32_t can_base;      /* CAN controller selected, usually is CAN0_BASE const in TM4C*/
    bool_t b_rx_flag;       /* Flag to debug when new RX msg has been received*/
    bool_t b_tx_flag;       /* Flag to debug when new TX msg has been sent */
    uint32_t msg_count;     /* Track all the RX received, increment it in CAN Isr handler*/
    /*Errors Secction*/
    bool_t b_can_bus_error; /* Flag activated when unexpected ISR has been activated, usually an error*/
    bool_t b_can_bus_off;
/* IN PROGRESS*/
#if CO_FIFO_ENABLE
    uint8_t can_rx_fifo_buffer_size;    /*This is the size of the RX FIFO buffer*/
    uint8_t can_tx_buffer_size;         /*This is the size of the TX FIFO buffer*/
    uint16_t can_tx_buffer_status;      /*This is a binary representation for CAN TX Msg Objects Availability, bit 0 = TX MSG OBJ ID: n0*/
    bool_t b_can_rx_fifo_is_full;       /*This flag is activated when the RX FIFO buffer is full*/
    bool_t b_can_tx_fifo_is_full;       /*This flag is activated when the TX FIFO buffer is full*/
#endif
    tCANMsgObject can_msg_rx;   /* CAN Msg Object instance to store transmitted msg*/
    tCANMsgObject can_msg_tx;   /* CAN Msg Object instance to store transmitted msg*/
    void (*canHandleIsr)();    /* Pointer to CAN controller ISR handler function*/
    void (*canInitFunction)(); /* Pointer to CAN controller initialization function*/
    /*CANopen members*/
    uint8_t desired_node_id;    /* CAN open take this id just if none active id has been defined at Object Dictionary*/
    uint8_t active_node_id;     /* Assigned Node ID */
    uint8_t co_red_led_value;   /* This will be updated by the stack - Use them for red led management*/
    uint8_t co_green_led_value; /* This will be updated by the stack - Use them for green led management*/
    CO_t *canopen_stack;        /* CO_t *CO_new(CO_config_t *config, uint32_t *heapMemoryUsed) in CANopen.c, Create new CANopen object */

} co_node_tm4c_t;

/* This variable is enabled globally and store all CANopen stuff */
extern co_node_tm4c_t *co_node_tm4c;

/* Initialize the CAN open Stack verifying that heap memory is enough to handle this operation */
CO_ReturnError_t CO_appInit(co_node_tm4c_t *p_node);
/*
 * This function map and initialize all the Object Dictionary structures,
 * use all the CAN module/controller parameters to configuration and setup
 */
CO_ReturnError_t CO_appResetCommunication(void);
/*
 * The "no timer" depending CANopen functionalities, like NMT, HB, SDO process
 * This are defined as polling routines and need to be verified periodically
 * The recommendation is to use it inside the main while loop.
 */
void CO_appProcess(void);

/* Timer dependent process to ensure CANopen devices time syncronization every 1ms*/
void co_appInterrupt(uint32_t sys_timer);
/*
 * Handle the @ref OD_extension_init for Change Of State (COS) OD entries
 * That are inside TPDOs, the application function @ref set_var_value will
 * Trigger the @ref OD_requestTPDO to update FLAGS in Event Transmission TPOs
 */
CO_ReturnError_t initODExtensions(uint16_t *index_arr, uint8_t size);

#endif /* CO_APP_TM4C_H_ */
