/*
 * @file        CO_driver_TM4C123.c
 * @brief       Driver file for CANopenNodeTM4C123 Library
 * 
 * @details     This file contains the driver functions for initializing and running
 *              the CANopenNode library on the TM4C123 microcontroller.
 * 
 * @project     CANopen TM4C123 Driver
 * @date        1 Jan 2025
 * @version     1.0.0
 * @author      Slashprototype
 */

#include "CO_app_TM4C123.h"
#include <stdint.h>
#include <stdio.h>
#include "301/CO_driver.h"
#include "driverlib/can.h"

extern co_node_tm4c_t *co_node_tm4c;

#define CANID_MASK 0x07FF
#define FLAG_RTR 0x8000

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr)
{
    /* Put CAN module in configuration mode */
    if (((co_node_tm4c_t *)CANptr) != NULL)
    {
        /* turn off the module */
        CANDisable(((co_node_tm4c_t *)CANptr)->can_base);
    }
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    /* Put CAN module in normal mode */
    CANmodule->CANnormal = true;
    CANEnable(((co_node_tm4c_t *)CANmodule->CANptr)->can_base);
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
    CO_CANmodule_t *CANmodule,
    void *CANptr,
    CO_CANrx_t rxArray[],
    uint16_t rxSize,
    CO_CANtx_t txArray[],
    uint16_t txSize,
    uint16_t CANbitRate)
{
    uint16_t i;

    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL)
    {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false; /* we have not implemented hw filters */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    for (i = 0U; i < rxSize; i++)
    {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (i = 0U; i < txSize; i++)
    {
        txArray[i].bufferFull = false;
    }

    /* Configure CAN module registers */
    ((co_node_tm4c_t *)CANptr)->canInitFunction(CANmodule->CANptr);

    /* Configure CAN timing */

    /* Configure CAN module hardware filters */
    if (CANmodule->useCANrxFilters)
    {
        /* CAN module filters are used, they will be configured with */
        /* CO_CANrxBufferInit() functions, called by separate CANopen */
        /* init functions. */
        /* Configure all masks so, that received message must match filter */
    }
    else
    {
        /* CAN module filters are not used, all messages with standard 11-bit */
        /* identifier will be received */
        /* Configure mask 0 so, that all messages with standard identifier are accepted */
    }

    /* configure CAN interrupt registers */

    return CO_ERROR_NO;
}

/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    if (CANmodule != NULL)
    {
        /* turn off the module */
        CANDisable(((co_node_tm4c_t *)CANmodule->CANptr)->can_base);
    }
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
    CO_CANmodule_t *CANmodule,
    uint16_t index,
    uint16_t ident,
    uint16_t mask,
    bool_t rtr,
    void *object,
    void (*CANrx_callback)(void *object, void *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if ((CANmodule != NULL) && (object != NULL) && (CANrx_callback != NULL) && (index < CANmodule->rxSize))
    {
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = (ident & CANID_MASK) | (rtr ? FLAG_RTR : 0x00);
        buffer->mask = (mask & CANID_MASK) | FLAG_RTR;

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters)
        {
        }
    }
    else
    {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}

/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
    CO_CANmodule_t *CANmodule,
    uint16_t index,
    uint16_t ident,
    bool_t rtr,
    uint8_t noOfBytes,
    bool_t syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if ((CANmodule != NULL) && (index < CANmodule->txSize))
    {
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
         * Microcontroller specific. */
        buffer->ident = ((uint32_t)ident & CANID_MASK) | ((uint32_t)(rtr ? FLAG_RTR : 0x00));
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}

/*****************************************************************************/

static uint8_t prv_send_can_message(co_node_tm4c_t *p_node, CO_CANtx_t *buffer)
{
    uint8_t success = 0;
    uint16_t buffer_status_value = 0U;
    uint32_t can_tx_channel_id = p_node->can_tx_buffer_size + 1;
    p_node->can_msg_tx.ui32MsgID = buffer->ident;
    p_node->can_msg_tx.ui32MsgIDMask = 0U;
    p_node->can_msg_tx.ui32MsgLen = buffer->DLC;

    /* Clear Data */
    for (int i = 0; i < 8; i++)
    {
        p_node->can_msg_tx.pui8MsgData[i] = 0;
    }
    memcpy(p_node->can_msg_tx.pui8MsgData, buffer->data, sizeof(uint8_t) * buffer->DLC);
    
    /* Use a binary 16 bits variable in witch every bit is corresponding to msg flag pending
     * 0 = free... so use this message channel if is the first message, msg channel will be 17 -> last could be 32
     * 1 = pending... so shift next flag and if it is free turn it on and send msg in next msg Channel */
    for ( uint8_t i = 0; i < p_node->can_tx_buffer_size && can_tx_channel_id <= 32 ; i++ )
    {
        if (p_node->can_tx_buffer_status == 0xFFFF)
        {
            p_node->b_can_tx_fifo_is_full = true;
            p_node->can_tx_buffer_status = 0;
        }
        buffer_status_value = 0U;
        buffer_status_value |= 1 << i; /* Shifting bit to left 0b0001 -> 0b0010 -> 0b0100 ...*/

        if (p_node->can_tx_buffer_status & buffer_status_value) /*When bit is turned on, it means TX msg ISR hasn't succeed yet, so +1 to channel ID*/
        {
            can_tx_channel_id += 1;
        }
        else    /* If bit is off use last channel ID (CAN Msg Object ID) to transmit the message and set bit on indicating new msg are pending*/
        {
            p_node->can_tx_buffer_status |= buffer_status_value;
            break;
        }
    }
    CANMessageSet(p_node->can_base, can_tx_channel_id, &p_node->can_msg_tx, MSG_OBJ_TYPE_TX);
    success = 1;
    return success;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;
    co_node_tm4c_t *p_node = (co_node_tm4c_t *)CANmodule->CANptr;

    /* Verify overflow */
    if (buffer->bufferFull)
    {
        if (!CANmodule->firstCANtxMessage)
        {
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND(CANmodule);
    /* if CAN TX buffer is free, copy message to it */
    if (prv_send_can_message(p_node, buffer) && CANmodule->CANtxCount == 0)
    {
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
        /* copy message and txRequest */
    }
    /* if no buffer is free, message will be sent by interrupt */
    else
    {
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/*messageIsOnCanBuffer && */ CANmodule->bufferInhibitFlag)
    {
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount != 0U)
    {
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for (i = CANmodule->txSize; i > 0U; i--)
        {
            if (buffer->bufferFull)
            {
                if (buffer->syncFlag)
                {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    if (tpdoDeleted != 0U)
    {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
 * different way to determine errors. */
static uint32_t rxErrors = 0, txErrors = 0, overflow = 0;

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    uint32_t err;
    co_node_tm4c_t *p_node = (co_node_tm4c_t *)CANmodule->CANptr;
    bool_t b_can_contoller_error = CANErrCntrGet(p_node->can_base, &rxErrors, &txErrors);

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

    if (CANmodule->errOld != err)
    {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        /* Wait until can_bus_error is false to authentically activate the TX FIFO BUFFER*/
        if( CANmodule->errOld < err ){  /*Error is incrementing, so reset buffer bits to avoid false msg flags turned on*/
            p_node->can_tx_buffer_status = 0;
        }

        if (txErrors >= 256U)
        {
            /* bus off */
            status |= CO_CAN_ERRTX_BUS_OFF;
        }
        else
        {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF |
                                CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                                CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128)
            {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            }
            else if (rxErrors >= 96)
            {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128)
            {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            }
            else if (rxErrors >= 96)
            {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0)
            {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        if (overflow != 0)
        {
            /* CAN RX bus overflow */
            status |= CO_CAN_ERRRX_OVERFLOW;
        }

        CANmodule->CANerrorStatus = status;
    }
    else{
        b_can_contoller_error = false;
    }
}

/******************************************************************************/
typedef struct
{
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
} CO_CANrxMsg_t;

/******************************************************************************/
void co_processRxIsr(CO_CANmodule_t *CANmodule)
{

    co_node_tm4c_t *p_node = ((co_node_tm4c_t *)CANmodule->CANptr);

    tCANMsgObject *rcvMsg;     /* pointer to received message in CAN module */
    uint16_t index;            /* index of received message */
    uint32_t rcvMsgIdent;      /* identifier of the received message */
    CO_CANrx_t *buffer = NULL; /* receive message buffer from CO_CANmodule_t object. */
    bool_t msgMatched = false;

    rcvMsg = &p_node->can_msg_rx; /* get message from module here */
    rcvMsgIdent = rcvMsg->ui32MsgID;
    if (CANmodule->useCANrxFilters)
    {
        /* CAN module filters are used. Message with known 11-bit identifier has */
        /* been received */
        index = 0; /* get index of the received message here. Or something similar */
        if (index < CANmodule->rxSize)
        {
            buffer = &CANmodule->rxArray[index];
            /* verify also RTR */
            if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U)
            {
                msgMatched = true;
            }
        }
    }
    else
    {
        /* CAN module filters are not used, message with any standard 11-bit identifier */
        /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
        buffer = &CANmodule->rxArray[0];
        for (index = CANmodule->rxSize; index > 0U; index--)
        {
            if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U)
            {
                msgMatched = true;
                break;
            }
            buffer++;
        }
    }

    /* Call specific function, which will process the message */
    if (msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL))
    {
        buffer->CANrx_callback(buffer->object, (void *)rcvMsg);
    }

    co_node_tm4c->b_can_bus_error = 0;
}

void co_processTxIsr(CO_CANmodule_t *CANmodule)
{
    /* First CAN message (bootup) was sent successfully */
    CANmodule->firstCANtxMessage = false;
    /* clear flag from previous message */
    CANmodule->bufferInhibitFlag = false;
    /* Are there any new messages waiting to be send */
    if (CANmodule->CANtxCount > 0U)
    {
        uint16_t i; /* index of transmitting message */

        /* first buffer */
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        /* search through whole array of pointers to transmit message buffers. */
        for (i = CANmodule->txSize; i > 0U; i--)
        {
            /* if message buffer is full, send it. */
            if (buffer->bufferFull)
            {
                buffer->bufferFull = false;
                CANmodule->CANtxCount--;

                /* Copy message to CAN buffer */
                CANmodule->bufferInhibitFlag = buffer->syncFlag;
                /* canSend... */
                break; /* exit for loop */
            }
            buffer++;
        } /* end of for loop */

        /* Clear counter if no more messages */
        if (i == 0U)
        {
            CANmodule->CANtxCount = 0U;
        }
    }
}
