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
 * 
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "CO_app_TM4C123.h"

/* default values for CO_CANopenInit() */



/* Pointer to CANopen Generic Object used locally */
CO_t* CO = NULL;

/* Pointer to Device CANopen Object used locally */
co_node_tm4c_t* co_node_tm4c;

/* Timers for CANopen App handling miliseconds routines*/
uint32_t time_old = 0;
uint32_t time_current = 0;

static CO_ReturnError_t err;    /* Error variable for CANopen functions */

CO_ReturnError_t CO_appInit(co_node_tm4c_t* p_node) {
    co_node_tm4c = p_node; // Update co_node_tm4c global
    CO_config_t* config_p = NULL;
    uint32_t heap_memory_used;

    /*Initialize CO object (canopen stack) and verify the heap memory used
     * It should return a valid CO initialized object, in other way the memory
     * is not enough and need config the arm linker to increase heap memory
     */
    CO = CO_new(config_p, &heap_memory_used);

    //    OD_extension_t od_extension;
    //    od_extension.flagsPDO[0] = 0b00000001;
    //    OD_extension_init(OD_find(OD, 0x2401), &od_extension);

    if (CO == NULL) {
        return CO_ERROR_OUT_OF_MEMORY; // Error, can't allocate memory
    }

    /* Assign the CO object initialized as the canopen_stack member of device struct*/
    co_node_tm4c->canopen_stack = CO;

    /* Canopen define the "reset communication" as a needed function*/
    err = CO_appResetCommunication();

    return err;
}

CO_ReturnError_t CO_appResetCommunication(void) {
    /*
     * The CO_CANinit() function must be called into "reset communication" section
     * This function Set CAN module in configuration mode and initialize the RX-TX arrays
     * to store the standard services ID's and pointers to respective functions.
     *
     * Also include the specific device hardware initialization of CAN controller.
     * The "CAN module" name is a general reference to tm4c CAN HW controller
     * */
    err = CO_CANinit(CO, co_node_tm4c, 0); // Inside this function controller init function should be called
    if (err != CO_ERROR_NO) {
        return err;
    }

    /* Another CiA standard service, this values needs to be defined at Object Dictionary*/
    CO_LSS_address_t lssAddress = {
        .identity = {.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                     .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                     .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                     .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber}
    };

    err = CO_LSSinit(CO, &lssAddress, &co_node_tm4c->desired_node_id, &co_node_tm4c->co_bit_rate);
    if (err != CO_ERROR_NO) {
        return err;
    }

    co_node_tm4c->active_node_id = co_node_tm4c->desired_node_id;
    uint32_t errInfo = 0;

    /*
     * The function CO_CANopenInit calls the respective "init" function to initialize all the
     * services listed in the input parameters
     */
    err = CO_CANopenInit(CO,                   /* CANopen object */
                         NULL,                 /* alternate NMT */
                         NULL,                 /* alternate em */
                         OD,                   /* Object dictionary */
                         OD_STATUS_BITS,       /* Optional OD_statusBits */
                         NMT_CONTROL,          /* CO_NMT_control_t */
                         FIRST_HB_TIME,        /* firstHBTime_ms */
                         SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                         SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                         SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                         co_node_tm4c->active_node_id, &errInfo);

    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        return err;
    }

    /* PDO's initialization require another separated init function */
    err = CO_CANopenInitPDO(CO, CO->em, OD, co_node_tm4c->active_node_id, &errInfo);
    if (err != CO_ERROR_NO) {
        return err;
    }

    CO_CANsetNormalMode(CO->CANmodule);
    fflush(stdout);

    return CO_ERROR_NO;
}

uint8_t ui8GreenValue = 0;
uint8_t ui8RedValue = 0;

static void co_ledsUpdate() {
    ui8GreenValue = co_node_tm4c->co_green_led_value == 0 ? 0 : 255;
    ui8RedValue = co_node_tm4c->co_red_led_value == 0 ? 0 : 255;
    GPIOPinWrite(CAN_LEDS_PORT, CAN_RED_LED_PIN, ui8RedValue);
    GPIOPinWrite(CAN_LEDS_PORT, CAN_GREEN_LED_PIN, ui8GreenValue);
}

uint16_t buss_off_timer = 0;

static void co_handleBusError(void) {
    CO_CANmodule_t* can_module = co_node_tm4c->canopen_stack->CANmodule;

    if (can_module->CANerrorStatus & (CO_CAN_ERRTX_WARNING || CO_CAN_ERRTX_BUS_OFF)) {
        buss_off_timer += 1;
        if (buss_off_timer >= 500) {
            CO_CANmodule_disable(co_node_tm4c->canopen_stack->CANmodule);
            CO_CANsetNormalMode(co_node_tm4c->canopen_stack->CANmodule);
            buss_off_timer = 0;
        }
    }
}

void CO_appProcess(void) {
    /* CANopen process, generic functionalities declared */
    if ((time_current - time_old) > 0) { // Make sure more than 1ms elapsed
        CO_NMT_reset_cmd_t reset_status;
        uint32_t timeDifference_us = (time_current - time_old) * 1000;
        time_old = time_current;
        /* Generic process for CANopen device */
        reset_status = CO_process(CO, false, timeDifference_us, NULL);

        /* LEDS implementation for TM4C MCU */
        co_node_tm4c->co_red_led_value = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
        co_node_tm4c->co_green_led_value = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);
        co_ledsUpdate();

        /* In case of "Reset communication" command, this will reset CO stack and reset CAN controller */
        if (reset_status == CO_RESET_COMM) {
            /* delete objects from memory */
            CO_CANsetConfigurationMode((void*)co_node_tm4c);
            CO_delete(CO);
            CO_appResetCommunication(); // Reset Communication routine
        }
        /* In case of "Reset App" command, device mus be restarted */
        else if (reset_status == CO_RESET_APP) {
            /* No way back, just die*/
            SysCtlReset();
        }
    }
}

/* Thread function executes in constant intervals, this function can be called from FreeRTOS tasks or Timers ********/
void co_appInterrupt(uint32_t sys_timer) {
    time_current = sys_timer;
    CO_LOCK_OD(CO->CANmodule);
    if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
        bool_t syncWas = false;
        /* get time difference since last function call */
        uint32_t timeDifference_us = 1000; // 1ms second

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
        syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
        CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
        CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif
        /* Further I/O or nonblocking application code may go here. */
    }
    CO_UNLOCK_OD(CO->CANmodule);
    co_handleBusError();
}
