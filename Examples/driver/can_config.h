/*
 * Project      CANopen TM4C Library
 * @file        can_config.h
 *
 * @brief
 * Header File used to set CAN HW Configuration
 *
 * @details
 * Include the module details here
 *
 * @date        9 Oct 2023
 * @version     3.0.0
 * @author      Carlos Ruiz
 * @copyright   CIDESI� 2023
 *
 */

#ifndef CANOPEN_TM4C_COMMONS_H_
#define CANOPEN_TM4C_COMMONS_H_

/* CAN Controller Definitions*/
#define CAN_BIT_RATE 1000000U
#define CAN_LOOPBACK_MODE_ENABLED false


/*TODO: Replace this value with your specific NODE ID*/
#ifndef CO_DESIRED_NODE_ID
#define CO_DESIRED_NODE_ID 0x05
#endif
/* TODO: Define your HW Target*/
#define TM4C123

#ifdef TM4C123
/* Definitions for CANopen LEDS
 * TODO: Replace PORT and PIN number with your application use*/
#define CAN_LEDS_PERIPH SYSCTL_PERIPH_GPIOF
#define CAN_LEDS_PORT GPIO_PORTF_BASE
#define CAN_RED_LED_PIN GPIO_PIN_1
#define CAN_GREEN_LED_PIN GPIO_PIN_3

#define CAN_SYSCTL_PERIPH SYSCTL_PERIPH_CAN0
#define CAN_SYSCTL_PERIPH_GPIO SYSCTL_PERIPH_GPIOB
#define CAN_PORT GPIO_PORTB_BASE
#define CAN_GPIO_PINS GPIO_PIN_4 | GPIO_PIN_5
#define CAN_RX_PIN GPIO_PB4_CAN0RX
#define CAN_TX_PIN GPIO_PB5_CAN0TX
#define CAN_BASE CAN0_BASE
#define CAN_INT INT_CAN0

#elif defined TM4C129
/* Definitions for CANopen LEDS
 * TODO: Replace PORT and PIN number with your application use*/
#define CAN_LEDS_PERIPH SYSCTL_PERIPH_GPION
#define CAN_LEDS_PORT GPIO_PORTN_BASE
#define CAN_RED_LED_PIN GPIO_PIN_0
#define CAN_GREEN_LED_PIN GPIO_PIN_1

#define CAN_SYSCTL_PERIPH SYSCTL_PERIPH_CAN0
#define CAN_SYSCTL_PERIPH_GPIO SYSCTL_PERIPH_GPIOA
#define CAN_PORT GPIO_PORTA_BASE
#define CAN_GPIO_PINS GPIO_PIN_0 | GPIO_PIN_1
#define CAN_RX_PIN GPIO_PA0_CAN0RX
#define CAN_TX_PIN GPIO_PA1_CAN0TX
#define CAN_BASE CAN0_BASE
#define CAN_INT INT_CAN0

#endif

#endif /* CANOPEN_TM4C_COMMONS_H_ */
