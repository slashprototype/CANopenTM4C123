# CANopenNode TM4C

**CANopenNodeTM4C** is a CANopen stack running on TM4C12xx microcontrollers from Texas Instruments, based on the open-source [CANopenNode](https://github.com/CANopenNode/CANopenNode) stack. This port provides a bare-metal approach without using any RTOS. It leverages the TivaWare™ Peripheral Driver Library and the CAN controller hardware present in TM4C MCUs.

---

## How to Run Demos

1. **Toolchain**: Demos are developed in [Code Composer Studio (CCS)](https://www.ti.com/tool/CCSTUDIO), the official development environment for TI microcontrollers, although you can adapt the project to other IDEs if desired.

2. **Hardware Requirements**:
  - A TM4C123-based launchpad or custom board with a functioning CAN transceiver circuit.
  - A CAN bus setup (e.g., transceiver + bus termination), or a CAN analysis tool if you want to inspect the CAN frames.

3. **Steps**:
  - Create a CCS project considering the custom board requirements.
  - Clone or download this repository (including submodules) to your workspace.
  - Install TivaWare for C Series and identify installation directory e.g. "C:/ti/TivaWare_C_Series-2.2.0.xxx"
  - Realize the next CCS workspace properties configurations:
   - Include the required paths:
    - "C:/ti/TivaWare_C_Series-2.2.0.xxx"
    - Examples/driver/
    - Examples/src/
    - tm4c_utils/
    - CANopenNode/
    - CANopenNode_TM4C123/
   - Into Build/Arm Linker/Basic Options:
    - Heap Size for C/C++ dynamic memory allocation = 8000
    - Set C system stack size = 8000
   - Configure your Dialect in order to compile program in C99 (--C99). For CCS is inside Build/Arm Compiler/Advanced Options/Language Options/C Dialect
  - Connect your TM4C board to your PC via USB for programming and debugging.
  - Compile and flash the project onto the board.
  - Once running, the CANopen stack will initialize, and you can observe or interact with the node on the CAN bus. Also Tiva Launchpad RGB Leds will be blinking according to CANopen Standard

---

## Repository Directories

- **./CANopenNode**  
  Contains the original CANopenNode stack implementation. Typically, you do not need to modify these files. They include the communication protocols, object dictionary handling, PDO/SDO services, and other core functionalities common across different platforms.

- **./CANopenNodeTM4C**  
  Platform-specific implementation layer for TI TM4C microcontrollers. It manages:
  - Hardware initialization (CAN controller, GPIO, interrupts).
  - Integration with TivaWare drivers.
  - Interrupt handlers for receiving/transmitting CAN frames.
  - Timer or SysTick usage for CANopen process scheduling.

- **./examples**  
  Includes example projects demonstrating how to set up and use CANopenNodeTM4C on various TM4C123 board or custom hardware. Each example has a **main.c** showing basic initialization, process loops, and minimal application code.

---

## Supported Boards

While this project can be easily adapted to many TM4C123 devices, below is a non-exhaustive list of boards we have tested:

- **EK-TM4C123GXL (TM4C123G LaunchPad)**  
  A commonly used low-cost development kit featuring the TM4C123 MCU. It exposes the CAN pins on booster pack headers—an external CAN transceiver is required.

If your board provides access to the CAN peripherals and pins, you can modify the startup and pin configuration files to match your hardware design.

---

## Example Usage

Below is a simplified structure of how a main application might look for a bare-metal project using **CANopenNodeTM4C**. The key steps are:

1. Set up system clock.
2. Initialize the CANopenNode stack (including the hardware).
3. Enable global interrupts.
4. Enter a loop calling the CANopen process function.

```c
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "CO_app_TM4C.h" // Include your CANopen TM4C interface

int main(void)
{
   // Set system clock (example: 40 MHz)
   SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ |
             SYSCTL_USE_OSC | SYSCTL_SYSDIV_2_5);

   // Retrieve system clock value
   uint32_t sysClock = SysCtlClockGet();

   // Create a node structure
   co_node_tm4c_t coNodeTM4C;

   // Initialize the CANopen stack and hardware
   if(canopenInit(&coNodeTM4C, sysClock) != CO_ERROR_NO) {
      // Error handling if needed
      while(1);
   }

   // Enable processor interrupts
   IntMasterEnable();

   // Main loop
   while (1)
   {
      // Periodically call CANopen process
      CO_appProcess();
   }
}
```

In this example:

1. Clock Setup: The system clock is configured using TivaWare calls.
2. CANopenNodeTM4C Init: canopenInit() sets up the CAN controller, configures interrupts, and initializes the internal CANopen data structures.
3. Loop: Continually call CO_appProcess() to handle background tasks like SDO/PDO management.

---

## Porting to Other TM4C Microcontrollers Checklist

- Create a new CCS project for your target TM4C MCU (e.g., TM4C123, TM4C129).
- Configure system clock using TivaWare (e.g., SysCtlClockSet()) or TI SysConfig.
- Enable CAN peripheral with SysCtlPeripheralEnable() and configure CAN pins with GPIOPinConfigure().
- Connect a CAN transceiver to the TX/RX pins. Ensure correct bus termination.
- Add or copy CANopenNode and CANopenNodeTM4C into your project.
- Include the relevant headers in your build settings and in main.c.
- Implement the timer logic for a 1ms interrupt if you want a dedicated timer for CANopen’s internal tick (although SysTick can also be used). If using an interrupt for the 1ms tick:

```c
// Example if using SysTick
void SysTickHandler(void)
{
   CO_appInterrupt(); // update CANopen timers
}
```

- In your main loop, call CO_appProcess() frequently to handle all non-interrupt CANopen tasks.

---

## Known Limitations

- Single CAN Module: By default, these examples support only a single CAN module. If your device has multiple, you must replicate the code for additional modules.
- No RTOS: This is a bare-metal implementation. You may adapt it to FreeRTOS or another OS with caution, ensuring thread-safety when accessing CANopen data structures.
- Limited Test Scope: Multi-node or advanced features (e.g., multi-OD instances) are not fully validated in this reference code.

---

## Clone or Update

To clone this repository with submodules:

```bash
git clone https://github.com/your-username/CANopenNodeTM4C.git
cd CANopenNodeTM4C
git submodule update --init --recursive
```

To update an existing local copy:

```bash
cd CANopenNodeTM4C
git pull
git submodule update --init --recursive
```

---

## License

This file is part of CANopenNode, an open-source CANopen Stack.
Project home page: https://github.com/CANopenNode/CANopenNode
For more information on CANopen, see CAN in Automation (CiA).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at:
http://www.apache.org/licenses/LICENSE-2.0