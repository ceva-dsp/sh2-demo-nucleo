# Example Application for CEVA SH2 sensor modules

This repository contains example code showing how to use the SH2
library to interface with CEVA's SH2 sensor modules.  These
include the BNO080 and BNO085 processors as well as the FSP200 module.

The BNO080 configurations of this project run on the BNO080
development kit.  This consists of an ST Microelectronics Nucleo board
combinded with a BNO080 shield board made by CEVA.

Note: This repository is similar to another one, bno080-nucleo-demo.
That earlier repository used FreeRTOS and only supported the BNO080
processor.  This one does not rely on any RTOS, features an improved
library API and simpler HAL interface, and also supports the FSP200
module.

## Requirements

* IAR Embedded Workbench for ARM (EWARM) version 7.4
* STM32F411 Nucleo board
* CEVA BNO080 Shield board

## Setup

Clone this repository using the --recursive flag with git:
  * git clone --recursive https://github.com/ceva-dsp/sh2-demo-nucleo.git

## Building the Code
* Use IAR EWARM to open the workspace, sh2-demo-nucleo/EWARM/Project.eww
* Select a project configuration, demo-i2c, for example.
* Run Project -> Rebuild All to compile the project.

## Running the Application

* Mount the shield board on the Nucleo platform.
* Ensure all switches are set properly (switch positions vary
  based on whether SPI or I2C interface is used.)
* Connect the Nucleo board to the development PC via USB.
* In IAR EWARM, execute Project -> Download and Debug.
* Once the debugger is ready, click the Go button.

The application should print the SH-2 version numbers, then start
reading and printing Game Rotation Vectors from the sensor hub:

```

CEVA SH2 Demo.
Part 10003608 : Version 3.6.0 Build 671
Part 10003606 : Version 1.5.0 Build 305
Part 10004135 : Version 4.11.0 Build 68
Part 10004149 : Version 4.6.7 Build 30
  0.2611 GRV: r:0.998901 i:-0.002075 j:-0.046509 k:0.000122
  0.2714 GRV: r:0.998901 i:-0.002075 j:-0.046509 k:0.000061
  0.2814 GRV: r:0.998901 i:-0.002075 j:-0.046509 k:0.000061
  0.2921 GRV: r:0.998901 i:-0.002075 j:-0.046509 k:0.000061
  0.3015 GRV: r:0.998901 i:-0.002075 j:-0.046509 k:0.000061
  0.3123 GRV: r:0.998901 i:-0.002075 j:-0.046509 k:0.000122
.
.
.
```

## Release Notes
### v1.4.0
* Sensor configuration is now based on a table.
* I2C HAL reads 4 bytes, not 2, on initial read.  (Fixes SHTP sequence number errors.)
* UART HAL now has #define ALWAYS_WAKE to bypass wake functionality.
* Fixed error in stability detector configuration.
* Added I2C bus retry logic to handle NAK events.
* DFU for FSP200 no longer checks product ids.
* New product id, 1000-4563, added to BNO08x DFU checks.
* Added #define switch to support console operation at 2MBPS.
* Updated IAR Embedded Workbench project to version 9.30.
