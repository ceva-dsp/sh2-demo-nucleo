# Example Application for Hillcrest SH2 sensor modules

This repository contains example code showing how to use the SH2
library to interface with Hillcrest Lab's SH2 sensor modules.  These
include the BNO080 and BNO085 processors as well as the FSP200 module.

The BNO080 configurations of this project run on the BNO080
development kit.  This consists of an ST Microelectronics Nucleo board
combinded with a BNO080 shield board made by Hillcrest.

Note: This repository is similar to another one, bno080-nucleo-demo.
That earlier repository used FreeRTOS and only supported the BNO080
processor.  This one does not rely on any RTOS, features an improved
library API and simpler HAL interface, and also supports the FSP200
module.

## Requirements

* IAR Embedded Workbench for ARM (EWARM) version 7.4
* STM32F411 Nucleo board
* Hillcrest BNO080 Shield board

## Setup

Clone this repository using the --recursive flag with git:
  * git clone --recursive https://github.com/hcrest/sh2-demo-nucleo.git

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

Hillcrest SH2 Demo.
Part 10003608 : Version 3.2.12 Build 475
Part 10003606 : Version 1.2.4 Build 230
Part 10003254 : Version 4.4.3 Build 485
Part 100e  0.3017 GRV: r:0.021179 i:-0.000244 j:0.999695 k:-0.011353
  0.3117 GRV: r:0.021179 i:-0.000244 j:0.999695 k:-0.011353
  0.3220 GRV: r:0.021118 i:-0.000244 j:0.999695 k:-0.011353
  0.3320 GRV: r:0.021118 i:-0.000244 j:0.999695 k:-0.011292
  0.3420 GRV: r:0.021118 i:-0.000244 j:0.999695 k:-0.011292
  0.3521 GRV: r:0.021118 i:-0.000244 j:0.999695 k:-0.011292
.
.
.
```
