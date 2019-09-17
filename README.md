Overview
========
This example runs keyword spotting inference on live audio captured using Synaptics Conexant board attached to i.MX8M Mini EVK board.
When performing keyword spotting on live audio data with multiple noise sources, outputs are typically averaged over a specified window
to generate smooth predictions. The averaging window length and the detection threshold (which may also be different for each keyword)
are two key parameters in determining the overall keyword spotting accuracy and user experience.
For more details regarding the Key Word Spotting refer to: https://github.com/ARM-software/ML-KWS-for-MCU

Dependencies
===================
CMSIS-NN -> https://github.com/ARM-software/CMSIS_5
MCU SDK version 2.6.0 (SDK_2.6.0_EVK-MIMX8MM-ARMGCC):
- OS: Linux, Toolchain: GCC ARM Embedded
- Components: Amazon-FreeRTOS, CMSIS DSP Library, multicore
- SDK Version: 2.6.0 (2019-06-14)
- SDK Tag: REL_2.6.0_REL10_RFP_RC3_4

Toolchain supported
===================
- IAR embedded Workbench  8.32.3
- GCC ARM Embedded  8.2.1

Hardware requirements
=====================
- Micro USB cable
- MIMX8MM6-EVK  board
- 12V power supply
- Personal Computer
- 60 Pins Header
- Synaptics Conexant Board

Board settings
==============
No special is needed.

Prepare the Demo
================
1.  Connect 12V power supply and J-Link Debug Probe to the board, switch SW101 to power on the board
2.  Connect a USB cable between the host PC and the J901 USB port on the target board.
3.  Open two serial terminals for A53 core and M4 core with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Using U-Boot command to run the demo.bin file. For details, please refer to Getting Started with MCUXpresso SDK for i.MX 8M Mini.pdf
5.  After running the demo.bin, using the "boot" command to boot the kernel on the A core terminal;
6.  After the kernel is boot, using "root" to login.
7.  After login, make sure imx_rpmsg_pingpong kernel module is inserted (lsmod) or insert it (modprobe imx_rpmsg_pingpong).

Running the demo
================
u-boot=>fatload mmc 0 0x80000000 eiq-kws.bin
u-boot=>dcache flush
u-boot=>bootaux 0x80000000

After the boot process succeeds, the ARM Cortex-M4 terminal displays the following information:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
RPMSG Ping-Pong FreeRTOS RTOS API Demo...
RPMSG Share Base Addr is 0xb8000000
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
During boot the Kernel,the ARM Cortex-M4 terminal displays the following information:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Link is up!
Nameservice announce sent.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
After the Linux RPMsg pingpong module was installed, the ARM Cortex-M4 terminal displays the following information:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Looping forever...
Waiting for ping...
Sending pong...
96% go