.. _firmware:

Firmware Documentation
======================

The firmware is developed in the Integrated Design Environment (IDE) of PSoC device, PSoC Creator 4.4.

Coding Guidelines
-----------------

This document explains how to participate in project conversations, log bugs and enhancement requests, and submit changes to the project so your patch will be accepted quickly in the codebase. All contributors need to follow certain set of rules while working on this project.
As a contributor, you’ll want to be familiar with the x96_Board project, how to configure, install, and use it. 



Principles for development
**************************

Unit Testing Guide
******************

Workflow
********

Bug report
**********

Communication Channel
*********************

References
**********

Flow Diagram of FW
------------------

The complete firmware is divided in 9 different .c files. :numref:`Fig. %s <fw_main_diagram>` shows the color block of the 9 different files and the flow diagram of the main program, which is located in main.c file.

.. _fw_main_diagram:
.. figure:: /images/FW_x96-fw_main_diagram.png
	:align: center

	FW main diagram

The main program starts with the initialization function and then enter to an infinite for-loop in "standby mode". During this mode the firmware polls the USB communication to detect any message. :numref:`Fig. %s <fw_init_diagram>` illustrates the 8 steps to initialize the board and the DUTs.

.. _fw_init_diagram:
.. figure:: /images/FW_x96-fw_x96_init.png
	:alt: Left floating image
	:align: left

	Initialization diagram

| 

|  1. Start MGSs supply with 3.3v by default

| 2. Init SPIM for the multiplexer module

| 3. Init I2x6 communication

| 4. Start USB UART communication

| 5. Detect the MGS devices physically connected

| 6. Detect the SHT25 devices physically connected

| 7. Detect the DPS310 devices physically connected

| 8. Setup the interruptions

|

|

All the mensages detected by the 'mgs_USB' module, the mensages that are sent from PSoC and the interpretation of these messages are handled with 'CommHandler' module, including the function 'CommRX' called in main function (:numref:`Fig. %s <fw_main_diagram>`). (:numref:`Fig. %s <_fw_CommRX_diagram>`) shows 'CommRX' function. 

.. _fw_CommRX_diagram:
.. figure:: /images/FW_x96-CommRX_diagram.png
	:align: center

	CommRX Function