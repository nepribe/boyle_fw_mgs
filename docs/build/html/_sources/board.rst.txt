.. _board:

Board Documentation
===================

The board used in this project is designed to support:

* Microcontroller
* Power Supply Circuit
* 96 MGSs
* 3 SHT25 sensors
* 3 DPS310 sensors
 
The design of the board has two parts: The first part, so-called the motherboard, contains the microcontroller, power supply and the components to communicate with the DUTs. The second part, called the daugtherboard, contains the MGSs and the reference sensors.


x96_board Roadmap
-----------------

The design of the board suffered changes until the last version. :numref:`Fig. %s <board_roadmap>` shows the roadmap of these changes.

.. _board_roadmap:
.. figure:: /images/FW_x96-x96_board_roadmap.svg
	:scale: 150 %
	:align: center

	Roadmap board design [**updated: [13/07/21**]

The highlights of each change in the design of the board is provided by :numref:`Fig. %s <board_roadmap_details>`

.. _board_roadmap_details:
.. figure:: /images/FW_x96-x96_board_roadmap_details.svg
	:scale: 150 %
	:align: center

	Board design sum-up

x96_board Motherboard
---------------------

The motherboard contains the microcontroller, power supply and the components to comunicate with the DUTs. :numref:`Fig. %s <motherboard_blockdiagram>` illustrates the block diagram of the motherboard, which has three main blocks. The micro-controller, the power supply block and the signal multiplexer module.

.. _motherboard_blockdiagram:
.. figure:: /images/FW_x96-x96_motherboard_blockdiagram.svg
	:scale: 100 %
	:align: center

	Block diagram of the motherboard

MicroController
***************
During the *Build 0.0* the microcontroller selected to be responsable to communicate with the DUTs, the power supply and be able to support an interface communication with a computer was an **Arduino Mega**. However, from *Version 0.0* and later it was changed to **PSoC 5**.


Supply Power
************
The block of power supply uses the 5V from **PSoC 5** to generate VDD required to supply other components.

Signal Multiplexer
******************
The `I2C protocol <../../source/_documents/i2c_protocol.pdf>`_ is the interface communication between the microController to the MGSs and the reference sensors. However, due to all the 96 MGSs have the same address the implementation of the standard two wires (SDA, SCL line) bus is not possible. Additionally. the MGSs have a output called INT, which must be read from each MGS. In order to achieve 'parallelism' to communicate with all the DUTs (could be just one), the implementation of multiplexer block was necessary. This block uses a daisy-chain configuration for multiplexer I2C SDA signals and the INT signal, the SCL signal comes from one single PSoC pin and it is replicate 96 times for each MGS. This block can multiplex up to 96 I2C signal lines throught 6 signal lines that comes from PSoC pins (SDA_MP0,1,2,...,5) and one single SCL signal. Each signal line can access to 16 SDA signal lines (total of 96 I2C SDA signal lines) and 16 INT signals. :numref:`Fig. %s <multiplexer_blockdiagram>` shows the overview block diagram of the signal multiplexer module. 


.. _multiplexer_blockdiagram:
.. figure:: /images/FW_x96-x96_multiplexer_blockdiagram.svg
	:scale: 100%
	:align: center

	Block diagram of the multiplexer

Daisy-Chain Block
+++++++++++++++++
:numref:`Fig. %s <daisy_chain_diagram>` shows the connections of the different components that the daisy chain block has.


.. _daisy_chain_diagram:
.. figure:: /images/FW_x96-x96_chaisy_chain_diagram.svg
	:scale: 100%
	:align: center

	Diagram of the Daisy-Chain block

Repeater Block
++++++++++++++


.. _repeater_diagram:
.. figure:: /images/FW_x96-x96_repeater_diagram.svg
	:scale: 100%
	:align: center

	Diagram of the Repeater block


x96_board Daugtherboard
-----------------------


