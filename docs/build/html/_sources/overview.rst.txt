.. _overview:

Overview Project
================

The objective of the x96_board project is to have a system that allows to obtain the performance of multiple multigas sensors-MGS, so called device under test (UDT) in each single test. The project has 3 phases in order to achieve this objective:

* :ref:`Design of the board <board>`
* :ref:`Firmware development <firmware>`
* :ref:`API development <driver>`

Each phase has a testing sub-phase before moving to the next phase.

The project is elaborated taking into account the following constraints:

* Up to 96 MGSs under test.
* Up to 3 reference sensors to measure temperature.
* Up to 3 reference sensors to measure pressure.


Overall architecture
--------------------

The abstracted overview
***********************

.. figure:: /images/FW_x96-abstracted_view.png
	:align: center


