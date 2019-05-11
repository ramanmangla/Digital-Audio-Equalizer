# Digital-Audio-Equalizer
Digital stereo audio filters created for Altera DE1 SoC (Cyclone 5 FPGA) board using Verilog HDL

## Project Objective
The project provides real time input audio equalization using Finite Impulse Response (FIR) filters.
The FIR filters correspond to 3 frequency bands:
* Low Pass (0-500Hz)
* Band Pass (550-3000Hz)
* High Pass (Higher than 3000Hz)

The filters and the digital logic have been written in Verilog hardware description language.
The Line-In and Line-Out audio ports are used for input and processed output respectively.
Different Switch combinations are used to select different FIR Filters.
The VGA Display is used to display the control instructions.
The HEX Display shows the band frequencies for the selected filters.

## I/O
* Switches (Input)
* Line-In port (Input)
* HEX Display (Output)
* Line-Out port (Output)
* VGA Display (Instructions)
