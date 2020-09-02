# OBD2-CAN-Simulator
This project simulates CAN BUS of OBD2 port, helping to develop OBD2 CAN BUS projects

How It Works:
This project get the third byte of data of received CAN message and return then with randomic values on fourth-seventh data of CAN message.
To read OBD protocol our device need to have an ID = 0x7DF, ECM will return with ID = 0x7E8. Knowing this, we configure CAN filter to receive only ID = 0x7E8, with this filter any other ID will be ignored.

Software:
It uses cubeMX for generate a pre-code. Compiler Keil uVision, but you can easily migrate to another suported compiler by cubeMX, like IAR.
Using STM32F103, we can't uses CAN and USB at same time, because both peripherals share same SRAM. It can works if turning off CAN and turning ON USB, and vice-versa, but I didn't have success.

Hardware:
This hardware uses a STM32F103C8T6 (bluepill board) with a CAN transceiver (MCP2251).
MCP2551 is 5V logic, because of that we need to use PB9 and PB8 pins (this pins are 5V tolerant). Pins PA11 and PA12 aren't 5V tolerant.

Clock was configured at 72MHz, with this clock set CAN preescaler = 9, Time Quantum in Bit Segment 1 = 3TQ, Time Quantum in Bit Segment 2 = 4TQ. This configuration gives a CAN speed = 500kbps (2uS for each bit).


