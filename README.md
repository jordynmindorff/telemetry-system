# Drone/Vehicle Telemetry System

Long story short, I wanted a little side project to work on involving some form of RF communication. I also wanted an excuse to buy a drone at some point.

And thus, this project.

## Overview

The telem_receive and telem_send folders contain arduino firmware code I wrote. The send side interacts with an SHT45 Temp/Humidity sensor and a DFRobot TEL0157 GPS board over I2C. I implemenetd the code for reading/writing the registers for the SHT45 myself, whereas I used a library for the GPS board (mainly due to a lack of good documentation).

On both the send and receive sides, nRF24L01 RF transceivers are used for communications over the air. See [comms-structure.md](comms-structure.md) for details.

The Rx side uses an Arduino UNO v3, whereas the send side uses an STM32 Nucleo F401RE board.
