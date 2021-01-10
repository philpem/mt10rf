# MT10RF transmitter

## Introduction

This is an Arduino sketch which uses a HopeRF RFM68 (868MHz version) to control a Worcester-Bosch boiler via the MT10RF receiver.

I wrote this because I had issues with battery drain and temperature setting inaccuracy with my MT10RF. Eventually this will become part of a home thermostat project.

This code was clean-room reverse engineered using a pair of MT10RF transmitters and a single receiver. While the code is Arduino-based, it should be portable to most platforms with minor changes to the hardware interface code.

## Hardware setup

This was tested on an Elegoo Uno R3, and should also work on the genuine Arduino Uno.

  * RFM68 power -> Arduino +3.3V
  * RFM68 NRESET -> Arduino pin 4
  * RFM68 DATA -> Arduino pin 5
  * RFM68 CTRL -> Arduino pin 6

## Contact details and licence

Phil Pemberton <philpem@philpem.me.uk>

This code is licenced under a 3-clause BSD licence. Please see the file "LICENCE" for more information.

Please email me if you do something cool with this code!
