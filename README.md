
# Sig Gen V1.1


1. [Introduction](#1-Introduction)
2. [Features](#2-features)
3. [Getting Started](#3-getting-started)
4. [Licenses](#4-licenses)

## 1. Introduction

I recently purchased a modern, reasonably capable oscilloscope. While waiting for the new scope to arrive I decided that I needed a
way to quickly and simply way test its basic functionality.  I looked at several scope manufacturers' demo/test boards and while they 
were appealing, they were also reasonably pricey (hundreds of dollars).  After careful consideration, I decided to use that money for
other things and create my own **basic** test board.  My goals were to create an **inexpensive** (couple of $), expandable, and customizable
system that would be usable by a wide audience. I called the project Sig Gen and decided to open source it.  Just be aware 
that even though I named it Sig Gen, it is **NOT** a general purpose higly functional signal generator. That was not its intended purpose.

 
>If you have questions, comments, ideas or if something is
not working as expected feel free to open a new issue or start a discussion.  If you wish, you may also reach out to me directly at [](mailto:)pwproj24@gmail.com


## 2. Features

Sig Gen will generate the following signals:

- 10/*20* MHz 50% Duty Cycle Waveform
- 100/*200* KHz 50%, 25%, 10%, 5%, 1% Duty Cycle Waveforms
- 1/*2* KHz 50% Duty Cycle Waveform
- 60/*120* MHz 50% Duty Cycle Waveform
- UART Waveform
- Manchester Encoded Data Waveform
- Low frequency PWM-DAC Analog Waveforms
  - 83.19/*166.38* Hz Ramp
  - 166.66/*333.32* Hz Sawtooth
  - 75.75/*151.5* Hz Staircase
  - 390.62/*781.24* Hz Sine
  - 195.31/*390.62* Hz Full Wave Rectified Sine
  - 75.75/*151.5* Hz Staircase with a normally distributed center segment
  - 75.75/*151.5* Hz Staircase with a uniformly distributed center segment
  - 1.0-1000.0/*2.0-2000.0* Hz Direct Digital Synthesis Sine
- I2C Waveforms
- SPI Waveforms

Note: The *Italic* notation indicates the operating frequency when the 2X overclocking option has been enabled.  The end of the README includes a photo that illustrates the signals listed in the previous table.


## 3. Getting Started

This distribution includes a full manual and a single page quick start "cheat-sheet". They can be found in the docs folder.



##  4. Licenses

A significant portion of this project utilizes code from the Raspberry Pi Pico example library.  As such its license file [LICENSE_RP_PICO.TXT](LICENSE_RP_PICO.TXT) applies.  For the portion of the code that I authored, [LICENSE_PW.TXT](LICENSE_PW.TXT) applies. 

This is an open-source project that is free of charge. Use this project in any way you like
(as long as it complies to the permissive license files).

---------------------------------------
![Sig Gen V1.1](images/WaveFormsBig.jpg)


