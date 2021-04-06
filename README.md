# usb-pd

USB-PD driver stack for the FUSB302.

This is a modified fork of the PD Buddy firmware stack.
All inspiration and groking of the PD spec belongs to [Clara Hobb's](https://hackaday.io/project/20424-pd-buddy-sink).

This was originally part of the IronOS project but has been extracted to allow for re-use and testing in isolation.
The code has been moved to C++ static classes and had minor changes to improve its features for its intended application.
As part of this, some logic that was originally in the PD Buddy firmware has been removed.

## Features supported

- PD 2.0 / PD 3.0
- - PPS requesting dynamic voltage

## Design requirements

- C++
- FUSB302 interface IC is required
- I2C connection to the FUSB302
- Should work with any reasonable RTOS
- - Must provide threads, blocking events and some form of mailbox

## Implementing this library

To add this library to your project, either download the source code as a zip file up there ^ or add this library as a git submodule.

Once the code is added to your project, implementations for all of the functions in the `user.h` file will be required in your project at compile time.
These are all bridges to your project's selected I2C / RTOS.
