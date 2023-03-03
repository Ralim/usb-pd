[![HitCounter](https://hits.dwyl.com/ralim/usb-pd.svg?style=flat-square)](http://hits.dwyl.com/ralim/usb-pd)
# usb-pd

USB-PD driver stack for the FUSB302.

This is a modified fork of the PD Buddy firmware stack.
All inspiration and groking of the PD spec belongs to [Clara Hobb's](https://hackaday.io/project/20424-pd-buddy-sink).

This was originally part of the IronOS project but has been extracted to allow for re-use and testing in isolation.
The code has been moved to C++ static classes and had minor changes to improve its features for its intended application.
As part of this, some logic that was originally in the PD Buddy firmware has been removed.

## Features supported

- PD 2.0 / PD 3.0 / PD 3.1 (EPR)
- - PPS requesting dynamic voltage
- Re-request a voltage change on the fly

## Design requirements

- C++
- FUSB302 interface IC is required
- I2C connection to the FUSB302
- Should work with any reasonable RTOS
- Can work without an RTOS (you may need to put some work into)
- Must be able to respond to the interrupt or poll in under 10ms
- Can be used in polling for events (no irq) but its not reccomended
- Little Endian processor (PD communication is LE, and this is assumed in the project)

## Implementing this library

To add this library to your project, either download the source code as a zip file up there ^ or add this library as a git submodule.

An example implementation of this library is available [for the black pill STM32F401](https://github.com/Ralim/STM32-USB-PD-Demo/).

Steps for implementation:

- Create an fusb302 object & link to a policy engine object
- You will need to create i2c function links and callbacks as per the objects
- See below notes on when to call functions
- Implement the logic for selecting from proposed power capabilities. (See demo implementation in the example implementation)

### Key function calls

The minimum required function calls are the fusb302 objects irq handler and the thread call in the policy engine.
The IRQ call will query over the I2C bus the status of the fusb object, and if a message is pending, read it in.
The thread call on the policy engine will perform at most one step of the state machine. It will return true if there are more iterations to perform.
This allows for the implementer to decide how to handle iterations, and makes each call a calculatable maximum execution time.
If this is a not a concern, a tight while loop (`while (pe.thread){}`) can be used.

Once an interrupt is recieved from the fusb302, it is reccomended to iterate the thread until it stops (to avoid backlog in processing).

### Implementing the selection logic

The key function to implement is the `pdbs_dpm_evaluate_capability`.
This is provided the chargers advertised power options, and should assemble a response to be sent back.
You can implement any logic that you desire to select the option.
