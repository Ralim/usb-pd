# Structure

This code is designed to support:

- Running on some form of RToS (at the moment only FreeRToS is supported but generalisation is planned)
- Only implements a SINK (aka this unit consumes power, does not provide)
- Only negotiates power and ignores the existance of all else
- Be somewhat as portable across Firmware as is easily possible

To this end this is currently build using a little indirection that can make instantiation a little messy but should allow for more hardware flexibility.

When creating an instance of the "PD Policy Engine" this needs to have access to a few key components:

- Ability to read and write to the I2C bus to an FUSB302 device
- Notification of IRQ from the FUSB302 (or you can call this fairly fast to poll)
- A user function is required to be provided to evaluate the options proposed by the charger

In order to faciliate these, these are passed in as a setup arguments as appropriate

External Support required functions:

- Tick counter with at least 10 ms precision
- Events for blocking tasks

Implementation notes:

- If the irq call returns a `true` the PD task must run as soon as possible afterwards
