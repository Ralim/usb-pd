#pragma once

/*
 * Implements a mockup of an FUSB302 that is used by testing
 * This works by having fake I2C handlers that talk to an internal state of registers
 * The testing code can then modify these to suit the situation
 */
class MockFUSB302 {
public:
  // Linkable to the fusb302 object

private:
  // Cached state of the internal regs
};