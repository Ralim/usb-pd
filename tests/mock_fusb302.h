#pragma once
#include <iostream>
#include <queue>
#include <stdint.h>

/*
 * Implements a mockup of an FUSB302 that is used by testing
 * This works by having fake I2C handlers that talk to an internal state of registers
 * The testing code can then modify these to suit the situation
 */
class MockFUSB302 {
public:
  // Linkable to the fusb302 object
  bool i2cRead(const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf);
  bool i2cWrite(const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf);

  // methods used via testing to put in values
  void    setRegister(const uint8_t reg, const uint8_t value);
  uint8_t getRegister(const uint8_t reg);
  void    addToFIFO(const uint8_t length, const uint8_t *data);
  void    addToFIFO(const uint8_t data);
  // Reset to defaults
  void reset();
  void resetFiFo();
  bool readFiFo(const uint8_t length, uint8_t *buffer);
  bool fifoEmpty() { return fifoContent.size() == 0; };

private:
  bool validateRegister(const uint8_t reg);
  void updateFiFoStatus();
  // Cached state of the internal regs
  uint8_t             mockRegs[0x43];
  std::queue<uint8_t> fifoContent;
};