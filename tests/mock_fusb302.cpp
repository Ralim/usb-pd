#include "mock_fusb302.h"
#include "CppUTest/TestHarness.h"
#include "fusb302_defines.h"
#include <cstring>
void MockFUSB302::reset() {
  // 0 init by default
  memset(mockRegs, 0, sizeof(mockRegs));
  // Set regs to default state from datasheet
  setRegister(FUSB_DEVICE_ID, 0x90);
  setRegister(FUSB_SWITCHES0, 0x03);
  setRegister(FUSB_SWITCHES1, 0x20);
  setRegister(FUSB_MEASURE, 0x31);
  setRegister(FUSB_SLICE, 0x60);
  setRegister(FUSB_CONTROL0, 0x24);
  setRegister(FUSB_CONTROL1, 0x00);
  setRegister(FUSB_CONTROL2, 0x02);
  setRegister(FUSB_CONTROL3, 0x06);
  setRegister(FUSB_MASK1, 0x00);
  setRegister(FUSB_POWER, 0x01);
  setRegister(FUSB_RESET, 0x00);
  setRegister(FUSB_OCPREG, 0x0F);
  setRegister(FUSB_MASKA, 0x00);
  setRegister(FUSB_MASKB, 0x00);
  setRegister(FUSB_CONTROL4, 0x00);
  setRegister(FUSB_STATUS0A, 0x00);
  setRegister(FUSB_STATUS1A, 0x00);
  setRegister(FUSB_INTERRUPTA, 0x00);
  setRegister(FUSB_INTERRUPTB, 0x00);
  setRegister(FUSB_STATUS0, 0x00);
  setRegister(FUSB_STATUS1, 0x28);
  setRegister(FUSB_INTERRUPT, 0x00);
  // Clear FiFo
  resetFiFo();
}
void MockFUSB302::resetFiFo() {
  while (!fifoContent.empty()) {
    fifoContent.pop();
  }
}

bool MockFUSB302::i2cRead(const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) {
  // Validate valid i2c address
  bool addressValid = (deviceAddress == FUSB302B_ADDR) || (deviceAddress == FUSB302B01_ADDR) || (deviceAddress == FUSB302B10_ADDR) || (deviceAddress == FUSB302B11_ADDR);
  CHECK_TRUE(addressValid);
  if (address == FUSB_FIFOS) {
    readFiFo(size, buf);
  } else {
    for (int i = 0; i < size; i++) {
      buf[i] = getRegister(address + i);
    }
  }
  return true;
}
bool MockFUSB302::i2cWrite(const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) {
  // Validate valid i2c address
  bool addressValid = (deviceAddress == FUSB302B_ADDR) || (deviceAddress == FUSB302B01_ADDR) || (deviceAddress == FUSB302B10_ADDR) || (deviceAddress == FUSB302B11_ADDR);
  CHECK_TRUE(addressValid);
  if (address == FUSB_FIFOS) {
    FAIL("TODO");
  } else {
    for (int i = 0; i < size; i++) {
      setRegister(address + i, buf[i]);
    }
  }
  return true;
}

bool MockFUSB302::validateRegister(const uint8_t reg) {
  switch (reg) {
  case FUSB_DEVICE_ID:
  case FUSB_SWITCHES0:
  case FUSB_SWITCHES1:
  case FUSB_MEASURE:
  case FUSB_SLICE:
  case FUSB_CONTROL0:
  case FUSB_CONTROL1:
  case FUSB_CONTROL2:
  case FUSB_CONTROL3:
  case FUSB_MASK1:
  case FUSB_POWER:
  case FUSB_RESET:
  case FUSB_OCPREG:
  case FUSB_MASKA:
  case FUSB_MASKB:
  case FUSB_CONTROL4:
  case FUSB_STATUS0A:
  case FUSB_STATUS1A:
  case FUSB_INTERRUPTA:
  case FUSB_INTERRUPTB:
  case FUSB_STATUS0:
  case FUSB_STATUS1:
  case FUSB_INTERRUPT:
  case FUSB_FIFOS:
    return true;
  default:
    return false;
  }
}

void MockFUSB302::setRegister(const uint8_t reg, const uint8_t value) {
  CHECK_TRUE(validateRegister(reg));
  mockRegs[reg] = value;
}
uint8_t MockFUSB302::getRegister(const uint8_t reg) {
  CHECK_TRUE(validateRegister(reg));
  return mockRegs[reg];
}
void MockFUSB302::addToFIFO(const uint8_t length, const uint8_t *data) {
  for (int i = 0; i < length; i++) {
    fifoContent.push(data[i]);
  }
}

bool MockFUSB302::readFiFo(const uint8_t length, uint8_t *buffer) {
  CHECK_TRUE(fifoContent.size() >= length);
  for (int i = 0; i < length; i++) {
    buffer[i] = fifoContent.front();
    fifoContent.pop();
  }
}