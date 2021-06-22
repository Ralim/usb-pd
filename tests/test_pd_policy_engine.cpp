#include "CppUTest/TestHarness.h"
#include "fusb302_defines.h"
#include "fusb302b.h"
#include <stdint.h>
// Working through testing the PD state machine
TEST_GROUP(PD){};
TEST(PD, ReadDeviceIDHappyPath) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    CHECK_EQUAL(0x23 << 1, deviceAddress);
    CHECK_EQUAL(FUSB_DEVICE_ID, address);
    CHECK_EQUAL(size, 1);
    buf[0] = 0x10;
    return true;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("Should not issue writes in read only functions");
    return false;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);
  CHECK_EQUAL(true, f.fusb_read_id());
}
