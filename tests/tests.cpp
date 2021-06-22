#include "CppUTest/TestHarness.h"
#include "fusb302_defines.h"
#include "fusb302b.h"
#include <stdint.h>
TEST_GROUP(FUSB){};
TEST(FUSB, ReadDeviceIDHappyPath) {
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
TEST(FUSB, ReadDeviceIDSadPath) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    CHECK_EQUAL(0x23 << 1, deviceAddress);
    CHECK_EQUAL(FUSB_DEVICE_ID, address);
    CHECK_EQUAL(size, 1);
    buf[0] = 0xFF;
    return true;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("Should not issue writes in read only functions");
    return false;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);
  CHECK_EQUAL(false, f.fusb_read_id());
}
TEST(FUSB, ReadDeviceIDI2CFailsPath) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    CHECK_EQUAL(0x23 << 1, deviceAddress);
    CHECK_EQUAL(FUSB_DEVICE_ID, address);
    CHECK_EQUAL(size, 1);
    buf[0] = 0x01;
    return false;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("Should not issue writes in read only functions");
    return false;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);
  CHECK_EQUAL(false, f.fusb_read_id());
}

TEST(FUSB, ResetDevice) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("No Reads should be required");
    return false;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    static uint8_t writeNumber = 0;
    CHECK_EQUAL(size, 1);
    switch (writeNumber) {
    case 0:
      CHECK_EQUAL(FUSB_CONTROL0, address);
      CHECK_EQUAL(buf[0], 0x44);
      break;
    case 1:
      CHECK_EQUAL(FUSB_CONTROL1, address);
      CHECK_EQUAL(buf[0], FUSB_CONTROL1_RX_FLUSH);
      break;
    case 2:
      CHECK_EQUAL(FUSB_RESET, address);
      CHECK_EQUAL(buf[0], FUSB_RESET_PD_RESET);
      break;
    default:
      FAIL("Unhandled read");
    }
    writeNumber++;
    return true;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);
  f.fusb_reset();
}

TEST(FUSB, ReadTypeCCurrentLevels) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    static uint8_t counter = 0;
    CHECK_EQUAL(FUSB_STATUS0, address);
    CHECK_EQUAL(1, size);
    // It reads the 2LSB
    buf[0] = counter;
    counter++;
    CHECK_TRUE(counter <= 4);
    return true;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("No Writes");
    return false;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);
  CHECK_EQUAL(fusb_typec_current::fusb_tcc_none, f.fusb_get_typec_current());
  CHECK_EQUAL(fusb_typec_current::fusb_tcc_default, f.fusb_get_typec_current());
  CHECK_EQUAL(fusb_typec_current::fusb_tcc_1_5, f.fusb_get_typec_current());
  CHECK_EQUAL(fusb_typec_current::fusb_tcc_3_0, f.fusb_get_typec_current());
}

TEST(FUSB, ReadStatus) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    CHECK_EQUAL(FUSB_STATUS0A, address);
    CHECK_EQUAL(7, size);
    buf[0] = 1;
    buf[1] = 2;
    buf[2] = 3;
    buf[3] = 4;
    buf[4] = 5;
    buf[5] = 6;
    buf[6] = 7;

    return true;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("No Writes");
    return false;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);

  fusb_status statusOut;
  f.fusb_get_status(&statusOut);
  CHECK_EQUAL(1, statusOut.status0a);
  CHECK_EQUAL(2, statusOut.status1a);
  CHECK_EQUAL(3, statusOut.interrupta);
  CHECK_EQUAL(4, statusOut.interruptb);
  CHECK_EQUAL(5, statusOut.status0);
  CHECK_EQUAL(6, statusOut.status1);
  CHECK_EQUAL(7, statusOut.interrupt);
}

TEST(FUSB, DeviceSetup) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    CHECK_EQUAL(FUSB_STATUS0A, address);
    CHECK_EQUAL(7, size);
    buf[0] = 1;
    buf[1] = 2;
    buf[2] = 3;
    buf[3] = 4;
    buf[4] = 5;
    buf[5] = 6;
    buf[6] = 7;

    return true;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("No Writes");
    return false;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);

  fusb_status statusOut;
  f.fusb_get_status(&statusOut);
  CHECK_EQUAL(1, statusOut.status0a);
  CHECK_EQUAL(2, statusOut.status1a);
  CHECK_EQUAL(3, statusOut.interrupta);
  CHECK_EQUAL(4, statusOut.interruptb);
  CHECK_EQUAL(5, statusOut.status0);
  CHECK_EQUAL(6, statusOut.status1);
  CHECK_EQUAL(7, statusOut.interrupt);
}