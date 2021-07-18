#include "CppUTest/TestHarness.h"
#include "fusb302_defines.h"
#include "fusb302b.h"
#include <cstring>
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

TEST(FUSB, RxMessagesPending) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    static uint8_t pos             = 0;
    const uint8_t  virtualBuffer[] = {0x00, 0x00, 1 << 5};
    CHECK_EQUAL(FUSB_STATUS1, address);
    memcpy(buf, virtualBuffer + pos, size);
    pos += size;
    CHECK_TRUE(pos <= sizeof(virtualBuffer));
    return true;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("Should not issue writes in read only functions");
    return false;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);
  CHECK_EQUAL(true, f.fusb_rx_pending());
  CHECK_EQUAL(true, f.fusb_rx_pending());
  CHECK_EQUAL(false, f.fusb_rx_pending());
}
TEST(FUSB, ReadGoodEmptyMessage) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    static uint8_t pos             = 0;
    const uint8_t  virtualBuffer[] = {0xE0, 0, 0, 1, 2, 3, 4};
    CHECK_EQUAL(FUSB_FIFOS, address);
    memcpy(buf, virtualBuffer + pos, size);
    pos += size;
    CHECK_TRUE(pos <= sizeof(virtualBuffer));
    return true;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("Should not issue writes in read only functions");
    return false;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);
  pd_msg  msg;
  CHECK_EQUAL(0, f.fusb_read_message(&msg));
}

TEST(FUSB, ReadGoodMessage) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    static uint8_t pos             = 0;
    const uint8_t  virtualBuffer[] = {0xE0, 0, 1 << 4, 0xAA, 0xAA, 0xAA, 0xAA, 1, 2, 3, 4};
    CHECK_EQUAL(FUSB_FIFOS, address);
    memcpy(buf, virtualBuffer + pos, size);
    pos += size;
    CHECK_TRUE(pos <= sizeof(virtualBuffer));
    return true;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("Should not issue writes in read only functions");
    return false;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);
  pd_msg  msg;
  CHECK_EQUAL(0, f.fusb_read_message(&msg));
  CHECK_EQUAL(1, PD_NUMOBJ_GET(&msg));
}
TEST(FUSB, ReadIgnoredMessage) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    static uint8_t pos             = 0;
    const uint8_t  virtualBuffer[] = {0xD0, 0, 1 << 4, 0xAA, 0xAA, 0xAA, 0xAA, 1, 2, 3, 4};
    CHECK_EQUAL(FUSB_FIFOS, address);
    memcpy(buf, virtualBuffer + pos, size);
    pos += size;
    CHECK_TRUE(pos <= sizeof(virtualBuffer));
    return true;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("Should not issue writes in read only functions");
    return false;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);
  pd_msg  msg;
  CHECK_EQUAL(1, f.fusb_read_message(&msg));
  CHECK_EQUAL(1, PD_NUMOBJ_GET(&msg));
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

TEST(FUSB, SendHardReset) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("No Reads should be required");
    return false;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    CHECK_EQUAL(size, 1);
    CHECK_EQUAL(FUSB_CONTROL3, address);
    CHECK_EQUAL(buf[0], 0x07 | FUSB_CONTROL3_SEND_HARD_RESET);
    return true;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);
  f.fusb_send_hardrst();
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

  FUSB302::fusb_status statusOut;
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
    static uint8_t state = 0;
    switch (state) {
    case 0:
      CHECK_EQUAL(FUSB_DEVICE_ID, address);
      CHECK_EQUAL(1, size);
      buf[0] = 0xFF; // Fake bad read first time
      break;
    case 1:
      CHECK_EQUAL(FUSB_DEVICE_ID, address);
      CHECK_EQUAL(1, size);
      buf[0] = 1; // Good read second time
      break;
    case 2: // Reading CC1
      CHECK_EQUAL(FUSB_STATUS0, address);
      CHECK_EQUAL(1, size);
      buf[0] = 1; // Return lower level for cc1
      break;
    case 3: // Reading CC2
      CHECK_EQUAL(FUSB_STATUS0, address);
      CHECK_EQUAL(1, size);
      buf[0] = 2; // Return higher level for cc2
      break;
    default:
      FAIL("Unhandled read");
      break;
    }
    state++;
    return true;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    static uint8_t state = 0;
    CHECK_EQUAL(1, size);
    switch (state) {
    case 0: // issues reset first
      CHECK_EQUAL(FUSB_RESET, address);
      CHECK_EQUAL(FUSB_RESET_SW_RES, buf[0]);
      break;
    case 1: // Turns on all power sections
      CHECK_EQUAL(FUSB_POWER, address);
      CHECK_EQUAL(0x0F, buf[0]);
      break;
    case 2: // Turns on all interrupts
      CHECK_EQUAL(FUSB_MASK1, address);
      CHECK_EQUAL(0x00, buf[0]);
      break;
    case 3: // Turns on all interrupts
      CHECK_EQUAL(FUSB_MASKA, address);
      CHECK_EQUAL(0x00, buf[0]);
      break;
    case 4: // Turns on all interrupts
      CHECK_EQUAL(FUSB_MASKB, address);
      CHECK_EQUAL(0x00, buf[0]);
      break;
    case 5:
      CHECK_EQUAL(FUSB_CONTROL0, address);
      CHECK_EQUAL(0x03 << 2, buf[0]);
      break;
    case 6: // Enable auto re-send on error
      CHECK_EQUAL(FUSB_CONTROL3, address);
      CHECK_EQUAL(0x07, buf[0]);
      break;
    case 7: // Set defaults just-in-case
      CHECK_EQUAL(FUSB_CONTROL2, address);
      CHECK_EQUAL(0x00, buf[0]);
      break;
    case 8: // Issue buffer flush
      CHECK_EQUAL(FUSB_CONTROL1, address);
      CHECK_EQUAL(FUSB_CONTROL1_RX_FLUSH, buf[0]);
      break;
    case 9: // Enables measuring the CC 1
      CHECK_EQUAL(FUSB_SWITCHES0, address);
      CHECK_EQUAL(0x07, buf[0]);
      break;
    case 10: // Enables measuring the CC 2 pin
      CHECK_EQUAL(FUSB_SWITCHES0, address);
      CHECK_EQUAL(0x0B, buf[0]);
      break;
    case 11: // Selects to signal on cc2
      CHECK_EQUAL(FUSB_SWITCHES1, address);
      CHECK_EQUAL(0x26, buf[0]);
      break;
    case 12: // Selects to signal on cc2
      CHECK_EQUAL(FUSB_SWITCHES0, address);
      CHECK_EQUAL(0x0B, buf[0]);
      break;
    case 13:
      CHECK_EQUAL(FUSB_CONTROL0, address);
      CHECK_EQUAL(buf[0], 0x44);
      break;
    case 14:
      CHECK_EQUAL(FUSB_CONTROL1, address);
      CHECK_EQUAL(buf[0], FUSB_CONTROL1_RX_FLUSH);
      break;
    case 15:
      CHECK_EQUAL(FUSB_RESET, address);
      CHECK_EQUAL(buf[0], FUSB_RESET_PD_RESET);
      break;
    default:
      FAIL("Unhandled write");
      break;
    }
    state++;
    return true;
  };
  auto mock_delay = [](uint32_t millis) { CHECK_EQUAL(10, millis); };

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);

  f.fusb_setup();
}

TEST(FUSB, SendMessage) {
  auto mock_read = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    FAIL("No reads");
    return false;
  };
  auto mock_write = [](const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) -> bool {
    static uint8_t       step       = 0;
    static const uint8_t sop_seq[5] = {FUSB_FIFO_TX_SOP1, FUSB_FIFO_TX_SOP1, FUSB_FIFO_TX_SOP1, FUSB_FIFO_TX_SOP2, FUSB_FIFO_TX_PACKSYM | (2)};
    static const uint8_t eop_seq[4] = {FUSB_FIFO_TX_JAM_CRC, FUSB_FIFO_TX_EOP, FUSB_FIFO_TX_TXOFF, FUSB_FIFO_TX_TXON};

    CHECK_EQUAL(FUSB_FIFOS, address);
    switch (step) {
    case 0:
      CHECK_EQUAL(0, memcmp(sop_seq, buf, 5));
      CHECK_EQUAL(5, size);
      break;
    case 1:
      CHECK_EQUAL(2, size);
      break;
    case 2:
      CHECK_EQUAL(0, memcmp(eop_seq, buf, 4));
      CHECK_EQUAL(4, size);
      break;
    default:
      FAIL("Unhandled write");
    }
    step++;
    return true;
  };
  auto mock_delay = [](uint32_t millis) {};

  FUSB302 f = FUSB302(0x23 << 1, mock_read, mock_write, mock_delay);
  pd_msg  msg;
  memset(&msg, 0, sizeof(msg));
  f.fusb_send_message(&msg);
}