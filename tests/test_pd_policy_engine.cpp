#include "CppUTest/TestHarness.h"
#include "fusb302_defines.h"
#include "fusb302b.h"
#include "mock_fusb302.h"
#include "policy_engine.h"
#include "user_functions.hpp"
#include <stdint.h>
#include <stdio.h>
// Working through testing the PD state machine
TEST_GROUP(PD){};
MockFUSB302 fusb_mock = MockFUSB302();
// remap's for usin the Mock
bool    i2c_read(const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) { return fusb_mock.i2cRead(deviceAddress, address, size, buf); }
bool    i2c_write(const uint8_t deviceAddress, const uint8_t address, const uint8_t size, uint8_t *buf) { return fusb_mock.i2cWrite(deviceAddress, address, size, buf); }
auto    mock_delay     = [](uint32_t millis) {};
auto    mock_timestamp = []() -> uint32_t { return 0; };
FUSB302 fusb           = FUSB302(FUSB302B_ADDR, i2c_read, i2c_write, mock_delay);

PolicyEngine pe = PolicyEngine(fusb, mock_timestamp, mock_delay, pdbs_dpm_get_sink_capability, pdbs_dpm_evaluate_capability, EPREvaluateCapabilityFunc, 140);
// Testing constants
const uint8_t message_SOP1[]                 = {FUSB_FIFO_RX_SOP1, 0, 0, 1, 2, 3, 4};
const uint8_t message_SOP2[]                 = {FUSB_FIFO_RX_SOP2, 0, 0, 1, 2, 3, 4};
const uint8_t message_good_crc[]             = {FUSB_FIFO_RX_SOP, PD_MSGTYPE_GOODCRC, 0, 0, 0, 0, 0}; // good crc with transaction counter of 0
const uint8_t message_request_capabilities[] = {FUSB_FIFO_RX_SOP, PD_MSGTYPE_GET_SINK_CAP, 0, 0, 0, 0, 0};
const uint8_t message_accept[]               = {FUSB_FIFO_RX_SOP, 0x63, 0x03, 0, 0, 0, 0}; // PS_ACCEPT
const uint8_t mock_capabilities[]            = {FUSB_FIFO_RX_SOP,
                                     0xA1, // Header
                                     0x71, // Header
                                     0x2c, // +
                                     0x91, // | Fixed 5V @ 3A
                                     0x01, // |
                                     0x08, // +
                                     0x2c, // +
                                     0xD1, // |
                                     0x02, // | Fixed 9V @ 3A
                                     0x00, // +
                                     0x2C, // +
                                     0xB1, // |
                                     0x04, // | Fixed 15V @ 3A
                                     0x00, // +
                                     0xE1, // +
                                     0x40, // |
                                     0x06, // | Fixed 20V @ 2.25A
                                     0x00, // +
                                     0x64, // +
                                     0x21, // |
                                     0xDC, // | PPS 3.3-11V @ 5A
                                     0xC8, // +
                                     0x3C, // +
                                     0x21, // |
                                     0x40, // | PPS 3.3-16V @ 3A
                                     0xC9, // +
                                     0x2D, // +
                                     0x21, // |
                                     0xA4, // | PPS 3.3-21V @ 2.25A
                                     0xC9, // +
                                     0,    // 0=CRC padding
                                     0,
                                     0,
                                     0};
const uint8_t message_ready[]                = {FUSB_FIFO_RX_SOP, 0x66, 0x05, 0, 0, 0, 0}; // PS_READY with 0'ed CRC
// Testing helpers

void injectTestmessage(const uint8_t len, const uint8_t *data) {

  fusb_mock.addToFIFO(len, data);
  fusb_mock.setRegister(FUSB_INTERRUPTB, FUSB_INTERRUPTB_I_GCRCSENT);
  CHECK_TRUE(fusb.fusb_rx_pending());
  pe.IRQOccured();
  CHECK_FALSE(fusb.fusb_rx_pending());
}

void iterateThoughExpectedStates(std::vector<int> expectedStates) {
  // Run state machine through all states until it stops and assert it goes through the expected states
  size_t iterationCounter = 0;
  while (pe.thread()) {
    CHECK_TRUE(iterationCounter < expectedStates.size());
    CHECK_EQUAL(expectedStates[iterationCounter], pe.currentStateCode());
    CHECK_TRUE(iterationCounter < 5);
    pe.printStateName();
    iterationCounter++;
  }
  pe.printStateName();
  // Check that we are at the final expected state
  CHECK_TRUE((iterationCounter + 1) == expectedStates.size());
  CHECK_EQUAL(expectedStates[iterationCounter], pe.currentStateCode());
}

void test_NormalNegotiation() {

  // Next queue an actual capabilities message

  injectTestmessage(sizeof(mock_capabilities), mock_capabilities);
  iterateThoughExpectedStates({6, 7, 8, 0});

  // We should now be able to pop the transmitted message out of the fifo
  const uint8_t expectedPD21VPPSMessage[] = {18, 18, 18, 19, 134, 130, 16, 45, 52, 8, 115, 255, 20, 254, 161};
  uint8_t       sendMessage[sizeof(expectedPD21VPPSMessage)];
  CHECK_EQUAL(5 + 6 + 4, sizeof(expectedPD21VPPSMessage));
  CHECK_TRUE(fusb_mock.readFiFo(sizeof(expectedPD21VPPSMessage), sendMessage));
  CHECK_TRUE(fusb_mock.fifoEmpty()); // Assert we read it all out
  for (size_t i = 0; i < sizeof(expectedPD21VPPSMessage); i++) {
    // std::cout << "Validate Request Message" << i << "->" << (int)sendMessage[i] << "|" << (int)expectedPD21VPPSMessage[i] << std::endl;
    CHECK_EQUAL(sendMessage[i], expectedPD21VPPSMessage[i]);
  }

  fusb_mock.setRegister(FUSB_INTERRUPTA, FUSB_INTERRUPTA_I_TXSENT);
  pe.IRQOccured();
  iterateThoughExpectedStates({1, 0});
  fusb_mock.setRegister(FUSB_INTERRUPTA, 0);
  // Now that tx has "sent" the charger will send a good crc back
  std::cout << "Faking Good CRC" << std::endl;

  injectTestmessage(sizeof(message_good_crc), message_good_crc);

  iterateThoughExpectedStates({2, 9, 0});
  // Now the unit should be waiting for the acceptance message from the power adapter
  CHECK_EQUAL(0, pe.currentStateCode());

  // Now that it has sent a "good" request, respond in kind with an acceptance

  injectTestmessage(sizeof(message_accept), message_accept);
  iterateThoughExpectedStates({10, 0});
  // Now send the PS_RDY
  injectTestmessage(sizeof(message_ready), message_ready);
  iterateThoughExpectedStates({11, 12, 0});
}

// Test Scenarios
TEST(PD, PDNegotiationTest) {
  // Testing states
  CHECK_FALSE(pe.isPD3_0());
  CHECK_FALSE(pe.NegotiationTimeoutReached(100));
  CHECK_FALSE(pe.pdHasNegotiated());
  // CHECK_FALSE(pe.setupCompleteOrTimedOut(100));
  // Crank the handle until we are waiting for a pd message to come in, but with deadlock detection
  iterateThoughExpectedStates({4, 5, 0});
  // Now the thread should wait for an IRQ to signify it needs to poll data from the FUSB302
  // First load up a SOP' message into the FIFO that it will need to ignore
  injectTestmessage(sizeof(message_SOP1), message_SOP1);
  iterateThoughExpectedStates({0});
  // Also test SOP" are ignored
  injectTestmessage(sizeof(message_SOP2), message_SOP2);
  iterateThoughExpectedStates({0});
  // Now wind up to normal state (negotiated)
  test_NormalNegotiation();
  // Test that the unit can be asked for its capabilities
  injectTestmessage(sizeof(message_request_capabilities), message_request_capabilities);
  // The unit will then transition to send its capabilities
  iterateThoughExpectedStates({12, 14, 0});
  // Read out the transmitted message
  const uint8_t expectedDeviceCaps[] = {18, 18, 18, 19, 142, 4, 50, 10, 144, 1, 28, 200, 64, 6, 0, 40, 200, 144, 193, 255, 20, 254, 161};
  uint8_t       sendMessage[sizeof(expectedDeviceCaps)];
  CHECK_TRUE(fusb_mock.readFiFo(sizeof(expectedDeviceCaps), sendMessage));
  CHECK_TRUE(fusb_mock.fifoEmpty());
  for (size_t i = 0; i < sizeof(expectedDeviceCaps); i++) {
    // std::cout << "Validate Caps Message " << i << "->" << (int)sendMessage[i] << "|" << (int)expectedDeviceCaps[i] << std::endl;
    CHECK_EQUAL(sendMessage[i], expectedDeviceCaps[i]);
  }

  fusb_mock.setRegister(FUSB_INTERRUPTA, FUSB_INTERRUPTA_I_TXSENT);
  pe.IRQOccured();
  iterateThoughExpectedStates({1, 0});
  fusb_mock.setRegister(FUSB_INTERRUPTA, 0);
}
