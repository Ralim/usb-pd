/*
 * PD Buddy Firmware Library - USB Power Delivery for everyone
 * Copyright 2017-2018 Clayton G. Hobbs
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "fusb302b.h"
#include "fusb302_defines.h"
#include <pd.h>
#ifdef PD_DEBUG_OUTPUT
#include "stdio.h"
#endif
uint8_t fusb_read_byte(uint8_t addr);
bool    fusb_write_byte(uint8_t addr, uint8_t byte);
void    FUSB302::fusb_send_message(const pd_msg *msg) const {

  /* Token sequences for the FUSB302B */
  static uint8_t       sop_seq[5] = {FUSB_FIFO_TX_SOP1, FUSB_FIFO_TX_SOP1, FUSB_FIFO_TX_SOP1, FUSB_FIFO_TX_SOP2, FUSB_FIFO_TX_PACKSYM};
  static const uint8_t eop_seq[4] = {FUSB_FIFO_TX_JAM_CRC, FUSB_FIFO_TX_EOP, FUSB_FIFO_TX_TXOFF, FUSB_FIFO_TX_TXON};

  /* Get the length of the message: a two-octet header plus NUMOBJ four-octet
   * data objects */
  uint8_t msg_len = 2 + 4 * PD_NUMOBJ_GET(msg);

  /* Set the number of bytes to be transmitted in the packet */
  sop_seq[4] = FUSB_FIFO_TX_PACKSYM | msg_len;

  /* Write all three parts of the message to the TX FIFO */
  bool result = I2CWrite(DeviceAddress, FUSB_FIFOS, 5, sop_seq);
  if (!result) {
#ifdef PD_DEBUG_OUTPUT
    printf("I2CWrite failed 1\r\n");
#endif
  }
  result = I2CWrite(DeviceAddress, FUSB_FIFOS, msg_len, (uint8_t *)msg->bytes);
  if (!result) {
#ifdef PD_DEBUG_OUTPUT
    printf("I2CWrite failed 2\r\n");
#endif
  }

  result = I2CWrite(DeviceAddress, FUSB_FIFOS, 4, (uint8_t *)eop_seq);
  if (!result) {
#ifdef PD_DEBUG_OUTPUT
    printf("I2CWrite failed 3\r\n");
#endif
  }
}

bool FUSB302::fusb_rx_pending() const { return (fusb_read_byte(FUSB_STATUS1) & FUSB_STATUS1_RX_EMPTY) != FUSB_STATUS1_RX_EMPTY; }

uint8_t FUSB302::fusb_read_message(pd_msg *msg) const {

  static uint8_t garbage[4];
  uint8_t        numobj;

  // Read the header. If its not a SOP we dont actually want it at all
  // But on some revisions of the fusb if you dont both pick them up and read
  // them out of the fifo, it gets stuck
  // TODO this might need a tad more testing about how many bites we throw out, but believe it is correct
  uint8_t returnValue = 0;
  if ((fusb_read_byte(FUSB_FIFOS) & FUSB_FIFO_RX_TOKEN_BITS) != FUSB_FIFO_RX_SOP) {
    returnValue = 1;
  }

  /* Read the message header into msg */
  I2CRead(DeviceAddress, FUSB_FIFOS, 2, msg->bytes);
  /* Get the number of data objects */
  numobj = PD_NUMOBJ_GET(msg);
  /* If there is at least one data object, read the data objects */
  if (numobj > 0) {
    I2CRead(DeviceAddress, FUSB_FIFOS, numobj * 4, msg->bytes + 2);
  }
  /* Throw the CRC32 in the garbage, since the PHY already checked it. */
  I2CRead(DeviceAddress, FUSB_FIFOS, 4, garbage);

  return returnValue;
}

void FUSB302::fusb_send_hardrst() const {

  /* Send a hard reset */
  fusb_write_byte(FUSB_CONTROL3, 0x07 | FUSB_CONTROL3_SEND_HARD_RESET);
}

bool FUSB302::fusb_setup() const {
  /* Fully reset the FUSB302B */
  if (!fusb_write_byte(FUSB_RESET, FUSB_RESET_SW_RES)) {
    return false;
  }
  osDelay(10);
  uint8_t tries = 0;
  while (!fusb_read_id()) {
    osDelay(10);
    tries++;
    if (tries > 5) {
      return false; // Welp :(
    }
  }

  /* Turn on all power */
  if (!fusb_write_byte(FUSB_POWER, 0x0F)) {
    return false;
  }

  /* Set interrupt masks */
  // Setting to 0 so interrupts are allowed
  if (!fusb_write_byte(FUSB_MASK1, 0x00)) {
    return false;
  }
  if (!fusb_write_byte(FUSB_MASKA, 0x00)) {
    return false;
  }
  if (!fusb_write_byte(FUSB_MASKB, 0x00)) {
    return false;
  }
  if (!fusb_write_byte(FUSB_CONTROL0, 0b11 << 2)) {
    return false;
  }

  /* Enable automatic retransmission */
  if (!fusb_write_byte(FUSB_CONTROL3, 0x07)) {
    return false;
  }
  // set defaults
  if (!fusb_write_byte(FUSB_CONTROL2, 0x00)) {
    return false;
  }
  /* Flush the RX buffer */
  if (!fusb_write_byte(FUSB_CONTROL1, FUSB_CONTROL1_RX_FLUSH)) {
    return false;
  }

  if (!runCCLineSelection()) {
    return false;
  }
  if (!fusb_reset()) {
    return false;
  }

  return true;
}

bool FUSB302::runCCLineSelection() const {

  /* Measure CC1 */
  if (!fusb_write_byte(FUSB_SWITCHES0, 0x07)) {
    return false;
  }
  osDelay(10);
  uint8_t cc1 = fusb_read_byte(FUSB_STATUS0) & FUSB_STATUS0_BC_LVL;

  /* Measure CC2 */
  if (!fusb_write_byte(FUSB_SWITCHES0, 0x0B)) {
    return false;
  }
  osDelay(10);
  uint8_t cc2 = fusb_read_byte(FUSB_STATUS0) & FUSB_STATUS0_BC_LVL;

  /* Select the correct CC line for BMC signaling; also enable AUTO_CRC */
  if (cc1 > cc2) {
    // TX_CC1|AUTO_CRC|SPECREV0
    if (!fusb_write_byte(FUSB_SWITCHES1, 0x25)) {
      return false;
    }
    // PWDN1|PWDN2|MEAS_CC1
    if (!fusb_write_byte(FUSB_SWITCHES0, 0x07)) {
      return false;
    }
  } else {
    // TX_CC2|AUTO_CRC|SPECREV0
    if (!fusb_write_byte(FUSB_SWITCHES1, 0x26)) {
      return false;
    }
    // PWDN1|PWDN2|MEAS_CC2
    if (!fusb_write_byte(FUSB_SWITCHES0, 0x0B)) {
      return false;
    }
  }
  return true;
}

bool FUSB302::isVBUSConnected() const {
  // So we want to set MEAS_VBUS to enable measuring the VBus signal
  // Then check the status
  uint8_t measureBackup  = fusb_read_byte(FUSB_MEASURE);
  uint8_t switchesBackup = fusb_read_byte(FUSB_SWITCHES0);
  // clear MEAS_CCx bits
  fusb_write_byte(FUSB_SWITCHES0, switchesBackup & 0b11110011);
  osDelay(10);
  fusb_write_byte(FUSB_MEASURE, 0b01000000);
  osDelay(100);
  uint8_t status = fusb_read_byte(FUSB_STATUS0);
  // Write back original value
  fusb_write_byte(FUSB_MEASURE, measureBackup);
  fusb_write_byte(FUSB_SWITCHES0, switchesBackup);
  return (status & (0b00100000)) != 0;
}

bool FUSB302::fusb_get_status(fusb_status *status) const {

  /* Read the interrupt and status flags into status */
  return I2CRead(DeviceAddress, FUSB_STATUS0A, 7, status->bytes);
}

enum fusb_typec_current FUSB302::fusb_get_typec_current() const {

  /* Read the BC_LVL into a variable */
  enum fusb_typec_current bc_lvl = (enum fusb_typec_current)(fusb_read_byte(FUSB_STATUS0) & FUSB_STATUS0_BC_LVL);

  return bc_lvl;
}

bool FUSB302::fusb_reset() const {

  /* Flush the TX buffer */
  if (!fusb_write_byte(FUSB_CONTROL0, 0x44)) {
    return false;
  }
  /* Flush the RX buffer */
  if (!fusb_write_byte(FUSB_CONTROL1, FUSB_CONTROL1_RX_FLUSH)) {
    return false;
  }
  /* Reset the PD logic */
  if (!fusb_write_byte(FUSB_RESET, FUSB_RESET_PD_RESET)) {
    return false;
  }
  return true;
}

bool FUSB302::fusb_read_id() const {
  // Return true if read of the revision ID is sane
  uint8_t version = 0;

  bool res = I2CRead(DeviceAddress, FUSB_DEVICE_ID, 1, &version);
  if (!res)
    return res;
  if (version == 0 || version == 0xFF)
    return false;
  return true;
}
/*
 * Read a single byte from the FUSB302B
 *
 * cfg: The FUSB302B to communicate with
 * addr: The memory address from which to read
 *
 * Returns the value read from addr.
 */
uint8_t FUSB302::fusb_read_byte(const uint8_t addr) const {
  uint8_t data[1];
  if (!I2CRead(DeviceAddress, addr, 1, (uint8_t *)data)) {
    return 0;
  }
  return data[0];
}

/*
 * Write a single byte to the FUSB302B
 *
 * cfg: The FUSB302B to communicate with
 * addr: The memory address to which we will write
 * byte: The value to write
 */
bool FUSB302::fusb_write_byte(const uint8_t addr, const uint8_t byte) const { return I2CWrite(DeviceAddress, addr, 1, (uint8_t *)&byte); }
