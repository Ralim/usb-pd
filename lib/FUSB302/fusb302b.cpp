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
#include <pd.h>
uint8_t fusb_read_byte(uint8_t addr);
bool    fusb_write_byte(uint8_t addr, uint8_t byte);
void    FUSB302::fusb_send_message(const pd_msg *msg) const {

  /* Token sequences for the FUSB302B */
  static uint8_t       sop_seq[5] = {FUSB_FIFO_TX_SOP1, FUSB_FIFO_TX_SOP1, FUSB_FIFO_TX_SOP1, FUSB_FIFO_TX_SOP2, FUSB_FIFO_TX_PACKSYM};
  static const uint8_t eop_seq[4] = {FUSB_FIFO_TX_JAM_CRC, FUSB_FIFO_TX_EOP, FUSB_FIFO_TX_TXOFF, FUSB_FIFO_TX_TXON};

  /* Take the I2C2 mutex now so there can't be a race condition on sop_seq */
  /* Get the length of the message: a two-octet header plus NUMOBJ four-octet
   * data objects */
  uint8_t msg_len = 2 + 4 * PD_NUMOBJ_GET(msg);

  /* Set the number of bytes to be transmitted in the packet */
  sop_seq[4] = FUSB_FIFO_TX_PACKSYM | msg_len;

  /* Write all three parts of the message to the TX FIFO */
  I2CWrite(DeviceAddress, FUSB_FIFOS, 5, sop_seq);
  I2CWrite(DeviceAddress, FUSB_FIFOS, msg_len, (uint8_t *)msg->bytes);
  I2CWrite(DeviceAddress, FUSB_FIFOS, 4, (uint8_t *)eop_seq);
}

bool FUSB302::fusb_rx_pending() const { return (fusb_read_byte(FUSB_STATUS1) & FUSB_STATUS1_RX_EMPTY) != FUSB_STATUS1_RX_EMPTY; }

uint8_t FUSB302::fusb_read_message(pd_msg *msg) const {

  static uint8_t garbage[4];
  uint8_t        numobj;

  // Read the header. If its not a SOP we dont actually want it at all
  // But on some revisions of the fusb if you dont both pick them up and read
  // them out of the fifo, it gets stuck
  // TODO this might need a tad more testing about how many bites we throw out
  if ((fusb_read_byte(FUSB_FIFOS) & FUSB_FIFO_RX_TOKEN_BITS) != FUSB_FIFO_RX_SOP) {
    return 1;
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

  return 0;
}

void FUSB302::fusb_send_hardrst() const {

  /* Send a hard reset */
  fusb_write_byte(FUSB_CONTROL3, 0x07 | FUSB_CONTROL3_SEND_HARD_RESET);
}

bool FUSB302::fusb_setup() const {
  /* Fully reset the FUSB302B */
  fusb_write_byte(FUSB_RESET, FUSB_RESET_SW_RES);
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
  fusb_write_byte(FUSB_POWER, 0x0F);

  /* Set interrupt masks */
  // Setting to 0 so interrupts are allowed
  fusb_write_byte(FUSB_MASK1, 0x00);
  fusb_write_byte(FUSB_MASKA, 0x00);
  fusb_write_byte(FUSB_MASKB, 0x00);
  fusb_write_byte(FUSB_CONTROL0, 0b11 << 2);

  /* Enable automatic retransmission */
  fusb_write_byte(FUSB_CONTROL3, 0x07);
  // set defaults
  fusb_write_byte(FUSB_CONTROL2, 0x00);
  /* Flush the RX buffer */
  fusb_write_byte(FUSB_CONTROL1, FUSB_CONTROL1_RX_FLUSH);

  /* Measure CC1 */
  fusb_write_byte(FUSB_SWITCHES0, 0x07);
  osDelay(10);
  uint8_t cc1 = fusb_read_byte(FUSB_STATUS0) & FUSB_STATUS0_BC_LVL;

  /* Measure CC2 */
  fusb_write_byte(FUSB_SWITCHES0, 0x0B);
  osDelay(10);
  uint8_t cc2 = fusb_read_byte(FUSB_STATUS0) & FUSB_STATUS0_BC_LVL;

  /* Select the correct CC line for BMC signaling; also enable AUTO_CRC */
  if (cc1 > cc2) {
    fusb_write_byte(FUSB_SWITCHES1, 0x25); // TX_CC1|AUTO_CRC|SPECREV0
    fusb_write_byte(FUSB_SWITCHES0, 0x07); // PWDN1|PWDN2|MEAS_CC1
  } else {
    fusb_write_byte(FUSB_SWITCHES1, 0x26); // TX_CC2|AUTO_CRC|SPECREV0
    fusb_write_byte(FUSB_SWITCHES0, 0x0B); // PWDN1|PWDN2|MEAS_CC2
  }

  fusb_reset();

  return true;
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

void FUSB302::fusb_reset() const {

  /* Flush the TX buffer */
  fusb_write_byte(FUSB_CONTROL0, 0x44);
  /* Flush the RX buffer */
  fusb_write_byte(FUSB_CONTROL1, FUSB_CONTROL1_RX_FLUSH);
  /* Reset the PD logic */
  fusb_write_byte(FUSB_RESET, FUSB_RESET_PD_RESET);
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
