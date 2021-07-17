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

#include "policy_engine.h"
#include <pd.h>
#include <string.h>
void PolicyEngine::readPendingMessage() {
  while (fusb.fusb_rx_pending()) {
    /* Read the message */
    if (fusb.fusb_read_message(&irqMessage) == 0) {
      /* If it's a Soft_Reset, go to the soft reset state */
      if (PD_MSGTYPE_GET(&irqMessage) == PD_MSGTYPE_SOFT_RESET && PD_NUMOBJ_GET(&irqMessage) == 0) {
        /* PE transitions to its reset state */
        notify(Notifications::RESET);
      } else {
        /* Tell PolicyEngine to discard the message being transmitted */
        notify(Notifications::DISCARD);
        /* Pass the message to the policy engine. */
        incomingMessages.push(&irqMessage);

        notify(PolicyEngine::Notifications::MSG_RX);
      }
    } else {
      // Invalid message or SOP', still discard tx message
      /* Tell PolicyEngine to discard the message being transmitted */
      notify(Notifications::DISCARD);
    }
  }
}

bool PolicyEngine::IRQOccured() {
  fusb_status status;
  bool        returnValue = false;
  /* Read the FUSB302B status and interrupt registers */
  if (fusb.fusb_get_status(&status)) {

    /* If the I_GCRCSENT flag is set, tell the Protocol RX thread */
    // This means a message was received with a good CRC
    if (status.interruptb & FUSB_INTERRUPTB_I_GCRCSENT) {
      readPendingMessage();
      returnValue = true;
    }

    /* If the I_TXSENT or I_RETRYFAIL flag is set, tell the Protocol TX
     * thread */
    if (status.interrupta & FUSB_INTERRUPTA_I_TXSENT) {
      notify(Notifications::I_TXSENT);
      returnValue = true;
    }
    if (status.interrupta & FUSB_INTERRUPTA_I_RETRYFAIL) {
      notify(Notifications::I_RETRYFAIL);
      returnValue = true;
    }

    /* If the I_OCP_TEMP and OVRTEMP flags are set, tell the Policy
     * Engine thread */
    if ((status.interrupta & FUSB_INTERRUPTA_I_OCP_TEMP) && (status.status1 & FUSB_STATUS1_OVRTEMP)) {
      notify(Notifications::I_OVRTEMP);
      returnValue = true;
    }
  }
  return returnValue;
}
