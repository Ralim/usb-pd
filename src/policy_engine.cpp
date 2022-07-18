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
#include "fusb302_defines.h"
#include "fusb302b.h"
#include <pd.h>
#include <stdbool.h>
#ifdef PD_DEBUG_OUTPUT
#include "stdio.h"
#endif
void PolicyEngine::notify(PolicyEngine::Notifications notification) {
  uint32_t val = (uint32_t)notification;
  currentEvents |= val;
#ifdef PD_DEBUG_OUTPUT
  printf("Notification received  %04X\r\n", (int)notification);
#endif
}
void PolicyEngine::printStateName() {
#ifdef PD_DEBUG_OUTPUT
  const char *names[] = {"PEWaitingEvent",
                         "PEWaitingMessageTx",
                         "PEWaitingMessageGoodCRC",
                         "PESinkStartup",
                         "PESinkDiscovery",
                         "PESinkSetupWaitCap",
                         "PESinkWaitCap",
                         "PESinkEvalCap",
                         "PESinkSelectCapTx",
                         "PESinkSelectCap",
                         "PESinkWaitCapResp",
                         "PESinkTransitionSink",
                         "PESinkReady",
                         "PESinkGetSourceCap",
                         "PESinkGiveSinkCap",
                         "PESinkHardReset",
                         "PESinkTransitionDefault",
                         "PESinkSoftReset",
                         "PESinkSendSoftReset",
                         "PESinkSendSoftResetTxOK",
                         "PESinkSendSoftResetResp",
                         "PESinkSendNotSupported",
                         "PESinkHandleEPRChunk",
                         "PESinkNotSupportedReceived",
                         "PESinkSourceUnresponsive",
                         "PESinkEPREvalCap",
                         "PESinkRequestEPR",
                         "PESinkSendEPRKeepAlive",
                         "PESinkWaitEPRKeepAliveAck"};
  printf("Current state - %s\r\n", names[(int)state]);
#endif
}
bool PolicyEngine::thread() {
  auto stateEnter = state;
  switch (state) {

  case PESinkStartup:
    state = pe_sink_startup();
    break;
  case PESinkDiscovery:
    state = pe_sink_discovery();
    break;
  case PESinkSetupWaitCap:
    state = pe_sink_setup_wait_cap();
    break;
  case PESinkWaitCap:
    state = pe_sink_wait_cap();
    break;
  case PESinkEvalCap:
    state = pe_sink_eval_cap();
    break;
  case PESinkSelectCapTx:
    state = pe_sink_select_cap_tx();
    break;
  case PESinkSelectCap:
    state = pe_sink_select_cap();
    break;
  case PESinkWaitCapResp:
    state = pe_sink_wait_cap_resp();
    break;
  case PESinkTransitionSink:
    state = pe_sink_transition_sink();
    break;
  case PESinkReady:
    state = pe_sink_ready();
    break;
  case PESinkGetSourceCap:
    state = pe_sink_get_source_cap();
    break;
  case PESinkGiveSinkCap:
    state = pe_sink_give_sink_cap();
    break;
  case PESinkHardReset:
    state = pe_sink_hard_reset();
    break;
  case PESinkTransitionDefault:
    state = pe_sink_transition_default();
    break;
  case PESinkHandleSoftReset:
    state = pe_sink_soft_reset();
    break;
  case PESinkSendSoftReset:
    state = pe_sink_send_soft_reset();
    break;
  case PESinkSendSoftResetTxOK:
    state = pe_sink_send_soft_reset_tx_ok();
    break;
  case PESinkSendSoftResetResp:
    state = pe_sink_send_soft_reset_resp();
    break;
  case PESinkSendNotSupported:
    state = pe_sink_send_not_supported();
    break;
  case PESinkWaitForHandleEPRChunk:
    state = pe_sink_wait_epr_chunk();
    break;
  case PESinkHandleEPRChunk:
    state = pe_sink_handle_epr_chunk();
    break;
  case PESinkSourceUnresponsive:
    state = pe_sink_source_unresponsive();
    break;
  case PESinkNotSupportedReceived:
    state = pe_sink_not_supported_received();
    break;
  case PEWaitingEvent:
    state = pe_sink_wait_event();
    break;
  case PEWaitingMessageTx:
    state = pe_sink_wait_send_done();
    break;
  case PEWaitingMessageGoodCRC:
    state = pe_sink_wait_good_crc();
    break;
  case PESinkEPREvalCap:
    state = pe_sink_epr_eval_cap();
    break;
  case PESinkRequestEPR:
    state = pe_sink_request_epr();
    break;
  case PESinkSendEPRKeepAlive:
    state = pe_sink_send_epr_keep_alive();
    break;
  case PESinkWaitEPRKeepAliveAck:
    state = pe_sink_wait_epr_keep_alive_ack();
    break;
  default:
    state = PESinkStartup;
    break;
  }
#ifdef PD_DEBUG_OUTPUT
  if (state != PEWaitingEvent) {
    printStateName();
  }
#endif
  return (state != stateEnter) || (state != PEWaitingEvent);
}

bool PolicyEngine::isPD3_0() { return (hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0; }

bool PolicyEngine::NegotiationTimeoutReached(uint8_t timeout) {
  // Check if have been waiting longer than timeout without finishing
  // If so force state into the failed state and return true

  // Timeout is in 100ms increments
  // If the system ticks is greater than the specified timeout then we call it all off
  if (timeout) {
    if ((getTimeStamp() - timestampNegotiationsStarted) > (timeout * 100)) {
      // state = policy_engine_state::PESinkSourceUnresponsive;
      return true;
    }
  }
  return false;
}

void PolicyEngine::TimersCallback() {
  if (PPSTimerEnabled) {
    // Have to periodically re-send to keep the voltage level active
    if ((getTimeStamp() - PPSTimeLastEvent) > (1000)) {
      // Send a new PPS message
      PolicyEngine::notify(Notifications::PPS_REQUEST);
      PPSTimeLastEvent = getTimeStamp();
    }
  }
  if (is_epr) {
    // We need to engage in _some_ PD communication to stay in EPR mode
    if ((getTimeStamp() - EPRTimeLastEvent) > (200)) {
      PolicyEngine::notify(Notifications::EPR_KEEPALIVE);
    }
  }
}
PolicyEngine::policy_engine_state PolicyEngine::pe_start_message_tx(PolicyEngine::policy_engine_state postTxState, PolicyEngine::policy_engine_state txFailState, pd_msg *msg) {
#ifdef PD_DEBUG_OUTPUT
  printf("Starting message Tx - %02X\r\n", PD_MSGTYPE_GET(msg));
#endif
  if (PD_MSGTYPE_GET(msg) == PD_MSGTYPE_SOFT_RESET && PD_NUMOBJ_GET(msg) == 0) {
    /* Clear MessageIDCounter */
    _tx_messageidcounter = 0;
    return postTxState; // Message is "done"
  }
  postSendFailedState = txFailState;
  postSendState       = postTxState;
  msg->hdr &= ~PD_HDR_MESSAGEID;
  msg->hdr |= (_tx_messageidcounter % 8) << PD_HDR_MESSAGEID_SHIFT;

  /* PD 3.0 collision avoidance */
  // if (PolicyEngine::isPD3_0()) {
  //   /* If we're starting an AMS, wait for permission to transmit */
  //   while (fusb.fusb_get_typec_current() != fusb_sink_tx_ok) {
  //     osDelay(1);
  //   }
  // }
  /* Send the message to the PHY */
  fusb.fusb_send_message(msg);
#ifdef PD_DEBUG_OUTPUT
  printf("Message queued to send\r\n");
#endif

  // Setup waiting for notification
  return waitForEvent(PEWaitingMessageTx, (uint32_t)Notifications::RESET | (uint32_t)Notifications::MSG_RX | (uint32_t)Notifications::I_TXSENT | (uint32_t)Notifications::I_RETRYFAIL, 0xFFFFFFFF);
}

void PolicyEngine::clearEvents(uint32_t notification) { currentEvents &= ~notification; }

PolicyEngine::policy_engine_state PolicyEngine::waitForEvent(PolicyEngine::policy_engine_state evalState, uint32_t notification, uint32_t timeout) {
  // Record the new state, and the desired notifications mask, then schedule the waiter state
  waitingEventsMask = notification;
#ifdef PD_DEBUG_OUTPUT
  printf("Waiting for events %04X\r\n", (int)notification);
#endif

  // If notification is already present, we can continue straight to eval state
  if (currentEvents & waitingEventsMask) {
    return evalState;
  }
  // If waiting for message rx, but one is in the buffer, jump to eval
  if ((waitingEventsMask & (uint32_t)Notifications::MSG_RX) == (uint32_t)Notifications::MSG_RX) {
    if (incomingMessages.getOccupied() > 0) {
      currentEvents |= (uint32_t)Notifications::MSG_RX;
      return evalState;
    }
  }
  postNotificationEvalState = evalState;
  if (timeout == 0xFFFFFFFF) {
    waitingEventsTimeout = 0xFFFFFFFF;
  } else {
    waitingEventsTimeout = getTimeStamp() + timeout;
  }
  return policy_engine_state::PEWaitingEvent;
}
void PolicyEngine::readPendingMessage() {
  while (fusb.fusb_rx_pending()) {
    /* Read the message */
    if (fusb.fusb_read_message(&irqMessage) == 0) {
      /* If it's a Soft_Reset, go to the soft reset state */
      if (PD_MSGTYPE_GET(&irqMessage) == PD_MSGTYPE_SOFT_RESET && PD_NUMOBJ_GET(&irqMessage) == 0) {
        /* PE transitions to its reset state */
        notify(Notifications::RESET);
      } else {

        /* Pass the message to the policy engine. */
        incomingMessages.push(&irqMessage);

        notify(PolicyEngine::Notifications::MSG_RX);
      }
    } else {
      // Invalid message or SOP'
    }
  }
}

bool PolicyEngine::IRQOccured() {
  FUSB302::fusb_status status;
  bool                 returnValue = false;
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