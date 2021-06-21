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
#include "fusb302b.h"
#include <pd.h>
#include <stdbool.h>

void PolicyEngine::notify(PolicyEngine::Notifications notification) {
  uint32_t val = (uint32_t)notification;
  currentEvents |= val;
}

void PolicyEngine::thread() {

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
  case PESinkSoftReset:
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
  case PESinkChunkReceived:
    state = pe_sink_chunk_received();
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
  default:
    state = PESinkStartup;
    break;
  }
}

bool PolicyEngine::isPD3_0() { return (hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0; }

void PolicyEngine::handleMessage() { notify(PolicyEngine::Notifications::PDB_EVT_PE_MSG_RX); }

void PolicyEngine::PPSTimerCallback() {
  if (PPSTimerEnabled && state == policy_engine_state::PESinkReady) {
    // Have to periodically re-send to keep the voltage level active
    if ((getTimeStamp() - PPSTimeLastEvent) > (1000)) {
      // Send a new PPS message
      PolicyEngine::notify(Notifications::PDB_EVT_PE_PPS_REQUEST);
      PPSTimeLastEvent = getTimeStamp();
    }
  }
}
PolicyEngine::policy_engine_state PolicyEngine::pe_start_message_tx(PolicyEngine::policy_engine_state postTxState, PolicyEngine::policy_engine_state txFailState, pd_msg *msg) {
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
  if (PolicyEngine::isPD3_0()) {
    /* If we're starting an AMS, wait for permission to transmit */
    //    while (fusb_get_typec_current() != fusb_sink_tx_ok) {
    //      vTaskDelay(TICKS_10MS);
    //    }
  }
  /* Send the message to the PHY */
  fusb.fusb_send_message(msg);
  // Setup waiting for notification
  return waitForEvent(PEWaitingMessageTx,
                      (uint32_t)Notifications::PDB_EVT_PE_RESET | (uint32_t)Notifications::PDB_EVT_TX_DISCARD | (uint32_t)Notifications::PDB_EVT_TX_I_TXSENT
                          | (uint32_t)Notifications::PDB_EVT_TX_I_RETRYFAIL,
                      0xFFFFFFFF);
}

/*
 * Find the index of the first PDO from capabilities in the voltage range,
 * using the desired order.
 *
 * If there is no such PDO, returns -1 instead.
 */
int8_t PolicyEngine::dpm_get_range_fixed_pdo_index(const pd_msg *caps) {
  /* Get the number of PDOs */
  uint8_t numobj = PD_NUMOBJ_GET(caps);

  /* Get ready to iterate over the PDOs */
  int8_t i;
  int8_t step;
  i                   = numobj - 1;
  step                = -1;
  uint16_t current    = 100; // in centiamps
  uint16_t voltagemin = 8000;
  uint16_t voltagemax = 10000;
  /* Look at the PDOs to see if one falls in our voltage range. */
  while (0 <= i && i < numobj) {
    /* If we have a fixed PDO, its V is within our range, and its I is at
     * least our desired I */
    uint16_t v = PD_PDO_SRC_FIXED_VOLTAGE_GET(caps->obj[i]);
    if ((caps->obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_FIXED) {
      if (PD_PDO_SRC_FIXED_CURRENT_GET(caps->obj[i]) >= current) {
        if (v >= PD_MV2PDV(voltagemin) && v <= PD_MV2PDV(voltagemax)) {
          return i;
        }
      }
    }
    i += step;
  }
  return -1;
}

void PolicyEngine::clearEvents(uint32_t notification) { currentEvents &= ~notification; }

PolicyEngine::policy_engine_state PolicyEngine::waitForEvent(PolicyEngine::policy_engine_state evalState, uint32_t notification, uint32_t timeout) {
  // Record the new state, and the desired notifications mask, then schedule the waiter state
  waitingEventsMask = notification;
  // If notification is already present, we can continue straight to eval state
  if (currentEvents & waitingEventsMask) {
    return evalState;
  }
  postNotifcationEvalState = evalState;
  waitingEventsTimeout     = getTimeStamp() + timeout;
  return policy_engine_state::PEWaitingEvent;
}
