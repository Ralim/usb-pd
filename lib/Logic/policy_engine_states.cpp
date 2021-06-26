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
#include "policy_engine.h"
#include <iostream>
#include <pd.h>
#include <stdbool.h>

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_startup() {
  /* We don't have an explicit contract currently */
  _explicit_contract = false;
  PPSTimerEnabled    = false;
  currentEvents      = 0;
  // If desired could send an alert that PD is starting

  /* No need to reset the protocol layer here.  There are two ways into this
   * state: startup and exiting hard reset.  On startup, the protocol layer
   * is reset by the startup procedure.  When exiting hard reset, the
   * protocol layer is reset by the hard reset state machine.  Since it's
   * already done somewhere else, there's no need to do it again here. */

  return PESinkDiscovery;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_discovery() {
  /* Wait for VBUS.  Since it's our only power source, we already know that
   * we have it, so just move on. */

  return PESinkSetupWaitCap;
}
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_setup_wait_cap() { //
  return waitForEvent(policy_engine_state::PESinkWaitCap, (uint32_t)Notifications::MSG_RX | (uint32_t)Notifications::I_OVRTEMP | (uint32_t)Notifications::RESET,
                      // Wait for cap timeout
                      PD_T_TYPEC_SINK_WAIT_CAP);
}
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_wait_cap() {
  /* Fetch a message from the protocol layer */
  uint32_t evt = currentEvents;
  clearEvents();

  /* If we're too hot, we shouldn't negotiate power yet */
  if (evt & (uint32_t)Notifications::I_OVRTEMP) {
    std::cout << "I_OVRTEMP" << std::endl;
    return PESinkSetupWaitCap;
  }

  /* If we got a message */
  if (evt & (uint32_t)Notifications::MSG_RX) {
    std::cout << "MSG_RX" << std::endl;
    /* Get the message */
    while (rxMessageWaiting) {
      memcpy(&tempMessage, &rxMessage, sizeof(rxMessage));
      rxMessageWaiting = false;
      /* If we got a Source_Capabilities message, read it. */
      std::cout << "Type" << PD_MSGTYPE_GET(&tempMessage) << std::endl;
      if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_SOURCE_CAPABILITIES && PD_NUMOBJ_GET(&tempMessage) > 0) {
        /* First, determine what PD revision we're using */
        if ((hdr_template & PD_HDR_SPECREV) == PD_SPECREV_1_0) {
          /* If the other end is using at least version 3.0, we'll
           * use version 3.0. */
          if ((tempMessage.hdr & PD_HDR_SPECREV) >= PD_SPECREV_3_0) {
            hdr_template |= PD_SPECREV_3_0;
            /* Otherwise, use 2.0.  Don't worry about the 1.0 case
             * because we don't have hardware for PD 1.0 signaling. */
          } else {
            hdr_template |= PD_SPECREV_2_0;
          }
        }
        return PESinkEvalCap;
      }
    }
  }

  /* If we failed to get a message, wait longer */
  std::cout << "NoMessages" << std::endl;
  return PESinkSetupWaitCap;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_eval_cap() {
  /* If we have a Source_Capabilities message, remember the index of the
   * first PPS APDO so we can check if the request is for a PPS APDO in
   * PE_SNK_Select_Cap. */
  /* Start by assuming we won't find a PPS APDO (set the index greater
   * than the maximum possible) */
  _pps_index = 0xFF;
  /* New capabilities also means we can't be making a request from the
   * same PPS APDO */
  /* Search for the first PPS APDO */
  for (int i = 0; i < PD_NUMOBJ_GET(&tempMessage); i++) {
    if ((tempMessage.obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_AUGMENTED && (tempMessage.obj[i] & PD_APDO_TYPE) == PD_APDO_TYPE_PPS) {
      _pps_index = i + 1;
      break;
    }
  }
  _unconstrained_power = tempMessage.obj[0] & PD_PDO_SRC_FIXED_UNCONSTRAINED;

  /* Ask the DPM what to request */
  if (pdbs_dpm_evaluate_capability(&tempMessage, &_last_dpm_request)) {
    _last_dpm_request.hdr |= hdr_template;
    /* If we're using PD 3.0 */
    if ((hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0) {
      /* If the request was for a PPS APDO, start time callbacks if not started
       */
      if (PD_RDO_OBJPOS_GET(&_last_dpm_request) >= _pps_index) {
        PPSTimerEnabled = true;
      } else {
        PPSTimerEnabled = false;
      }
    }
    return PESinkSelectCapTx;
  }

  return PESinkWaitCap;
}
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_select_cap_tx() {

  /* Transmit the request */
  clearEvents(); // clear all pending incase of an rx while prepping

  return pe_start_message_tx(policy_engine_state::PESinkSelectCap, PESinkHardReset, &_last_dpm_request);
}
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_select_cap() {
  // Have transmitted the selected cap, transition to waiting for the response
  clearEvents();
  // wait for a response
  return waitForEvent(PESinkWaitCapResp, (uint32_t)Notifications::MSG_RX | (uint32_t)Notifications::RESET | (uint32_t)Notifications::TIMEOUT, PD_T_SENDER_RESPONSE);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_wait_cap_resp() {
  /* Wait for a response */
  uint32_t evt = currentEvents;
  clearEvents();

  /* Get the response message */
  if (rxMessageWaiting) {
    memcpy(&tempMessage, &rxMessage, sizeof(rxMessage));
    rxMessageWaiting = false;
    /* If the source accepted our request, wait for the new power */
    if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_ACCEPT && PD_NUMOBJ_GET(&tempMessage) == 0) {
      return waitForEvent(PESinkTransitionSink, (uint32_t)Notifications::MSG_RX | (uint32_t)Notifications::RESET, PD_T_PS_TRANSITION);
      /* If the message was a Soft_Reset, do the soft reset procedure */
    } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_SOFT_RESET && PD_NUMOBJ_GET(&tempMessage) == 0) {
      return PESinkSoftReset;
      /* If the message was Wait or Reject */
    } else if ((PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_REJECT || PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_WAIT) && PD_NUMOBJ_GET(&tempMessage) == 0) {
      /* If we don't have an explicit contract, wait for capabilities */
      if (!_explicit_contract) {
        return PESinkSetupWaitCap;
        /* If we do have an explicit contract, go to the ready state */
      } else {
        return waitForEvent(PESinkReady, (uint32_t)Notifications::ALL, 0xFFFFFFFF);
      }
    } else {
      return PESinkSoftReset;
    }
  }
  return PESinkHardReset;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_transition_sink() {
  /* Wait for the PS_RDY message */
  uint32_t evt = currentEvents;
  clearEvents();
  /* If we received a message, read it */
  while (rxMessageWaiting) {
    memcpy(&tempMessage, &rxMessage, sizeof(rxMessage));
    rxMessageWaiting = false;
    /* If we got a PS_RDY, handle it */
    if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_PS_RDY && PD_NUMOBJ_GET(&tempMessage) == 0) {
      /* We just finished negotiating an explicit contract */
      _explicit_contract = true;

      /* Negotiation finished */
      return PESinkReady;
      /* If there was a protocol error, send a hard reset */
    }
  }
  // Timeout
  return PESinkSoftReset;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_ready() {
  uint32_t evt = currentEvents;
  clearEvents();
  /* If SinkPPSPeriodicTimer ran out, send a new request */
  if (evt & (uint32_t)Notifications::PPS_REQUEST) {
    return PESinkSelectCap;
  }

  /* If we overheated, send a hard reset */
  if (evt & (uint32_t)Notifications::I_OVRTEMP) {
    return PESinkHardReset;
  }
  /* If the DPM wants us to, send a Get_Source_Cap message */
  if (evt & (uint32_t)Notifications::GET_SOURCE_CAP) {
    return PESinkGetSourceCap;
  }
  /* If the DPM wants new power, let it figure out what power it wants
   * exactly.  This isn't exactly the transition from the spec (that would be
   * SelectCap, not EvalCap), but this works better with the particular
   * design of this firmware. */
  if (evt & (uint32_t)Notifications::NEW_POWER) {
    /* Tell the protocol layer we're starting an AMS */
    return PESinkEvalCap;
  }

  /* If we received a message */
  if (evt & (uint32_t)Notifications::MSG_RX) {
    if (rxMessageWaiting) {
      memcpy(&tempMessage, &rxMessage, sizeof(rxMessage));
      rxMessageWaiting = false;
      /* Ignore vendor-defined messages */
      if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_VENDOR_DEFINED && PD_NUMOBJ_GET(&tempMessage) > 0) {
        return waitForEvent(PESinkReady, (uint32_t)Notifications::ALL);
        /* Ignore Ping messages */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_PING && PD_NUMOBJ_GET(&tempMessage) == 0) {
        return waitForEvent(PESinkReady, (uint32_t)Notifications::ALL);
        /* DR_Swap messages are not supported */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_DR_SWAP && PD_NUMOBJ_GET(&tempMessage) == 0) {
        return PESinkSendNotSupported;
        /* Get_Source_Cap messages are not supported */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_GET_SOURCE_CAP && PD_NUMOBJ_GET(&tempMessage) == 0) {
        return PESinkSendNotSupported;
        /* PR_Swap messages are not supported */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_PR_SWAP && PD_NUMOBJ_GET(&tempMessage) == 0) {
        return PESinkSendNotSupported;
        /* VCONN_Swap messages are not supported */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_VCONN_SWAP && PD_NUMOBJ_GET(&tempMessage) == 0) {
        return PESinkSendNotSupported;
        /* Request messages are not supported */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_REQUEST && PD_NUMOBJ_GET(&tempMessage) > 0) {
        return PESinkSendNotSupported;
        /* Sink_Capabilities messages are not supported */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_SINK_CAPABILITIES && PD_NUMOBJ_GET(&tempMessage) > 0) {
        return PESinkSendNotSupported;
        /* Handle GotoMin messages */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_GOTOMIN && PD_NUMOBJ_GET(&tempMessage) == 0) {
        return PESinkSendNotSupported;
        /* Evaluate new Source_Capabilities */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_SOURCE_CAPABILITIES && PD_NUMOBJ_GET(&tempMessage) > 0) {
        return PESinkEvalCap;
        /* Give sink capabilities when asked */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_GET_SINK_CAP && PD_NUMOBJ_GET(&tempMessage) == 0) {
        return PESinkGiveSinkCap;
        /* If the message was a Soft_Reset, do the soft reset procedure */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_SOFT_RESET && PD_NUMOBJ_GET(&tempMessage) == 0) {
        return PESinkSoftReset;
        /* PD 3.0 messges */
      } else if ((hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0) {
        /* If the message is a multi-chunk extended message, let it
         * time out. */
        if ((tempMessage.hdr & PD_HDR_EXT) && (PD_DATA_SIZE_GET(&tempMessage) > PD_MAX_EXT_MSG_LEGACY_LEN)) {

          return PESinkChunkReceived;
          /* Tell the DPM a message we sent got a response of
           * Not_Supported. */
        } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_NOT_SUPPORTED && PD_NUMOBJ_GET(&tempMessage) == 0) {

          return PESinkNotSupportedReceived;
          /* If we got an unknown message, send a soft reset */
        } else {

          return PESinkSendSoftReset;
        }
      }
    }
  }

  return waitForEvent(PESinkReady, (uint32_t)Notifications::ALL, 0xFFFFFFFF);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_get_source_cap() {
  /* Get a message object */
  pd_msg *get_source_cap = &tempMessage;
  /* Make a Get_Source_Cap message */
  get_source_cap->hdr = hdr_template | PD_MSGTYPE_GET_SOURCE_CAP | PD_NUMOBJ(0);
  /* Transmit the Get_Source_Cap */
  // On fail -> hard reset, on send -> Sink Ready
  return pe_start_message_tx(PESinkReady, PESinkHardReset, get_source_cap);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_give_sink_cap() {
  /* Get a message object */
  pd_msg *snk_cap = &tempMessage;
  /* Get our capabilities from the DPM */
  pdbs_dpm_get_sink_capability(snk_cap, ((hdr_template & PD_HDR_SPECREV) >= PD_SPECREV_3_0));
  /* Transmit our capabilities */
  return pe_start_message_tx(PESinkReady, PESinkHardReset, snk_cap);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_hard_reset() {
  /* If we've already sent the maximum number of hard resets, assume the
   * source is unresponsive. */
  if (_hard_reset_counter > PD_N_HARD_RESET_COUNT) {
    return PESinkSourceUnresponsive;
  }
  // So, we could send a hardreset here; however that will cause a power cycle
  // on the PSU end.. Which will then reset this MCU So therefore we went get
  // anywhere :)
  /* Increment HardResetCounter */
  _hard_reset_counter++;

  return PESinkTransitionDefault;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_transition_default() {
  _explicit_contract = false;

  /* There is no local hardware to reset. */
  /* Since we never change our data role from UFP, there is no reason to set
   * it here. */

  return PESinkStartup;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_soft_reset() {
  /* No need to explicitly reset the protocol layer here.  It resets itself
   * when a Soft_Reset message is received. */

  /* Get a message object */
  pd_msg accept;
  /* Make an Accept message */
  accept.hdr = hdr_template | PD_MSGTYPE_ACCEPT | PD_NUMOBJ(0);
  /* Transmit the Accept */
  return pe_start_message_tx(PESinkSetupWaitCap, PESinkHardReset, &accept);
}
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_send_soft_reset() {
  /* No need to explicitly reset the protocol layer here.  It resets itself
   * just before a Soft_Reset message is transmitted. */

  /* Get a message object */
  pd_msg *softrst = &tempMessage;
  /* Make a Soft_Reset message */
  softrst->hdr = hdr_template | PD_MSGTYPE_SOFT_RESET | PD_NUMOBJ(0);
  /* Transmit the soft reset */
  return pe_start_message_tx(PESinkSendSoftResetTxOK, PESinkHardReset, softrst);
}
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_send_soft_reset_tx_ok() {
  // Transmit is good, wait for response event
  return waitForEvent(PESinkSendSoftResetResp, (uint32_t)Notifications::TIMEOUT | (uint32_t)Notifications::MSG_RX | (uint32_t)Notifications::RESET, PD_T_SENDER_RESPONSE);
}
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_send_soft_reset_resp() {

  /* Wait for a response */
  uint32_t evt = currentEvents;
  clearEvents();

  /* Get the response message */
  if (rxMessageWaiting) {
    memcpy(&tempMessage, &rxMessage, sizeof(rxMessage));
    rxMessageWaiting = false;
    /* If the source accepted our soft reset, wait for capabilities. */
    if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_ACCEPT && PD_NUMOBJ_GET(&tempMessage) == 0) {

      return PESinkSetupWaitCap;
      /* If the message was a Soft_Reset, do the soft reset procedure */
    } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_SOFT_RESET && PD_NUMOBJ_GET(&tempMessage) == 0) {

      return PESinkSoftReset;
      /* Otherwise, send a hard reset */
    } else {
      return PESinkHardReset;
    }
  }
  return PESinkHardReset;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_send_not_supported() {
  /* Get a message object */

  if ((hdr_template & PD_HDR_SPECREV) == PD_SPECREV_2_0) {
    /* Make a Reject message */
    tempMessage.hdr = hdr_template | PD_MSGTYPE_REJECT | PD_NUMOBJ(0);
  } else if ((hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0) {
    /* Make a Not_Supported message */
    tempMessage.hdr = hdr_template | PD_MSGTYPE_NOT_SUPPORTED | PD_NUMOBJ(0);
  }

  /* Transmit the message */
  return pe_start_message_tx(PESinkReady, PESinkSendSoftReset, &tempMessage);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_chunk_received() {

  /* Wait for tChunkingNotSupported */
  osDelay(PD_T_CHUNKING_NOT_SUPPORTED);

  return PESinkSendNotSupported;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_not_supported_received() {
  /* Inform the Device Policy Manager that we received a Not_Supported
   * message. */

  return waitForEvent(PESinkReady, (uint32_t)Notifications::ALL, 0xFFFFFFFF);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_source_unresponsive() {
  // Sit and chill, as PD is not working
  osDelay(PD_T_PD_DEBOUNCE);

  return PESinkSourceUnresponsive;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_wait_event() {
  // Check timeout
  if (getTimeStamp() > waitingEventsTimeout) {
    std::cout << "Timeout ! " << getTimeStamp() << std::endl;
    notify(Notifications::TIMEOUT);
  }
  if (currentEvents & (uint32_t)Notifications::TIMEOUT) {
    return PESinkSoftReset;
  }
  if (currentEvents & (uint32_t)Notifications::RESET) {
    return PESinkTransitionDefault;
  }

  std::cout << "Current events " << currentEvents << " Wanted " << waitingEventsMask << std::endl;
  if (currentEvents & waitingEventsMask) {
    return postNotifcationEvalState;
  }
  return policy_engine_state::PEWaitingEvent;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_wait_good_crc() {
  clearEvents();

  if (rxMessageWaiting) {
    // Wait for the Good CRC
    pd_msg goodcrc;

    /* Read the GoodCRC */
    memcpy(&goodcrc, &rxMessage, sizeof(rxMessage));
    rxMessageWaiting = false;

    /* Check that the message is correct */
    if (PD_MSGTYPE_GET(&goodcrc) == PD_MSGTYPE_GOODCRC && PD_NUMOBJ_GET(&goodcrc) == 0 && PD_MESSAGEID_GET(&goodcrc) == _tx_messageidcounter) {
      /* Increment MessageIDCounter */
      _tx_messageidcounter = (_tx_messageidcounter + 1) % 8;

      notify(Notifications::TX_DONE);
      return postSendState;
    } else {
      notify(Notifications::TX_ERR);
      return postSendFailedState;
    }
  }
  notify(Notifications::TX_ERR);
  return postSendFailedState;
}
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_wait_send_done() {

  /* Waiting for response*/
  uint32_t evt = currentEvents;
  clearEvents();

  if ((uint32_t)evt & (uint32_t)Notifications::DISCARD) {
    // increment the counter
    _tx_messageidcounter = (_tx_messageidcounter + 1) % 8;
    notify(Notifications::TX_ERR);
    return postSendFailedState;
  }

  /* If the message was sent successfully */
  if ((uint32_t)evt & (uint32_t)Notifications::I_TXSENT) {

    clearEvents();
    if (rxMessageWaiting) {
      return pe_sink_wait_good_crc();
    } else {
      // No Good CRC has arrived, these should _normally_ come really fast, but users implementation may be lagging
      // Setup a callback for this state
      return waitForEvent(PEWaitingMessageGoodCRC, (uint32_t)Notifications::MSG_RX, 100);
    }
  }
  /* If the message failed to be sent */
  if ((uint32_t)evt & (uint32_t)Notifications::I_RETRYFAIL) {
    notify(Notifications::TX_ERR);
    return postSendFailedState;
  }

  /* Silence the compiler warning */
  notify(Notifications::TX_ERR);
  return postSendFailedState;
}