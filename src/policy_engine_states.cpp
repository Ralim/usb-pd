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
#include <pd.h>
#include <stdbool.h>
#ifdef PD_DEBUG_OUTPUT
#include "stdio.h"
#endif
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_startup() {

  /* No need to reset the protocol layer here.  There are two ways into this
   * state: startup and exiting hard reset.  On startup, the protocol layer
   * is reset by the startup procedure.  When exiting hard reset, the
   * protocol layer is reset by the hard reset state machine.  Since it's
   * already done somewhere else, there's no need to do it again here. */

  return PESinkDiscovery;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_discovery() {

  /* Wait for VBUS.  Since it's our only power source, we already know that
   * we have it, so just move on.
   * If this was not true, we would want to wait and then re-run CC line selection
   * fusb.runCCLineSelection();
   */

  return PESinkSetupWaitCap;
}
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_setup_wait_cap() { //
  _explicit_contract = false;
  PPSTimerEnabled    = false;
  currentEvents      = 0;

  timestampNegotiationsStarted = getTimeStamp();
  return waitForEvent(policy_engine_state::PESinkWaitCap, (uint32_t)Notifications::MSG_RX | (uint32_t)Notifications::I_OVRTEMP | (uint32_t)Notifications::RESET,
                      // Wait for cap timeout
                      PD_T_TYPEC_SINK_WAIT_CAP);
}
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_wait_cap() {
  /* Fetch a message from the protocol layer */
  uint32_t evt = currentEvents;
  clearEvents(evt);
#ifdef PD_DEBUG_OUTPUT
  printf("Wait Cap Event %04X\r\n", (int)evt);
#endif

  /* If we're too hot, we shouldn't negotiate power yet */
  if (evt & (uint32_t)Notifications::I_OVRTEMP) {
    return PESinkSetupWaitCap;
  }

  /* If we got a message */
  /* Get the message */
  while (incomingMessages.getOccupied()) {
    incomingMessages.pop(&tempMessage);
    /* If we got a Source_Capabilities message, read it. */
    if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_SOURCE_CAPABILITIES && PD_NUMOBJ_GET(&tempMessage) > 0) {
#ifdef PD_DEBUG_OUTPUT
      printf("Source Capabilities message RX\r\n");
#endif

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

  /* If we failed to get a message, wait longer */
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
  sourceIsEPRCapable   = tempMessage.obj[0] & PD_PDO_SRC_FIXED_EPR_CAPABLE;

  /* Ask the DPM what to request */
  if (pdbs_dpm_evaluate_capability(&tempMessage, &_last_dpm_request)) {
    _last_dpm_request.hdr |= hdr_template;
    /* If we're using PD 3.0 */
    if ((hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0) {
      /* If the request was for a PPS, start time callbacks if not started
       */
      auto pdoPos = PD_RDO_OBJPOS_GET(&_last_dpm_request);
      if (pdoPos <= 7 && pdoPos >= _pps_index) {
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
  // clearEvents(0xFFFFFF); // clear all pending incase of an rx while prepping

#ifdef PD_DEBUG_OUTPUT
  printf("Sending desired capability\r\n");
#endif
  return pe_start_message_tx(policy_engine_state::PESinkSelectCap, PESinkHardReset, &_last_dpm_request);
}
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_select_cap() {
  // Have transmitted the selected cap, transition to waiting for the response
  clearEvents(0xFFFFFF);
  // wait for a response
  return waitForEvent(PESinkWaitCapResp, (uint32_t)Notifications::MSG_RX | (uint32_t)Notifications::RESET | (uint32_t)Notifications::TIMEOUT, PD_T_SENDER_RESPONSE);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_wait_cap_resp() {
  /* Wait for a response */
  clearEvents(0xFFFFFF);

  /* Get the response message */
  while (incomingMessages.getOccupied()) {
    incomingMessages.pop(&tempMessage);
    /* If the source accepted our request, wait for the new power message*/
    if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_ACCEPT) {

      is_epr = (PD_NUMOBJ_GET(&_last_dpm_request) == 2);
      if (is_epr) {
        EPRTimeLastEvent = getTimeStamp();
      }
      return waitForEvent(PESinkTransitionSink, (uint32_t)Notifications::MSG_RX | (uint32_t)Notifications::RESET, PD_T_PS_TRANSITION);
      /* If the message was a Soft_Reset, do the soft reset procedure */
    } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_SOFT_RESET) {
      return PESinkHandleSoftReset;
      /* If the message was Wait or Reject */
    } else if ((PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_REJECT || PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_WAIT)) {
#ifdef PD_DEBUG_OUTPUT
      printf("Requested Capabilities Rejected\r\n");
#endif
      /* If we don't have an explicit contract, wait for capabilities */
      if (!_explicit_contract) {
        return PESinkSetupWaitCap;
        /* If we do have an explicit contract, go to the ready state */
      } else {
        return waitForEvent(PESinkReady, (uint32_t)Notifications::ALL, 0xFFFFFFFF);
      }
    }
  }
  return waitForEvent(PESinkWaitCapResp, (uint32_t)Notifications::MSG_RX | (uint32_t)Notifications::RESET | (uint32_t)Notifications::TIMEOUT, PD_T_SENDER_RESPONSE);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_transition_sink() {
  /* Wait for the PS_RDY message */
  clearEvents(0xFFFFFF);
  /* If we received a message, read it */
  while (incomingMessages.getOccupied()) {

    incomingMessages.pop(&tempMessage);

    /* If we got a PS_RDY, handle it */
    if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_PS_RDY) {
      /* We just finished negotiating an explicit contract */
      /* Negotiation finished */
      negotiationOfEPRInProgress = false;
      if (sourceIsEPRCapable && (device_epr_wattage > 0) && !is_epr) {
        // We have entered into an SPR contract, but we support EPR and the supply does too
        //  Request entering EPR mode
        negotiationOfEPRInProgress = true;
        PolicyEngine::notify(Notifications::REQUEST_EPR);
      }
      _explicit_contract = true;

      return PESinkReady;
    } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_SOURCE_CAPABILITIES) {
      return PESinkEvalCap;
    }
  }
  // Timeout
  return PESinkSendSoftReset;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_ready() {
  uint32_t evt = currentEvents;
  clearEvents(evt);
  /* If SinkPPSPeriodicTimer ran out, send a new request */
  if (evt & (uint32_t)Notifications::PPS_REQUEST) {
    return PESinkSelectCapTx;
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
    return PESinkEvalCap;
  }

  if (evt & (uint32_t)Notifications::REQUEST_EPR) {
    return PESinkRequestEPR;
  }

  if (evt & (uint32_t)Notifications::EPR_KEEPALIVE) {
    return PESinkSendEPRKeepAlive;
  }

  /* If we received a message */
  if (evt & (uint32_t)Notifications::MSG_RX) {
    while (incomingMessages.getOccupied()) {

      incomingMessages.pop(&tempMessage);

      if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_VENDOR_DEFINED && PD_NUMOBJ_GET(&tempMessage) > 0) {
        // return waitForEvent(PESinkReady, (uint32_t)Notifications::ALL);
        /* Ignore Ping messages */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_PING && PD_NUMOBJ_GET(&tempMessage) == 0) {
        // return waitForEvent(PESinkReady, (uint32_t)Notifications::ALL);
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
        return PESinkHandleSoftReset;
        /* PD 3.0 messges */
      } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_EPR_MODE && PD_NUMOBJ_GET(&tempMessage) > 0) {
        if (tempMessage.bytes[0] == 3) {
          is_epr = true;
          // return PESinkReady;
          // We start off from here, but let the message read loop run until all are read
        } else if (tempMessage.bytes[0] == 4) {
          is_epr = false;
          return PESinkReady;
          // We attempted to enter EPR and failed, no need to renegotiate
        } else if (tempMessage.bytes[0] == 5) {
          is_epr = false;
          return PESinkWaitCap; // We exited EPR so now need to renegotiate an SPR contract
        }
      } else if ((hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0) {
        /* If the message is a multi-chunk extended message */
        if ((tempMessage.hdr & PD_HDR_EXT) && (PD_DATA_SIZE_GET(&tempMessage) >= PD_MAX_EXT_MSG_LEGACY_LEN)) {

          if ((PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_EPR_SOURCE_CAPABILITIES)) {

            return PESinkHandleEPRChunk;
          } else {
            // We can support _some_ chunked messages but not all
            return PESinkSendNotSupported;
          }
          /* Tell the DPM a message we sent got a response of Not_Supported. */
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

#ifdef PD_DEBUG_OUTPUT
  printf("Sending hard reset\r\n");
#endif
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

  /* There is no local hardware to reset. */
  /* Since we never change our data role from UFP, there is no reason to set
   * it here. */

  return PESinkStartup;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_soft_reset() {
  // Soft reset message is received
  /* No need to explicitly reset the protocol layer here.  It resets itself
   * when a Soft_Reset message is received. */

  /* Get a message object */
  pd_msg accept;
  /* Make an soft reset message */
  accept.hdr = hdr_template | PD_MSGTYPE_SOFT_RESET | PD_NUMOBJ(0);
  /* Transmit the Accept */
  return pe_start_message_tx(PESinkSetupWaitCap, PESinkHardReset, &accept);
}
PolicyEngine::policy_engine_state PolicyEngine::pe_sink_send_soft_reset() {
  /* No need to explicitly reset the protocol layer here.  It resets itself
   * just before a Soft_Reset message is transmitted. */

#ifdef PD_DEBUG_OUTPUT
  printf("Sending soft reset\r\n");
#endif
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
  clearEvents(0xFFFFFF);

  /* Get the response message */
  if (incomingMessages.getOccupied()) {

    incomingMessages.pop(&tempMessage);

    /* If the source accepted our soft reset, wait for capabilities. */
    if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_ACCEPT && PD_NUMOBJ_GET(&tempMessage) == 0) {

      return PESinkSetupWaitCap;
      /* If the message was a Soft_Reset, do the soft reset procedure */
    } else if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_SOFT_RESET && PD_NUMOBJ_GET(&tempMessage) == 0) {
      return PESinkHandleSoftReset;
      /* Otherwise, send a hard reset */
    } else {
      return PESinkHardReset;
    }
  }
  return PESinkHardReset;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_send_not_supported() {
  /* Get a message object */

#ifdef PD_DEBUG_OUTPUT
  printf("Sending not supported\r\n");
#endif
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

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_wait_epr_chunk() {
  uint32_t evt = currentEvents;
  clearEvents(evt);
  /* If we received a message */
  if (evt & (uint32_t)Notifications::MSG_RX) {
    while (incomingMessages.getOccupied()) {
      incomingMessages.pop(&tempMessage);

      if ((hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0) {
        /* If the message is a multi-chunk extended message */
        if ((tempMessage.hdr & PD_HDR_EXT) && (PD_DATA_SIZE_GET(&tempMessage) >= PD_MAX_EXT_MSG_LEGACY_LEN)) {
          if ((PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_EPR_SOURCE_CAPABILITIES)) {
            return PESinkHandleEPRChunk;
          } else {
            // We can support _some_ chunked messages but not all
            return PESinkSendNotSupported;
          }
          /* Tell the DPM a message we sent got a response of Not_Supported. */
        }
      }
    }
  }

  return waitForEvent(PESinkWaitForHandleEPRChunk, (uint32_t)Notifications::ALL, 0xFFFFFFFF);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_handle_epr_chunk() {
  if (tempMessage.exthdr & PD_EXTHDR_REQUEST_CHUNK) {
    return waitForEvent(PESinkWaitForHandleEPRChunk, (uint32_t)Notifications::ALL, 0xFFFFFFFF);
  }
  uint8_t chunk_index = PD_CHUNK_NUMBER_GET(&tempMessage);

  if (chunk_index == 0) {
    // Copy first message directly over the object to set header,ext-header + start of PDO's
    memcpy(&this->recent_epr_capabilities, &tempMessage.bytes, sizeof(tempMessage.bytes));
  } else {
    memcpy(&(this->recent_epr_capabilities.data[chunk_index * PD_MAX_EXT_MSG_CHUNK_LEN]), &(tempMessage.data), 2 + (4 * (PD_NUMOBJ_GET(&tempMessage) - 1)));
  }
  const auto recievedLength = (PD_MAX_EXT_MSG_CHUNK_LEN * chunk_index) /*Bytes Implicit by chunk index*/ + 2 /*half PDO*/ + (4 * (PD_NUMOBJ_GET(&tempMessage) - 1) /* Data in this message*/);

  if ((recievedLength) >= PD_DATA_SIZE_GET(&this->recent_epr_capabilities)) {
    return PESinkEPREvalCap;
  }
  memset(tempMessage.data,0,sizeof(tempMessage.data));
  tempMessage.hdr    = this->hdr_template | (tempMessage.hdr & PD_HDR_MSGTYPE) | PD_NUMOBJ(1) | PD_HDR_EXT;
  tempMessage.exthdr = ((chunk_index + 1) << PD_EXTHDR_CHUNK_NUMBER_SHIFT) | PD_EXTHDR_REQUEST_CHUNK | PD_EXTHDR_CHUNKED;
  return pe_start_message_tx(PESinkWaitForHandleEPRChunk, PESinkHardReset, &tempMessage);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_not_supported_received() {
  /* Inform the Device Policy Manager that we received a Not_Supported
   * message. */

  return waitForEvent(PESinkReady, (uint32_t)Notifications::ALL, 0xFFFFFFFF);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_source_unresponsive() {
  // Sit and chill, as PD is not working
  _explicit_contract = false;
  osDelay(PD_T_PD_DEBOUNCE);

  return PESinkSourceUnresponsive;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_wait_event() {
  // Check timeout
  if (getTimeStamp() > waitingEventsTimeout) {
    notify(Notifications::TIMEOUT);
  }
  if (currentEvents & (uint32_t)Notifications::TIMEOUT) {
    clearEvents(0xFFFFFF);
    if (postNotificationEvalState >= PESinkHandleSoftReset && postNotificationEvalState <= PESinkSendSoftResetResp) {
      // Timeout in soft reset, so reset state machine
      return PESinkStartup;
    }
    return PESinkSendSoftReset;
  }
  if (currentEvents & (uint32_t)Notifications::RESET) {
    return PESinkTransitionDefault;
  }

  if (currentEvents & waitingEventsMask) {
    return postNotificationEvalState;
  }
  return policy_engine_state::PEWaitingEvent;
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_wait_good_crc() {
  clearEvents(0xFFFFFF);

  while (incomingMessages.getOccupied()) {
    // Wait for the Good CRC
    pd_msg goodcrc;
    /* Read the GoodCRC */
    incomingMessages.pop(&goodcrc);
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
  clearEvents(evt);

  /* If the message was sent successfully */
  if ((uint32_t)evt & (uint32_t)Notifications::I_TXSENT) {

    if (incomingMessages.getOccupied()) {
      return pe_sink_wait_good_crc();
    } else {
      // No Good CRC has arrived, these should _normally_ come really fast (100us), but users implementation may be lagging
      // Setup a callback for this state
      return waitForEvent(PEWaitingMessageGoodCRC, (uint32_t)Notifications::MSG_RX, 120);
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

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_epr_eval_cap() {
  EPRTimeLastEvent = getTimeStamp();
  if (pdbs_dpm_epr_evaluate_capability(&recent_epr_capabilities, &_last_dpm_request)) {
    auto pps_index  = PD_RDO_OBJPOS_GET(&_last_dpm_request);
    PPSTimerEnabled = (recent_epr_capabilities.obj[pps_index - 1] & PD_PDO_TYPE) == PD_PDO_TYPE_AUGMENTED && (recent_epr_capabilities.obj[pps_index - 1] & PD_APDO_TYPE) == PD_APDO_TYPE_PPS;
    _last_dpm_request.hdr |= hdr_template;
    return PESinkSelectCapTx;
  } else {
    return PESinkWaitCap;
  }
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_request_epr() {
  EPRTimeLastEvent = getTimeStamp();
  pd_msg *epr_mode = &tempMessage;
  epr_mode->hdr    = this->hdr_template | PD_MSGTYPE_EPR_MODE | PD_NUMOBJ(1);
  epr_mode->obj[0] = (0x01 << PD_EPR_MODE_ACTION_SHIFT) | (device_epr_wattage << PD_EPR_MODE_DATA_SHIFT);
  return pe_start_message_tx(PESinkReady, PESinkHardReset, epr_mode);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_send_epr_keep_alive() {
  while (incomingMessages.getOccupied()) {
    incomingMessages.pop(nullptr);
  }
  negotiationOfEPRInProgress = true;
  tempMessage.hdr            = PD_HDR_EXT | this->hdr_template | PD_NUMOBJ(1) | PD_MSGTYPE_EXTENDED_CONTROL;
  tempMessage.exthdr         = (PD_EXTHDR_DATA_SIZE & 2) << PD_EXTHDR_DATA_SIZE_SHIFT | PD_EXTHDR_CHUNKED;
  tempMessage.data[0]        = PD_EXTENDED_CONTROL_TYPE_EPR_KEEPALIVE;
  tempMessage.data[1]        = PD_EXTENDED_CONTROL_DATA_UNUSED;
  return pe_start_message_tx(PESinkWaitEPRKeepAliveAck, PESinkReady, &tempMessage);
}

PolicyEngine::policy_engine_state PolicyEngine::pe_sink_wait_epr_keep_alive_ack() {
  // We want to wait for an ACK for the epr message
  while (incomingMessages.getOccupied()) {
    incomingMessages.pop(&tempMessage);
    if (PD_MSGTYPE_GET(&tempMessage) == PD_MSGTYPE_EXTENDED_CONTROL && tempMessage.data[0] == PD_EXTENDED_CONTROL_TYPE_EPR_KEEPALIVE_ACK) {
      negotiationOfEPRInProgress = false;
      EPRTimeLastEvent           = getTimeStamp();
      return PESinkReady;
    }
  }
  return PESinkSendEPRKeepAlive;
}
