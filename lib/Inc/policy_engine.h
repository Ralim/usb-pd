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

#ifndef PDB_POLICY_ENGINE_H
#define PDB_POLICY_ENGINE_H
#include "fusb302b.h"
#include "pd.h"
#include "ringbuffer.h"
#include <cstring>
#include <stdint.h>
class PolicyEngine {
public:
  // Functions required to be created by the user for their end application
  /*
   * Create a Request message based on the given Source_Capabilities message. If
   * capabilities is NULL, the last non-null Source_Capabilities message passes
   * is used.  If none has been provided, the behavior is undefined.
   *
   * Returns true if sufficient power is available, false otherwise.
   */
  typedef bool (*EvaluateCapabilityFunc)(const pd_msg *capabilities, pd_msg *request);

  /*
   * Create a Sink_Capabilities message for our current capabilities.
   */
  typedef void (*SinkCapabilityFunc)(pd_msg *cap, const bool isPD3);
  typedef uint32_t (*TimestampFunc)();
  typedef void (*DelayFunc)(uint32_t milliseconds);
  PolicyEngine(FUSB302 fusbStruct, TimestampFunc getTimestampF, DelayFunc delayFuncF, SinkCapabilityFunc sinkCapabilities, EvaluateCapabilityFunc evalFunc)
      : fusb(fusbStruct),                               //
        getTimeStamp(getTimestampF),                    //
        pdbs_dpm_get_sink_capability(sinkCapabilities), //
        pdbs_dpm_evaluate_capability(evalFunc),         //
        osDelay(delayFuncF)                             //
  {
    hdr_template = PD_DATAROLE_UFP | PD_POWERROLE_SINK;
    _pps_index   = 0xFF;
  };
  // Runs the internal thread, returns true if should re-run again immediately if possible
  bool thread();

  // Returns true if headers indicate PD3.0 compliant
  bool isPD3_0();
  bool setupCompleteOrTimedOut(uint8_t timeout) {
    if (pdNegotiationComplete) {
      return true;
    }
    if (PolicyEngine::NegotiationTimeoutReached(timeout)) {
      return true;
    }
    if (state == policy_engine_state::PESinkSourceUnresponsive) {
      return true;
    }
    if (state == policy_engine_state::PESinkReady) {
      return true;
    }
    return false;
  }
  // Has pd negotiation completed
  bool pdHasNegotiated() {
    if (state == policy_engine_state::PESinkSourceUnresponsive)
      return false;
    return state >= PESinkReady;
  }
  // Call this periodically, by the spec at least once every 10 seconds. <5 is reccomended
  void PPSTimerCallback();

  bool NegotiationTimeoutReached(uint8_t timeout);

  bool IRQOccured();
  void printStateName();
  // Useful for debug reading out
  int currentStateCode() { return (int)state; }

private:
  const FUSB302                fusb;
  const TimestampFunc          getTimeStamp;
  const SinkCapabilityFunc     pdbs_dpm_get_sink_capability;
  const EvaluateCapabilityFunc pdbs_dpm_evaluate_capability;
  const DelayFunc              osDelay;
  bool                         pdNegotiationComplete;
  int                          current_voltage_mv;   // The current voltage PD is expecting
  int                          _requested_voltage;   // The voltage the unit wanted to requests
  bool                         _unconstrained_power; // If the source is unconstrained
  uint8_t                      _tx_messageidcounter; // Counter for messages sent to be packed into messages sent
  uint16_t                     hdr_template;         /* PD message header template */

  /* Whether or not we have an explicit contract */
  bool _explicit_contract;
  /* The number of hard resets we've sent */
  int8_t _hard_reset_counter;
  /* The index of the first PPS APDO */
  uint8_t _pps_index;

  void readPendingMessage(); // Irq read message pending from the FiFo

  typedef enum {
    PEWaitingEvent             = 0,  // Meta state: waiting for event or timeout
    PEWaitingMessageTx         = 1,  // Meta state: waiting for message tx to confirm
    PEWaitingMessageGoodCRC    = 2,  // We have sent a message, waiting for a GoodCRC to come back
    PESinkStartup              = 3,  // Start of state machine
    PESinkDiscovery            = 4,  // no-op as source yells its features
    PESinkSetupWaitCap         = 5,  // Setup events wanted by waitCap
    PESinkWaitCap              = 6,  // Waiting for source
    PESinkEvalCap              = 7,  // Evaluating the source provided capabilities message
    PESinkSelectCapTx          = 8,  // Send cap selected
    PESinkSelectCap            = 9,  // Wait send ok
    PESinkWaitCapResp          = 10, // Wait response message
    PESinkTransitionSink       = 11, // Transition to sink mode
    PESinkReady                = 12, // Normal operational state, all is good
    PESinkGetSourceCap         = 13, // Request source capabilities
    PESinkGiveSinkCap          = 14, // Device has been requested for its capabilities
    PESinkHardReset            = 15, // Send a hard reset
    PESinkTransitionDefault    = 16, // Transition to reset
    PESinkHandleSoftReset            = 17, // Soft reset received
    PESinkSendSoftReset        = 18, // Send soft reset (comms resync)
    PESinkSendSoftResetTxOK    = 19, // Sending soft reset, waiting message tx
    PESinkSendSoftResetResp    = 20, // Soft reset waiting for response
    PESinkSendNotSupported     = 21, // Send a NACK message
    PESinkChunkReceived        = 22, // On chunked (larger) message recieved
    PESinkNotSupportedReceived = 23, // One of our messages was not supported
    PESinkSourceUnresponsive   = 24, // A resting state for a source that doesnt talk (aka no PD)
  } policy_engine_state;
  enum class Notifications {
    RESET          = EVENT_MASK(0),//1
    MSG_RX         = EVENT_MASK(1),//2
    TX_DONE        = EVENT_MASK(2),//4
    TX_ERR         = EVENT_MASK(3),//8
    HARD_SENT      = EVENT_MASK(4),//10
    I_OVRTEMP      = EVENT_MASK(5),//20
    PPS_REQUEST    = EVENT_MASK(6),//40
    GET_SOURCE_CAP = EVENT_MASK(7),//80
    NEW_POWER      = EVENT_MASK(8),//100
    I_TXSENT       = EVENT_MASK(9),//200
    I_RETRYFAIL    = EVENT_MASK(10),//400
    DISCARD        = EVENT_MASK(11),//800
    TIMEOUT        = EVENT_MASK(12),//1000 Internal notification for timeout waiting for an event
    ALL            = (EVENT_MASK(13) - 1),
  };
  // Send a notification
  void                notify(Notifications notification);
  policy_engine_state postNotifcationEvalState;
  policy_engine_state postSendState;
  policy_engine_state postSendFailedState;
  uint32_t            waitingEventsMask    = 0;
  uint32_t            waitingEventsTimeout = 0;
  uint32_t            currentEvents;
  void                clearEvents(uint32_t notification = 0xFFFFFF);
  policy_engine_state waitForEvent(policy_engine_state evalState, uint32_t notification, uint32_t timeout = 0xFFFFFFFF);

  policy_engine_state pe_sink_startup();
  policy_engine_state pe_sink_discovery();
  policy_engine_state pe_sink_setup_wait_cap();
  policy_engine_state pe_sink_wait_cap();
  policy_engine_state pe_sink_eval_cap();
  policy_engine_state pe_sink_select_cap();
  policy_engine_state pe_sink_select_cap_tx();
  policy_engine_state pe_sink_wait_cap_resp();
  policy_engine_state pe_sink_transition_sink();
  policy_engine_state pe_sink_ready();
  policy_engine_state pe_sink_get_source_cap();
  policy_engine_state pe_sink_give_sink_cap();
  policy_engine_state pe_sink_hard_reset();
  policy_engine_state pe_sink_transition_default();
  policy_engine_state pe_sink_soft_reset();
  policy_engine_state pe_sink_send_soft_reset_resp();
  policy_engine_state pe_sink_send_soft_reset_tx_ok();
  policy_engine_state pe_sink_send_soft_reset();
  policy_engine_state pe_sink_send_not_supported();
  policy_engine_state pe_sink_chunk_received();
  policy_engine_state pe_sink_not_supported_received();
  policy_engine_state pe_sink_source_unresponsive();
  policy_engine_state pe_sink_wait_event();
  policy_engine_state pe_sink_wait_send_done();
  policy_engine_state pe_sink_wait_good_crc();
  // Sending messages, starts send and returns next state
  policy_engine_state pe_start_message_tx(policy_engine_state postTxState, policy_engine_state txFailState, pd_msg *msg);

  // Event group
  // Temp messages for storage
  pd_msg                tempMessage = {0};
  ringbuffer<pd_msg, 4> incomingMessages;
  pd_msg                irqMessage        = {0}; // irq will unpack recieved message to here
  pd_msg                _last_dpm_request = {0};
  policy_engine_state   state             = policy_engine_state::PESinkStartup;
  // Read a pending message into the temp message
  bool     PPSTimerEnabled;
  uint32_t PPSTimeLastEvent;
};

#endif /* PDB_POLICY_ENGINE_H */
