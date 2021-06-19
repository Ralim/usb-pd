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

class PolicyEngine {
public:
  PolicyEngine(FUSB302 fusbStruct);

  // Runs the internal thread. DOES NOT RETURN
  void thread();

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
    return true;
  }
  // Call this periodically, at least once every second or so
  void PPSTimerCallback();

  bool NegotiationTimeoutReached(uint8_t timeout);

  // Debugging allows looking at state
  uint32_t getState() { return (uint32_t)state; }

  bool IRQOccured();

private:
  // Push an incoming message to the Policy Engine
  void handleMessage(pd_msg *msg);
  void readPendingMessage();
  enum class Notifications {
    PDB_EVT_PE_RESET          = EVENT_MASK(0),
    PDB_EVT_PE_MSG_RX         = EVENT_MASK(1),
    PDB_EVT_PE_TX_DONE        = EVENT_MASK(2),
    PDB_EVT_PE_TX_ERR         = EVENT_MASK(3),
    PDB_EVT_PE_HARD_SENT      = EVENT_MASK(4),
    PDB_EVT_PE_I_OVRTEMP      = EVENT_MASK(5),
    PDB_EVT_PE_PPS_REQUEST    = EVENT_MASK(6),
    PDB_EVT_PE_GET_SOURCE_CAP = EVENT_MASK(7),
    PDB_EVT_PE_NEW_POWER      = EVENT_MASK(8),
    PDB_EVT_TX_I_TXSENT       = EVENT_MASK(9),
    PDB_EVT_TX_I_RETRYFAIL    = EVENT_MASK(10),
    PDB_EVT_TX_DISCARD        = EVENT_MASK(11),
    PDB_EVT_PE_ALL            = (EVENT_MASK(12) - 1),
  };
  // Send a notification
  void notify(Notifications notification);

  const FUSB302 fusb;
  bool          pdNegotiationComplete;
  int           current_voltage_mv;   // The current voltage PD is expecting
  int           _requested_voltage;   // The voltage the unit wanted to requests
  bool          _unconstrained_power; // If the source is unconstrained
                                      /* PD message header template */
  uint16_t hdr_template;
  /* Whether or not we have an explicit contract */
  bool _explicit_contract;
  /* The number of hard resets we've sent */
  int8_t _hard_reset_counter;
  /* The index of the first PPS APDO */
  uint8_t _pps_index;

  uint32_t pushMessage(pd_msg *msg);
  uint8_t  _tx_messageidcounter;
  typedef enum {
    PESinkStartup,              // 0
    PESinkDiscovery,            // 1
    PESinkWaitCap,              // 2
    PESinkEvalCap,              // 3
    PESinkSelectCap,            // 4
    PESinkTransitionSink,       // 5
    PESinkReady,                // 6
    PESinkGetSourceCap,         // 7
    PESinkGiveSinkCap,          // 8
    PESinkHardReset,            // 9
    PESinkTransitionDefault,    // 10
    PESinkSoftReset,            // 11
    PESinkSendSoftReset,        // 12
    PESinkSendNotSupported,     // 13
    PESinkChunkReceived,        // 14
    PESinkNotSupportedReceived, // 15
    PESinkSourceUnresponsive    // 16

  } policy_engine_state;
  policy_engine_state pe_sink_startup();
  policy_engine_state pe_sink_discovery();
  policy_engine_state pe_sink_wait_cap();
  policy_engine_state pe_sink_eval_cap();
  policy_engine_state pe_sink_select_cap();
  policy_engine_state pe_sink_transition_sink();
  policy_engine_state pe_sink_ready();
  policy_engine_state pe_sink_get_source_cap();
  policy_engine_state pe_sink_give_sink_cap();
  policy_engine_state pe_sink_hard_reset();
  policy_engine_state pe_sink_transition_default();
  policy_engine_state pe_sink_soft_reset();
  policy_engine_state pe_sink_send_soft_reset();
  policy_engine_state pe_sink_send_not_supported();
  policy_engine_state pe_sink_chunk_received();
  policy_engine_state pe_sink_not_supported_received();
  policy_engine_state pe_sink_source_unresponsive();
  // Event group
  uint32_t waitForEvent(uint32_t mask, uint32_t ticksToWait = 0xFFFFFFFF);
  // Temp messages for storage
  pd_msg              tempMessage       = {0};
  bool                rxMessageWaiting  = false;
  pd_msg              rxMessage         = {0}; // irq will unpack recieved message to here
  pd_msg              _last_dpm_request = {0};
  policy_engine_state state             = policy_engine_state::PESinkStartup;
  // Read a pending message into the temp message
  bool     readMessage();
  bool     PPSTimerEnabled;
  uint32_t PPSTimeLastEvent;
  int8_t   dpm_get_range_fixed_pdo_index(const pd_msg *caps);
  // These callbacks are called to implement the logic for the iron to select
  // the desired voltage

  /*
   * Create a Request message based on the given Source_Capabilities message. If
   * capabilities is NULL, the last non-null Source_Capabilities message passes
   * is used.  If none has been provided, the behavior is undefined.
   *
   * Returns true if sufficient power is available, false otherwise.
   */
  bool pdbs_dpm_evaluate_capability(const pd_msg *capabilities, pd_msg *request);

  /*
   * Create a Sink_Capabilities message for our current capabilities.
   */
  void pdbs_dpm_get_sink_capability(pd_msg *cap);

  /*
   * Indicate that power negotiations are starting.
   */
  void pdbs_dpm_pd_start();

  /*
   * Transition the sink to default power.
   */
  void pdbs_dpm_transition_default();

  /*
   * Transition to the requested minimum current.
   */
  void pdbs_dpm_transition_min();

  /*
   * Transition to Sink Standby if necessary.
   */
  void pdbs_dpm_transition_standby();

  /*
   * Transition to the requested power level
   */
  void pdbs_dpm_transition_requested();

  /*
   * Transition to the Type-C Current power level
   */
  void pdbs_dpm_transition_typec();
};

#endif /* PDB_POLICY_ENGINE_H */
