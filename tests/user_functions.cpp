#include "user_functions.hpp"
#include "CppUTest/TestHarness.h"
#include <cstring>
/* The current draw when the output is disabled */
#define DPM_MIN_CURRENT PD_MA2PDI(100)

bool pdbs_dpm_evaluate_capability(const pd_msg *capabilities, pd_msg *request) {

  /* Get the number of PDOs */
  uint8_t numobj = PD_NUMOBJ_GET(capabilities);

  /* Get whether or not the power supply is constrained */

  /* Make sure we have configuration */
  /* Look at the PDOs to see if one matches our desires */
  // Look against USB_PD_Desired_Levels to select in order of preference
  uint8_t bestIndex        = 0xFF;
  int     bestIndexVoltage = 0;
  int     bestIndexCurrent = 0;
  bool    bestIsPPS        = false;
  for (uint8_t i = 0; i < numobj; i++) {
    /* If we have a fixed PDO, its V equals our desired V, and its I is
     * at least our desired I */
    if ((capabilities->obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_FIXED) {
      // This is a fixed PDO entry
      // Evaluate if it can produve sufficient current based on the
      // tipResistance (ohms*10) V=I*R -> V/I => minimum resistance, if our tip
      // resistance is >= this then we can use this supply

      int voltage_mv     = PD_PDV2MV(PD_PDO_SRC_FIXED_VOLTAGE_GET(capabilities->obj[i])); // voltage in mV units
      int current_a_x100 = PD_PDO_SRC_FIXED_CURRENT_GET(capabilities->obj[i]);            // current in 10mA units
      if (voltage_mv > bestIndexVoltage || bestIndex == 0xFF) {
        // Higher voltage and valid, select this instead
        bestIndex        = i;
        bestIndexVoltage = voltage_mv;
        bestIndexCurrent = current_a_x100;
        bestIsPPS        = false;
      }
    } else if ((capabilities->obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_AUGMENTED && (capabilities->obj[i] & PD_APDO_TYPE) == PD_APDO_TYPE_PPS) {
      // If this is a PPS slot, calculate the max voltage in the PPS range that
      // can we be used and maintain
      uint16_t max_voltage = PD_PAV2MV(PD_APDO_PPS_MAX_VOLTAGE_GET(capabilities->obj[i]));
      // uint16_t min_voltage =
      // PD_PAV2MV(PD_APDO_PPS_MIN_VOLTAGE_GET(capabilities->obj[i]));
      uint16_t max_current = PD_PAI2CA(PD_APDO_PPS_CURRENT_GET(capabilities->obj[i])); // max current in 10mA units
      // Using the current and tip resistance, calculate the ideal max voltage
      // if this is range, then we will work with this voltage
      // if this is not in range; then max_voltage can be safely selected
      if (max_voltage > bestIndexVoltage || bestIndex == 0xFF) {
        bestIndex        = i;
        bestIndexVoltage = max_voltage;
        bestIndexCurrent = max_current;
        bestIsPPS        = true;
      }
    }
  }
  if (bestIndex != 0xFF) {
    std::cout << "Found desired capability at index " << (int)bestIndex << " for V " << bestIndexVoltage << " Current " << bestIndexCurrent << std::endl;
    /* We got what we wanted, so build a request for that */
    request->hdr = PD_MSGTYPE_REQUEST | PD_NUMOBJ(1);
    if (bestIsPPS) {
      request->obj[0] = PD_RDO_PROG_CURRENT_SET(PD_CA2PAI(bestIndexCurrent)) | PD_RDO_PROG_VOLTAGE_SET(PD_MV2PRV(bestIndexVoltage)) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(bestIndex + 1);
    } else {
      request->obj[0] = PD_RDO_FV_MAX_CURRENT_SET(bestIndexCurrent) | PD_RDO_FV_CURRENT_SET(bestIndexCurrent) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(bestIndex + 1);
    }
    // USB Data
    request->obj[0] |= PD_RDO_USB_COMMS;
  } else {
    /* Nothing matched (or no configuration), so get 5 V at low current */
    request->hdr    = PD_MSGTYPE_REQUEST | PD_NUMOBJ(1);
    request->obj[0] = PD_RDO_FV_MAX_CURRENT_SET(DPM_MIN_CURRENT) | PD_RDO_FV_CURRENT_SET(DPM_MIN_CURRENT) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(1);
    /* If the output is enabled and we got here, it must be a capability
     * mismatch. */
    if (false /*TODO: Check if you have already negotiated*/) {
      request->obj[0] |= PD_RDO_CAP_MISMATCH;
    }
    // USB Data
    request->obj[0] |= PD_RDO_USB_COMMS;
  }
  // Even if we didnt match, we return true as we would still like to handshake
  // on 5V at the minimum
  return true;
}

bool EPREvaluateCapabilityFunc(const epr_pd_msg *capabilities, pd_msg *request) { /* Nothing matched (or no configuration), so get 5 V at low current */
  request->hdr    = PD_MSGTYPE_EPR_REQUEST | PD_NUMOBJ(2);
  request->obj[1] = capabilities->obj[0]; // Copy PDO into slot 2
  request->obj[0] = PD_RDO_FV_MAX_CURRENT_SET(DPM_MIN_CURRENT) | PD_RDO_FV_CURRENT_SET(DPM_MIN_CURRENT) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(1);

  request->obj[0] |= PD_RDO_EPR_CAPABLE;
  // USB Data
  request->obj[0] |= PD_RDO_USB_COMMS;
  return true;
}
void pdbs_dpm_get_sink_capability(pd_msg *cap, const bool isPD3) {
  /* Keep track of how many PDOs we've added */
  int numobj = 0;

  // Must always have a PDO object for vSafe5V, indicate the bare minimum power required
  /* Minimum current, 5 V, and higher capability. */
  cap->obj[numobj++] = PD_PDO_TYPE_FIXED | PD_PDO_SNK_FIXED_VOLTAGE_SET(PD_MV2PDV(5000)) | PD_PDO_SNK_FIXED_CURRENT_SET(DPM_MIN_CURRENT);

  if (true) { // If requesting more than 5V
    /* Get the current we want */
    uint16_t voltage = 20 * 1000; // in mv => 20V
    uint16_t current = 2 * 100;   // In centi-amps => 2A

    /* Add a PDO for the desired power. */
    cap->obj[numobj++] = PD_PDO_TYPE_FIXED | PD_PDO_SNK_FIXED_VOLTAGE_SET(PD_MV2PDV(voltage)) | PD_PDO_SNK_FIXED_CURRENT_SET(current);

    /* If we want more than 5 V, set the Higher Capability flag */
    if (PD_MV2PDV(voltage) != PD_MV2PDV(5000)) {
      cap->obj[0] |= PD_PDO_SNK_FIXED_HIGHER_CAP;
    }
    /* If we're using PD 3.0, add a PPS APDO for our desired voltage */
    if (isPD3) {
      cap->obj[numobj++]
          = PD_PDO_TYPE_AUGMENTED | PD_APDO_TYPE_PPS | PD_APDO_PPS_MAX_VOLTAGE_SET(PD_MV2PAV(voltage)) | PD_APDO_PPS_MIN_VOLTAGE_SET(PD_MV2PAV(voltage)) | PD_APDO_PPS_CURRENT_SET(PD_CA2PAI(current));
    }
  }
  /* Set the USB communications capable flag. */
  cap->obj[0] |= PD_PDO_SNK_FIXED_USB_COMMS;
  // if this device is unconstrained, set the flag
  cap->obj[0] |= PD_PDO_SNK_FIXED_UNCONSTRAINED;

  /* Set the Sink_Capabilities message header */
  cap->hdr = PD_MSGTYPE_SINK_CAPABILITIES | PD_NUMOBJ(numobj);
}
