#pragma once
#include "fusb302_defines.h"
#include "fusb302b.h"
#include "mock_fusb302.h"
#include "policy_engine.h"
#include <stdint.h>
#include <stdio.h>
bool pdbs_dpm_evaluate_capability(const pd_msg *capabilities, pd_msg *request);
bool EPREvaluateCapabilityFunc(const epr_pd_msg *capabilities, pd_msg *request);
void pdbs_dpm_get_sink_capability(pd_msg *cap, const bool isPD3);