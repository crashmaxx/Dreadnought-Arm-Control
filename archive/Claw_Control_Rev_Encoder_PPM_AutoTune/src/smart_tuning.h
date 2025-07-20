#ifndef SMART_TUNING_H
#define SMART_TUNING_H

#include <Arduino.h>

// Smart tuning state
extern bool smartTuningEnabled;
extern bool smartWaitingForArm;
extern unsigned long smartStartTime;

// Smart tuning functions
void startSmartTuning();
void stopSmartTuning();
void applySmartTuning();
void handleSmartTuning();

#endif // SMART_TUNING_H
