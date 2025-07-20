#ifndef MANUAL_TUNING_H
#define MANUAL_TUNING_H

#include <Arduino.h>

// Manual tuning state
extern bool manualTuningEnabled;

// Manual tuning functions
void startManualTuning();
void stopManualTuning();
void applyManualTuning();
void runStepTest();
void adjustKp(bool increase);
void adjustKi(bool increase);
void adjustKd(bool increase);
void printTuningStatus();

#endif // MANUAL_TUNING_H
