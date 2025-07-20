#pragma once

#include "shared_globals.h"

// Start PID AutoTune
void startAutoTune();

// Stop AutoTune and apply results
void stopAutoTune();

// Handle AutoTune process (call this in main loop)
void handleAutoTune();
