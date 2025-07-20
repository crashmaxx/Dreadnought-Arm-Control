#pragma once

#include "shared_globals.h"

// Handle serial commands (call this in main loop)
void handleSerialCommands();

// Print debug information
void printDebugInfo(int ppm_out);

// Print all CRSF channel values
void printChannels();

// Show available commands
void showAvailableCommands();
