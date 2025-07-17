#pragma once

#include "shared_globals.h"

// Calculate checksum for config validation
byte calculateChecksum(Config* cfg);

// Save configuration to EEPROM
void saveConfig();

// Load configuration from EEPROM
bool loadConfig();

// Export configuration to JSON format
void exportConfigJSON();

// Import configuration from JSON (paste into Serial Monitor)
void importConfigJSON();
