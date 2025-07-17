#include "config_manager.h"

// Calculate checksum for config validation
byte calculateChecksum(Config* cfg) {
  byte checksum = 0;
  byte* ptr = (byte*)cfg;
  for (int i = 0; i < sizeof(Config) - 1; i++) {
    checksum += ptr[i];
  }
  return checksum;
}

// Save configuration to EEPROM
void saveConfig() {
  config.version = CONFIG_VERSION;
  config.Kp = Kp;
  config.Ki = Ki;
  config.Kd = Kd;
  strncpy(config.board_name, board_name.c_str(), sizeof(config.board_name) - 1);
  config.board_name[sizeof(config.board_name) - 1] = '\0';
  config.channel_number = channel_number;
  config.min_angle = min_deg;
  config.max_angle = max_deg;
  config.checksum = calculateChecksum(&config);
  
  EEPROM.put(CONFIG_START, config);
  EEPROM.commit();
  Serial.println("Configuration saved to EEPROM");
}

// Load configuration from EEPROM
bool loadConfig() {
  EEPROM.get(CONFIG_START, config);
  
  // Validate config
  if (config.version != CONFIG_VERSION || 
      config.checksum != calculateChecksum(&config)) {
    Serial.println("Invalid or corrupted config, using defaults");
    return false;
  }
  
  // Load values
  Kp = config.Kp;
  Ki = config.Ki;
  Kd = config.Kd;
  board_name = String(config.board_name);
  channel_number = config.channel_number;
  
  Serial.println("Configuration loaded from EEPROM");
  Serial.printf("Kp: %.3f, Ki: %.3f, Kd: %.3f\n", Kp, Ki, Kd);
  return true;
}

// Export configuration to JSON format
void exportConfigJSON() {
  JsonDocument doc;
  
  doc["board_name"] = board_name;
  doc["firmware_version"] = "AutoTune_v1.0";
  doc["config_version"] = CONFIG_VERSION;
  doc["timestamp"] = millis();
  
  JsonObject pid = doc["pid"];
  pid["Kp"] = Kp;
  pid["Ki"] = Ki;
  pid["Kd"] = Kd;
  
  JsonObject settings = doc["settings"];
  settings["channel_number"] = channel_number;
  settings["min_angle"] = min_deg;
  settings["max_angle"] = max_deg;
  
  JsonObject hardware = doc["hardware"];
  hardware["encoder_ppr"] = ENCODER_PPR;
  hardware["pwm_min"] = 1;
  hardware["pwm_max"] = 1060;
  
  Serial.println("=== CONFIG EXPORT START ===");
  serializeJsonPretty(doc, Serial);
  Serial.println();
  Serial.println("=== CONFIG EXPORT END ===");
  Serial.println("Copy the JSON above to save your configuration");
}

// Import configuration from JSON (paste into Serial Monitor)
void importConfigJSON() {
  Serial.println("Paste your JSON configuration and press Enter:");
  Serial.println("(Waiting for input...)");
  
  // Wait for serial input
  String jsonInput = "";
  unsigned long startTime = millis();
  
  while (millis() - startTime < 30000) { // 30 second timeout
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (jsonInput.length() > 10) break; // Got some data
      } else {
        jsonInput += c;
      }
    }
  }
  
  if (jsonInput.length() < 10) {
    Serial.println("Import timeout or no data received");
    return;
  }
  
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, jsonInput);
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Import values
  if (doc.containsKey("pid")) {
    if (doc["pid"].containsKey("Kp")) Kp = doc["pid"]["Kp"];
    if (doc["pid"].containsKey("Ki")) Ki = doc["pid"]["Ki"];
    if (doc["pid"].containsKey("Kd")) Kd = doc["pid"]["Kd"];
  }
  
  if (doc.containsKey("settings")) {
    if (doc["settings"].containsKey("channel_number")) 
      channel_number = doc["settings"]["channel_number"];
  }
  
  if (doc.containsKey("board_name")) {
    board_name = doc["board_name"].as<String>();
  }
  
  // Update PID controller
  myPID.SetTunings(Kp, Ki, Kd);
  
  // Save to EEPROM
  saveConfig();
  
  Serial.println("Configuration imported successfully!");
  Serial.printf("New PID values - Kp: %.3f, Ki: %.3f, Kd: %.3f\n", Kp, Ki, Kd);
}
