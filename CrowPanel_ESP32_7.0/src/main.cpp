#include <PCA9557.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include "ui.h"
#include "WiFi.h"
#include <esp_now.h>
#include <esp_wifi.h>

// VESC sender's packet structures (matching the actual sender)
typedef struct {
    char board_name[16];                  // Board name (null-terminated string)
    float encoder_degrees;                // Current encoder position in degrees
    float crsf_target_degrees;            // CRSF target position in degrees
    float vesc_target_revolutions;        // VESC target position in revolutions
} __attribute__((packed)) telemetry_payload_t;

typedef struct {
    uint8_t type;                         // Broadcast or unicast ESPNOW data
    uint8_t state;                        // State indicator
    uint16_t seq_num;                     // Sequence number
    uint16_t crc;                         // CRC16 value
    uint32_t magic;                       // Magic number
    uint8_t payload[0];                   // Real payload (telemetry_payload_t follows)
} __attribute__((packed)) telemetry_espnow_data_t;

// Structure to store telemetry data (matches the sender's payload)
struct TelemetryData {
    char board_name[16];                  // Board name (null-terminated string)
    float encoder_degrees;                // Current encoder position in degrees
    float crsf_target_degrees;            // CRSF target position in degrees
    float vesc_target_revolutions;        // VESC target position in revolutions
    unsigned long lastUpdate;             // When this data was last received
};

// Map to store telemetry data by name (using simple array for embedded system)
#define MAX_TELEMETRY_SOURCES 10
struct TelemetryEntry {
    char name[16];
    TelemetryData data;
    bool active;
};

TelemetryEntry telemetryArray[MAX_TELEMETRY_SOURCES];
int telemetryCount = 0;

// Function to find or create telemetry entry by name
int findOrCreateTelemetryEntry(const char* name) {
    // First, try to find existing entry
    for (int i = 0; i < telemetryCount; i++) {
        if (strcmp(telemetryArray[i].name, name) == 0) {
            return i;
        }
    }
    
    // If not found and we have space, create new entry
    if (telemetryCount < MAX_TELEMETRY_SOURCES) {
        strncpy(telemetryArray[telemetryCount].name, name, sizeof(telemetryArray[telemetryCount].name) - 1);
        telemetryArray[telemetryCount].name[sizeof(telemetryArray[telemetryCount].name) - 1] = '\0';
        telemetryArray[telemetryCount].active = true;
        return telemetryCount++;
    }
    
    return -1; // No space available
}

// Add this raw ESP-NOW receive callback for debugging
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    Serial.println("\n>>> RAW ESP-NOW DATA RECEIVED <<<");
    Serial.printf("From MAC: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", mac[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.printf("\nData length: %d bytes\n", len);
    
    // Raw hex dump
    Serial.println("Raw hex data:");
    for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", incomingData[i]);
        if ((i + 1) % 16 == 0) Serial.println();
    }
    if (len % 16 != 0) Serial.println();
    
    // ASCII interpretation
    Serial.print("ASCII interpretation: '");
    for (int i = 0; i < len; i++) {
        char c = incomingData[i];
        if (c >= 32 && c <= 126) {
            Serial.print(c);
        } else {
            Serial.print('.');
        }
    }
    Serial.println("'");
    Serial.println(">>> END RAW DATA <<<\n");
    
    // Check if this is a VESC telemetry packet
    if (len >= sizeof(telemetry_espnow_data_t) + sizeof(telemetry_payload_t)) {
        telemetry_espnow_data_t* header = (telemetry_espnow_data_t*)incomingData;
        telemetry_payload_t* payload = (telemetry_payload_t*)(incomingData + sizeof(telemetry_espnow_data_t));
        
        Serial.println("=== VESC TELEMETRY PACKET ===");
        Serial.printf("Header - Type:%d, State:%d, Seq:%d, Magic:%08X\n", 
                     header->type, header->state, header->seq_num, header->magic);
        Serial.printf("Board: '%s'\n", payload->board_name);
        Serial.printf("Encoder: %.2f degrees\n", payload->encoder_degrees);
        Serial.printf("CRSF Target: %.2f degrees\n", payload->crsf_target_degrees);
        Serial.printf("VESC Target: %.3f revolutions\n", payload->vesc_target_revolutions);
        Serial.println("============================");
        
        // Find or create entry for this telemetry source
        int index = findOrCreateTelemetryEntry(payload->board_name);
        if (index >= 0) {
            // Copy the board name
            strncpy(telemetryArray[index].data.board_name, payload->board_name, sizeof(telemetryArray[index].data.board_name) - 1);
            telemetryArray[index].data.board_name[sizeof(telemetryArray[index].data.board_name) - 1] = '\0';
            
            // Store the float values directly from payload
            telemetryArray[index].data.encoder_degrees = payload->encoder_degrees;
            telemetryArray[index].data.crsf_target_degrees = payload->crsf_target_degrees;
            telemetryArray[index].data.vesc_target_revolutions = payload->vesc_target_revolutions;
            telemetryArray[index].data.lastUpdate = millis();
            Serial.printf("Stored data for '%s' at index %d\n", payload->board_name, index);
        } else {
            Serial.println("Warning: No space to store telemetry data");
        }
    } else {
        Serial.printf("Packet too small for VESC telemetry (need %d bytes, got %d)\n", 
                     sizeof(telemetry_espnow_data_t) + sizeof(telemetry_payload_t), len);
    }
}


class LGFX : public lgfx::LGFX_Device
{
public:

  lgfx::Bus_RGB     _bus_instance;
  lgfx::Panel_RGB   _panel_instance;

  LGFX(void)
  {


    {
      auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;
      
      cfg.pin_d0  = GPIO_NUM_15; // B0
      cfg.pin_d1  = GPIO_NUM_7;  // B1
      cfg.pin_d2  = GPIO_NUM_6;  // B2
      cfg.pin_d3  = GPIO_NUM_5;  // B3
      cfg.pin_d4  = GPIO_NUM_4;  // B4
      
      cfg.pin_d5  = GPIO_NUM_9;  // G0
      cfg.pin_d6  = GPIO_NUM_46; // G1
      cfg.pin_d7  = GPIO_NUM_3;  // G2
      cfg.pin_d8  = GPIO_NUM_8;  // G3
      cfg.pin_d9  = GPIO_NUM_16; // G4
      cfg.pin_d10 = GPIO_NUM_1;  // G5
      
      cfg.pin_d11 = GPIO_NUM_14; // R0
      cfg.pin_d12 = GPIO_NUM_21; // R1
      cfg.pin_d13 = GPIO_NUM_47; // R2
      cfg.pin_d14 = GPIO_NUM_48; // R3
      cfg.pin_d15 = GPIO_NUM_45; // R4

      cfg.pin_henable = GPIO_NUM_41;
      cfg.pin_vsync   = GPIO_NUM_40;
      cfg.pin_hsync   = GPIO_NUM_39;
      cfg.pin_pclk    = GPIO_NUM_0;
      cfg.freq_write  = 15000000;

      cfg.hsync_polarity    = 0;
      cfg.hsync_front_porch = 40;
      cfg.hsync_pulse_width = 48;
      cfg.hsync_back_porch  = 40;
      
      cfg.vsync_polarity    = 0;
      cfg.vsync_front_porch = 1;
      cfg.vsync_pulse_width = 31;
      cfg.vsync_back_porch  = 13;

      cfg.pclk_active_neg   = 1;
      cfg.de_idle_high      = 0;
      cfg.pclk_idle_high    = 0;

      _bus_instance.config(cfg);
    }
            {
      auto cfg = _panel_instance.config();
      cfg.memory_width  = 800;
      cfg.memory_height = 480;
      cfg.panel_width  = 800;
      cfg.panel_height = 480;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      _panel_instance.config(cfg);
    }
    _panel_instance.setBus(&_bus_instance);
    setPanel(&_panel_instance);

  }
};


LGFX lcd;
#include "touch.h"

//UI
#define TFT_BL 2
int led;
SPIClass& spi = SPI;

/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
//static lv_color_t *disp_draw_buf;
static lv_color_t disp_draw_buf[800 * 480 / 15];
static lv_disp_drv_t disp_drv;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{

  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  

  //lcd.fillScreen(TFT_WHITE);
#if (LV_COLOR_16_SWAP != 0)
 lcd.pushImageDMA(area->x1, area->y1, w, h,(lgfx::rgb565_t*)&color_p->full);
#else
  lcd.pushImageDMA(area->x1, area->y1, w, h,(lgfx::rgb565_t*)&color_p->full);//
#endif

  lv_disp_flush_ready(disp);

}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      Serial.print( "Data x " );
      Serial.println( data->point.x );
      Serial.print( "Data y " );
      Serial.println( data->point.y );
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
  delay(15);
}



//PCA9557 Out;
void setup()
{
    // Setup Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station (matching reference code)
  WiFi.mode(WIFI_STA);
  
  // Initialize WiFi interface for ESP-NOW
  esp_wifi_start();
  
  // Set WiFi channel to match VESC sender (Channel 1)
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  // Init ESP-NOW (matching reference code)
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW initialized successfully");
  
  // Register callback with proper type casting (matching reference code)
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  Serial.println("ESP-NOW receive callback registered");
 
  // Print MAC Address and WiFi channel to Serial monitor
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("WiFi Channel: ");
  Serial.println(WiFi.channel());
  Serial.print("WiFi Mode: ");
  Serial.println(WiFi.getMode());  

  pinMode(38, OUTPUT);
  digitalWrite(38, LOW);
  
  // Init Display
  lcd.begin();
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextSize(2);
  delay(200);
  

//    lcd.fillScreen(TFT_RED);
//    delay(1000);
//    lcd.fillScreen(TFT_GREEN);
//    delay(1000);
//    lcd.fillScreen(TFT_BLUE);
//    delay(1000);
//    lcd.fillScreen(TFT_BLACK);
//    delay(1000);
  lv_init();


  // Init touch device
  touch_init();

  screenWidth = lcd.width();
  screenHeight = lcd.height();

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 15); //4

  /* Initialize the display */
  lv_disp_drv_init(&disp_drv);
  /* Change the following line to your display resolution */
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /* Initialize the (dummy) input device driver */
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
#ifdef TFT_BL
 
  //digitalWrite(TFT_BL, HIGH);
  ledcSetup(1, 300, 8);
  ledcAttachPin(TFT_BL, 1);
  ledcWrite(1, 255); /* Screen brightness can be modified by adjusting this parameter. (0-255) */
#endif
 
  #ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW); 
  delay(500);
  digitalWrite(TFT_BL, HIGH);
  #endif

  
  ui_init();
  

  lv_timer_handler();

  Serial.println("ESP-NOW receiver ready - waiting for telemetry data...");
  Serial.printf("Listening on MAC: %s\n", WiFi.macAddress().c_str());

}

// Function to update UI labels for a specific sensor by name and number
void updateSensorDisplay(const char* sensorName, int sensorNumber) {
  char sensor_buffer[6];
  
  // Look for the specified sensor telemetry data
  int sensorIndex = -1;
  for (int i = 0; i < telemetryCount; i++) {
    if (strcmp(telemetryArray[i].name, sensorName) == 0) {
      sensorIndex = i;
      break;
    }
  }

  // Arrays of UI label pointers indexed by sensor number (0-based, so sensor 1 = index 0)
  lv_obj_t* posLabels[] = {ui_PosDeg1, ui_PosDeg2, ui_PosDeg3, ui_PosDeg4, ui_PosDeg5, ui_PosDeg6, ui_PosDeg7, ui_PosDeg8};
  lv_obj_t* targetLabels[] = {ui_TargetDeg1, ui_TargetDeg2, ui_TargetDeg3, ui_TargetDeg4, ui_TargetDeg5, ui_TargetDeg6, ui_TargetDeg7, ui_TargetDeg8};
  lv_obj_t* ppmLabels[] = {ui_PPMOUT1, ui_PPMOUT2, ui_PPMOUT3, ui_PPMOUT4, ui_PPMOUT5, ui_PPMOUT6, ui_PPMOUT7, ui_PPMOUT8};

  // Convert to 0-based index and check bounds
  int labelIndex = sensorNumber - 1;
  if (labelIndex < 0 || labelIndex >= 8) {
    return; // Invalid sensor number
  }
  
  // Get the labels for this sensor number
  lv_obj_t* posLabel = posLabels[labelIndex];
  lv_obj_t* targetLabel = targetLabels[labelIndex];
  lv_obj_t* ppmLabel = ppmLabels[labelIndex];

  // Only proceed if we have valid labels for this sensor number
  if (posLabel && targetLabel && ppmLabel) {
    // Check if sensor data exists and is recent (less than 10 seconds old)
    if (sensorIndex >= 0) {
      unsigned long dataAge = millis() - telemetryArray[sensorIndex].data.lastUpdate;
      
      if (dataAge < 10000) {  // Data is fresh (less than 10 seconds)
        snprintf(sensor_buffer, sizeof(sensor_buffer), "%.1f", telemetryArray[sensorIndex].data.encoder_degrees);
        lv_label_set_text(posLabel, sensor_buffer);
        snprintf(sensor_buffer, sizeof(sensor_buffer), "%.1f", telemetryArray[sensorIndex].data.crsf_target_degrees);
        lv_label_set_text(targetLabel, sensor_buffer);
        snprintf(sensor_buffer, sizeof(sensor_buffer), "%.2f", telemetryArray[sensorIndex].data.vesc_target_revolutions);
        lv_label_set_text(ppmLabel, sensor_buffer);
      } else {
        // Data is stale, show values with "OLD" indicator
        char stale_buffer[16];
        snprintf(stale_buffer, sizeof(stale_buffer), "%.1f OLD", telemetryArray[sensorIndex].data.encoder_degrees);
        lv_label_set_text(posLabel, stale_buffer);
        snprintf(stale_buffer, sizeof(stale_buffer), "%.1f OLD", telemetryArray[sensorIndex].data.crsf_target_degrees);
        lv_label_set_text(targetLabel, stale_buffer);
        snprintf(stale_buffer, sizeof(stale_buffer), "%.2f OLD", telemetryArray[sensorIndex].data.vesc_target_revolutions);
        lv_label_set_text(ppmLabel, stale_buffer);
      }
    } else {
      // No sensor data available, show default values
      lv_label_set_text(posLabel, "---");
      lv_label_set_text(targetLabel, "---");
      lv_label_set_text(ppmLabel, "---");
    }
  }
}

void loop()
{
  // Print telemetry status every 5 seconds
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 5000) {
    Serial.printf("=== Telemetry Status ===\n");
    Serial.printf("Current telemetry count: %d\n", telemetryCount);
    
    // Check for each expected sensor
    const char* expectedSensors[] = {"Left_Shoulder", "Left_Upper", "Left_Elbow", "Left_Claw", "Right_Shoulder", "Right_Upper", "Right_Elbow", "Right_Claw"};
    for (int j = 0; j < 8; j++) {
      bool found = false;
      unsigned long lastUpdate = 0;
      
      for (int i = 0; i < telemetryCount; i++) {
        if (strcmp(telemetryArray[i].name, expectedSensors[j]) == 0) {
          found = true;
          lastUpdate = millis() - telemetryArray[i].data.lastUpdate;
          Serial.printf("%s: Encoder: %.1f°, CRSF Target: %.1f°, VESC Target: %.2f rev - Last update: %lu ms ago - %s\n", 
                        expectedSensors[j],
                        telemetryArray[i].data.encoder_degrees,
                        telemetryArray[i].data.crsf_target_degrees,
                        telemetryArray[i].data.vesc_target_revolutions,
                        lastUpdate,
                        (lastUpdate < 10000) ? "FRESH" : "STALE");
          break;
        }
      }
      
      if (!found) {
        Serial.printf("%s: NOT FOUND\n", expectedSensors[j]);
      }
    }
    Serial.println("========================");
    lastDebugTime = millis();
  }

  // Update the UI with telemetry data for each sensor
  updateSensorDisplay("Left_Shoulder", 1);
  updateSensorDisplay("Left_Upper", 2);
  updateSensorDisplay("Left_Elbow", 3);
  updateSensorDisplay("Left_Claw", 4);
  updateSensorDisplay("Right_Shoulder", 5);
  updateSensorDisplay("Right_Upper", 6);
  updateSensorDisplay("Right_Elbow", 7);
  updateSensorDisplay("Right_Claw", 8);

  if(led == 1)
    digitalWrite(38, HIGH);
  if(led == 0)
    digitalWrite(38, LOW);
  lv_timer_handler(); /* let the GUI do its work */
  delay(10);
}
