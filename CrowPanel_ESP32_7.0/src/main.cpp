#include <PCA9557.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include "ui.h"
#include "esp_now_telemetry.h"
#include "WiFi.h"
#include <esp_now.h>

// Structure to store telemetry data
struct TelemetryData {
    int value1;
    int value2;
    int value3;
    unsigned long lastUpdate;
};

// Map to store telemetry data by name (using simple array for embedded system)
#define MAX_TELEMETRY_SOURCES 10
struct TelemetryEntry {
    char name[32];
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

// Callback function to handle received telemetry packets
void onTelemetryReceived(const TelemetryPacket* packet, const uint8_t* mac, int len) {
    Serial.print("Received telemetry from: ");
    Serial.println(packet->name);
    Serial.print("Value1: "); Serial.println(packet->value1);
    Serial.print("Value2: "); Serial.println(packet->value2);
    Serial.print("Value3: "); Serial.println(packet->value3);
    Serial.print("Packet length: "); Serial.println(len);
    Serial.print("MAC: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", mac[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    
    // Find or create entry for this telemetry source
    int index = findOrCreateTelemetryEntry(packet->name);
    if (index >= 0) {
        telemetryArray[index].data.value1 = packet->value1;
        telemetryArray[index].data.value2 = packet->value2;
        telemetryArray[index].data.value3 = packet->value3;
        telemetryArray[index].data.lastUpdate = millis();
        Serial.printf("Stored data for %s at index %d\n", packet->name, index);
    } else {
        Serial.println("Warning: No space to store telemetry data");
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
  
  // Put ESP32 into Station mode
  WiFi.mode(WIFI_MODE_STA);
  
  // Initialize ESP-NOW with error checking
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW initialized successfully");
 
  // Print MAC Address to Serial monitor
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());  

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

  // Register the telemetry receive callback
  getTelemetry(onTelemetryReceived);  

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
  lv_obj_t* posLabels[] = {ui_PosDeg1, ui_PosDeg2, ui_PosDeg3, ui_PosDeg4};
  lv_obj_t* targetLabels[] = {ui_TargetDeg1, ui_TargetDeg2, ui_TargetDeg3, ui_TargetDeg4};
  lv_obj_t* ppmLabels[] = {ui_PPMOUT1, ui_PPMOUT2, ui_PPMOUT3, ui_PPMOUT4};
  
  // Convert to 0-based index and check bounds
  int labelIndex = sensorNumber - 1;
  if (labelIndex < 0 || labelIndex >= 4) {
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
        snprintf(sensor_buffer, sizeof(sensor_buffer), "%d", telemetryArray[sensorIndex].data.value1);
        lv_label_set_text(posLabel, sensor_buffer);
        snprintf(sensor_buffer, sizeof(sensor_buffer), "%d", telemetryArray[sensorIndex].data.value2);
        lv_label_set_text(targetLabel, sensor_buffer);
        snprintf(sensor_buffer, sizeof(sensor_buffer), "%d", telemetryArray[sensorIndex].data.value3);
        lv_label_set_text(ppmLabel, sensor_buffer);
      } else {
        // Data is stale, show values with "Timeout" indicator
        char stale_buffer[16];
        snprintf(stale_buffer, sizeof(stale_buffer), "%d OLD", telemetryArray[sensorIndex].data.value1);
        lv_label_set_text(posLabel, stale_buffer);
        snprintf(stale_buffer, sizeof(stale_buffer), "%d OLD", telemetryArray[sensorIndex].data.value2);
        lv_label_set_text(targetLabel, stale_buffer);
        snprintf(stale_buffer, sizeof(stale_buffer), "%d OLD", telemetryArray[sensorIndex].data.value3);
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
    const char* expectedSensors[] = {"Left_Shoulder", "Left_Upper", "Left_Elbow", "Left_Claw"};
    for (int j = 0; j < 4; j++) {
      bool found = false;
      unsigned long lastUpdate = 0;
      
      for (int i = 0; i < telemetryCount; i++) {
        if (strcmp(telemetryArray[i].name, expectedSensors[j]) == 0) {
          found = true;
          lastUpdate = millis() - telemetryArray[i].data.lastUpdate;
          Serial.printf("%s: Values: %d,%d,%d - Last update: %lu ms ago - %s\n", 
                        expectedSensors[j],
                        telemetryArray[i].data.value1,
                        telemetryArray[i].data.value2,
                        telemetryArray[i].data.value3,
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

  if(led == 1)
    digitalWrite(38, HIGH);
  if(led == 0)
    digitalWrite(38, LOW);
  lv_timer_handler(); /* let the GUI do its work */
  delay(10);
}
