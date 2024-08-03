#include "motorgo_mini_pins.h"
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#include "encoders/calibrated/CalibratedSensor.h"

// position / angle sensor instance from SimpleFOCDrivers
MagneticSensorMT6701SSI sensor_ch0(CH0_ENC_CS);
MagneticSensorMT6701SSI sensor_ch1(CH1_ENC_CS);

SPIClass hspi = SPIClass(HSPI);

void setup() {
    // start serial
    Serial.begin(115200);

    // initialize sensor
    hspi.begin(ENC_SCL, ENC_SDA, ENC_MOSI);
    sensor_ch0.init(&hspi);
    sensor_ch1.init(&hspi);

    Serial.println("Sensor ready");

    _delay(1000);
}

void loop() {
    sensor_ch0.update();
    sensor_ch1.update();

    // display the angle and the angular velocity to the terminal
    Serial.print(sensor_ch0.getAngle());
    Serial.print("\t");
    Serial.print(sensor_ch0.getVelocity());
    Serial.print("\t");
    Serial.print(sensor_ch1.getAngle());
    Serial.print("\t");
    Serial.println(sensor_ch1.getVelocity());

    delay(100);
}
