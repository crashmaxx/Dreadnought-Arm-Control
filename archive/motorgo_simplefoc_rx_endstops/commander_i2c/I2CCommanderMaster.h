
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <inttypes.h>
#include "./SimpleFOCRegisters.h"
#include "./CommanderMaster.h"

#if !defined(I2COMMANDER_DEFAULT_MAX_REMOTE_MOTORS)
#define I2COMMANDER_DEFAULT_MAX_REMOTE_MOTORS 4
#endif

namespace commander_i2c {

    enum FOCMotorStatus : uint8_t {
        motor_uninitialized = 0x00,     //!< Motor is not yet initialized
        motor_initializing  = 0x01,     //!< Motor intiialization is in progress
        motor_uncalibrated  = 0x02,     //!< Motor is initialized, but not calibrated (open loop possible)
        motor_calibrating   = 0x03,     //!< Motor calibration in progress
        motor_ready         = 0x04,     //!< Motor is initialized and calibrated (closed loop possible)
        motor_error         = 0x08,     //!< Motor is in error state (recoverable, e.g. overcurrent protection active)
        motor_calib_failed  = 0x0E,     //!< Motor calibration failed (possibly recoverable)
        motor_init_failed   = 0x0F,     //!< Motor initialization failed (not recoverable)
    };


    typedef struct {
        TwoWire* wire;
        uint8_t address;    
    } I2CRemoteMotor;


    class I2CCommanderMaster : public CommanderMaster {

        public:
            I2CCommanderMaster(int maxMotors = I2COMMANDER_DEFAULT_MAX_REMOTE_MOTORS);
            ~I2CCommanderMaster();
            void init();
            void addI2CMotors(uint8_t i2cAddress, uint8_t motorCount, TwoWire *wire = &Wire);

            int writeRegister(int motor, SimpleFOCRegister registerNum, void* data, uint8_t size) override;
            int readRegister(int motor, SimpleFOCRegister registerNum, void* data, uint8_t size) override;
            int readLastUsedRegister(int motor, void* data, uint8_t size);

            // Motor intialization interface for convenience - think about how this will best work
            // i.e. which parameters should be set by i2c and which should be hard-coded, and where config info is saved
            // TODO bool initializeMotor(int motor);

        private:
            int maxMotors;
            int numMotors = 0;
            I2CRemoteMotor* motors;
    };


};