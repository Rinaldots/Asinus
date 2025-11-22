#ifndef ICM_MANAGER_H
#define ICM_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <ICM_20948.h>
#include "AsinusManager.h"

// **************************************************************************************
// IMPORTANTE: Para usar o DMP, você DEVE descomentar a linha #define ICM_20948_USE_DMP
// em: SparkFun_ICM-20948_ArduinoLibrary/src/util/ICM_20948_C.h
// **************************************************************************************

class ICMManager
{
public:
    // sda / scl default to common ESP32 pins (can be overridden)
    ICMManager(int sda = 18, int scl = 17);

    // Initialize I2C (calls Wire.begin with provided pins)
    bool beginI2C();

    // Initialize the ICM-20948 device AND the DMP (returns true on success)
    bool initialize();

    // Call regularly to read & print SCALED sensor values (if available) - Now reads DMP data too!
    void update();

    // Convenience
    bool available() const { return icmAvailable; }

    // Returns a populated IMUTelemetry struct
    IMUTelemetry returnTelemetry();

private:
    ICM_20948_I2C myICM;
    int sda_pin;
    int scl_pin;
    bool icmAvailable;
    bool dmpInitialized; // Novo flag para o estado do DMP

    // Small helpers copied/adapted from the example sketch
    void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);
    void printScaledAGMT();

    // NOVO: Processa o quaternion do DMP e preenche o struct IMUTelemetry
    void processDMPData(icm_20948_DMP_data_t *data, IMUTelemetry *imu);
};

#endif // ICM_MANAGER_H