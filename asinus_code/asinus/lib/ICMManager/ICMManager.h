// ICMManager.h
// Lightweight manager for the SparkFun ICM-20948 (I2C) sensor
// Provides initialization and simple read/print helpers adapted from the example

#ifndef ICM_MANAGER_H
#define ICM_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <ICM_20948.h>

class ICMManager {
public:
    // sda / scl default to common ESP32 pins (can be overridden)
    ICMManager(int sda = 16, int scl = 17);

    // Initialize I2C (calls Wire.begin with provided pins)
    bool beginI2C();

    // Initialize the ICM-20948 device (returns true on success)
    bool initialize();

    // Call regularly to read & print scaled sensor values (if available)
    void update();

    // Convenience
    bool available() const { return icmAvailable; }

private:
    ICM_20948_I2C myICM;
    int sda_pin;
    int scl_pin;
    bool icmAvailable;

    // Small helpers copied/adapted from the example sketch
    void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);
    void printScaledAGMT();
};

#endif // ICM_MANAGER_H
