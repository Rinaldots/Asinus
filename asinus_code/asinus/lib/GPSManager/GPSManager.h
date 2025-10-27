#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "AsinusManager.h"



class GPSManager {
private:
    TinyGPSPlus gps;
    EspSoftwareSerial::UART* gpsSerial;
    int rxPin;
    int txPin;
    int baudRate;
    unsigned long lastStatusUpdate;
    
public:
    GPSManager(int rx_pin = 15, int tx_pin = 16, int baud = 9600);
    ~GPSManager();
    
    // Initialization
    bool begin();
    
    // Simplified interface for main.cpp
    bool initializeGPS();
    void readGPS();
    
    // Data processing
    bool update();
    
    // GPS status checks
    bool isLocationValid() const;
    bool isDateTimeValid() const;
    bool hasNewData();
    
    // GPS data getters
    double getLatitude();
    double getLongitude();
    double getAltitude();
    double getSpeed();
    double getCourse();
    double getHDOP();
    uint32_t getSatellites();
    
    // Date/Time getters
    uint8_t getDay();
    uint8_t getMonth();
    uint16_t getYear();
    uint8_t getHour();
    uint8_t getMinute();
    uint8_t getSecond();
    
    // Status and diagnostics
    uint32_t getCharsProcessed();
    bool shouldShowPeriodicStatus() const; // Check if 5 seconds have passed and no GPS fix
    void printStatus();
    void printLocationData();
    void printDateTime();
    void printFullGPSData();
    
    // Configuration
    String getLibraryVersion() const;

    GPSTelemetry returnTelemetry();
};

#endif // GPS_MANAGER_H
