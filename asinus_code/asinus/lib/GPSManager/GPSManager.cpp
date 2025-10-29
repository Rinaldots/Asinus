#include "GPSManager.h"
#include "AsinusManager.h"

GPSManager::GPSManager(int rx_pin, int tx_pin, int baud) 
    : rxPin(rx_pin), txPin(tx_pin), baudRate(baud), lastStatusUpdate(0) {
    gpsSerial = new EspSoftwareSerial::UART(rxPin, txPin);
}

GPSManager::~GPSManager() {
    delete gpsSerial;
}

bool GPSManager::begin() {
    gpsSerial->begin(baudRate);
    
    Serial.print(F("GPS initialized on pins RX:"));
    Serial.print(rxPin);
    Serial.print(F(", TX:"));
    Serial.println(txPin);
    Serial.print(F("TinyGPSPlus version: "));
    Serial.println(TinyGPSPlus::libraryVersion());
    Serial.println();
    
    return true; // GPS initialization is always considered successful
}

bool GPSManager::update() {
    bool newData = false;
    
    // Process all available GPS data
    while (gpsSerial->available()) {
        if (gps.encode(gpsSerial->read())) {
            newData = true;
        }
    }
    
    return newData;
}

bool GPSManager::isLocationValid() const {
    return gps.location.isValid();
}

bool GPSManager::isDateTimeValid() const {
    return gps.date.isValid() && gps.time.isValid();
}

bool GPSManager::hasNewData() {
    return gps.location.isUpdated();
}

double GPSManager::getLatitude() {
    return gps.location.lat();
}

double GPSManager::getLongitude() {
    return gps.location.lng();
}

double GPSManager::getAltitude() {
    return gps.altitude.meters();
}

double GPSManager::getSpeed() {
    return gps.speed.kmph();
}

double GPSManager::getCourse() {
    return gps.course.deg();
}

double GPSManager::getHDOP() {
    return gps.hdop.hdop();
}

uint32_t GPSManager::getSatellites() {
    return gps.satellites.value();
}

uint8_t GPSManager::getDay() {
    return gps.date.day();
}

uint8_t GPSManager::getMonth() {
    return gps.date.month();
}

uint16_t GPSManager::getYear() {
    return gps.date.year();
}

uint8_t GPSManager::getHour() {
    return gps.time.hour();
}

uint8_t GPSManager::getMinute() {
    return gps.time.minute();
}

uint8_t GPSManager::getSecond() {
    return gps.time.second();
}

uint32_t GPSManager::getCharsProcessed() {
    return gps.charsProcessed();
}

bool GPSManager::shouldShowPeriodicStatus() const {
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 5000 && !isLocationValid()) {
        const_cast<unsigned long&>(lastStatusUpdate) = millis(); // Safe const_cast for static variable
        return true;
    }
    return false;
}

void GPSManager::printStatus() {
    Serial.print(F("GPS Status: "));
    if (isLocationValid()) {
        Serial.println(F("Location fix acquired"));
    } else {
        Serial.print(F("Searching for satellites... "));
        Serial.print(F("Satellites: ")); Serial.print(getSatellites());
        Serial.print(F(", Characters processed: ")); Serial.println(getCharsProcessed());
    }
}

void GPSManager::printLocationData() {
    if (!isLocationValid()) {
        Serial.println(F("GPS location not valid"));
        return;
    }
    
    Serial.print(F("Satellites: ")); Serial.println(getSatellites());
    Serial.print(F("Location: ")); 
    Serial.print(getLatitude(), 6); Serial.print(F(", "));
    Serial.println(getLongitude(), 6);
    Serial.print(F("Altitude: ")); Serial.print(getAltitude()); Serial.println(F(" m"));
    Serial.print(F("Speed: ")); Serial.print(getSpeed()); Serial.println(F(" km/h"));
    Serial.print(F("Course: ")); Serial.print(getCourse()); Serial.println(F("°"));
    Serial.print(F("HDOP: ")); Serial.println(getHDOP());
}

void GPSManager::printDateTime() {
    if (isDateTimeValid()) {
        Serial.printf("DateTime: %02d/%02d/%04d %02d:%02d:%02d\n", 
                     getDay(), getMonth(), getYear(),
                     getHour(), getMinute(), getSecond());
    } else {
        Serial.println(F("GPS date/time not valid"));
    }
}

void GPSManager::printFullGPSData() {
    Serial.println(F("--- GPS ---"));
    printLocationData();
    printDateTime();
}

String GPSManager::getLibraryVersion() const {
    return String(TinyGPSPlus::libraryVersion());
}

// Simplified interface for main.cpp - Initialize GPS
bool GPSManager::initializeGPS() {
    bool result = begin();
    Serial.println(F("GPS: Waiting for satellite fix..."));
    return result;
}

// Simplified interface for main.cpp - Read and display GPS data
void GPSManager::readGPS() {
    // Update GPS data
    update();

    // Display data when GPS location is updated
    if (hasNewData()) {
        Serial.println(F("=== SENSOR DATA UPDATE ==="));
        
        // GPS Data
        printFullGPSData();
        
        Serial.println(F("============================"));
        Serial.println("");
    }
}

GPSTelemetry GPSManager::returnTelemetry() {
    update();
    GPSTelemetry gps;
    gps.lat = getLatitude();
    gps.lng = getLongitude();
    gps.hdop = getHDOP();
    gps.ts = millis();
    return gps;
}

