#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "GPSManager.h"
#include "ICMManager.h"
#include "RelayManager.h"
#include "AsinusManager.h"
#include "MotorManager.h"

// --- Helper Functions ---
namespace {
  void configureHardwareSerial(Stream& stream, long baud, int rxPin, int txPin) {
    // We must ensure the object passed here is actually HardwareSerial
    auto& hw = static_cast<HardwareSerial&>(stream);
    // Assuming HoverSetupEsp32 is a valid function in your library
    // that accepts (HardwareSerial, baud, rx, tx)
    HoverSetupEsp32(hw, baud, rxPin, txPin); 
  }

    void configureSoftwareSerial(Stream& stream, long baud, int rxPin, int txPin) {
    auto& sw = static_cast<SoftwareSerial&>(stream);
    
    // Remove "EspSoftwareSerial::" prefix. 
    // If your library version is recent, SWSERIAL_8N1 is globally available 
    // or under SoftwareSerialConfig::SWSERIAL_8N1
    sw.begin(baud, SWSERIAL_8N1, rxPin, txPin, false); 
  }
}

// --- Serial Port Definitions ---

// Ports 1 & 2 must be HardwareSerial to match the casting in configureHardwareSerial
SoftwareSerial oSerialHover1;
SoftwareSerial oSerialHover2;

// Ports 3 & 4 are SoftwareSerial (Class name is SoftwareSerial, not EspSoftwareSerial)
SoftwareSerial oSerialHover3;
SoftwareSerial oSerialHover4;

// Instantiate the manager
MotorManager motorManager(
  {oSerialHover1, configureSoftwareSerial},
  {oSerialHover2, configureSoftwareSerial},
  {oSerialHover3, configureSoftwareSerial},
  {oSerialHover4, configureSoftwareSerial}
);

ICMManager icmManager;
GPSManager gpsManager;
// RelayManager relayManager(...) // Kept commented as per original

// --- Timing Variables ---
unsigned long lastDebugPrint = 0;
const unsigned long DEBUG_INTERVAL = 400; // Print every 400ms
#define UpdateRate 30 
const unsigned long LOOP_INTERVAL = 1000 / UpdateRate;
unsigned long lastLoopTime = 0;

void setup(void)
{
  Serial.begin(115200);
  
  icmManager.initialize();
  gpsManager.initializeGPS();

  // Initialize motors with specific pins
  // Ensure these pins match the HW/SW Serial definitions
  motorManager.begin(19200, 45, 48, 42, 40, 21, 47, 19, 20);
  //21, 47, 19, 20
  asinusManager.init(4, 1); // 4 motors
}

void loop(void)
{
  unsigned long currentMillis = millis();

  // 1. Telemetry Updates (Run as fast as possible)
  asinusManager.setIMU(icmManager.returnTelemetry());
  asinusManager.setGPS(gpsManager.returnTelemetry());

  // 2. Motor Update (Reads feedback from hoverboards)
  motorManager.update();

  // 3. Debug Printing (NON-BLOCKING)
  if (currentMillis - lastDebugPrint >= DEBUG_INTERVAL) {
    lastDebugPrint = currentMillis;
    asinusManager.printDebug();
    // asinusManager.printCompactSerial(); 
  }

  // 4. Control Loop (Throttled to UpdateRate)
  if (currentMillis - lastLoopTime >= LOOP_INTERVAL) {
    lastLoopTime = currentMillis;

    motorManager.processSerialCommands();
    motorManager.sendMotorCommands();
  }
  
  // No delay() here to allow the loop to run freely for incoming serial data
}