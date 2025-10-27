#include <Arduino.h>
#include <Wire.h>
#include "GPSManager.h"
#include "ICMManager.h"
#include "RelayManager.h"
#include "AsinusManager.h"
#include "MotorManager.h"
// Hardware serial ports used by the hover motors (ESP32 UART numbers)
HardwareSerial oSerialHover1(1);
HardwareSerial oSerialHover2(2);

// Instantiate the manager
MotorManager motorManager(oSerialHover1, oSerialHover2);

ICMManager icmManager;

GPSManager gpsManager;

//RelayManager relayManager(14,13,12,11,10,9,46,3,true); // Pinos dos relés e ativo em nível baixo

unsigned long lastDebugPrint = 0;
const unsigned long DEBUG_INTERVAL = 0; // Print debug every 2 seconds

void setup(void)
{
  Serial.begin(115200);
  icmManager.initialize();
  gpsManager.initializeGPS();
  motorManager.begin(19200, 21, 47, 19, 20);
  asinusManager.init(2, 1); // 2 motors, offset 0
  //relayManager.begin();
  //relayManager.turnOn(0); // Liga o relé 1
  //setup_icm();
}

#define UpdateRate 100 // Update at 100hz
void loop(void)
{
  
  asinusManager.setIMU(icmManager.returnTelemetry());
  asinusManager.setGPS(gpsManager.returnTelemetry());

  motorManager.update();


  // Periodic debug print
  
  unsigned long now = millis();
  if (now - lastDebugPrint >= DEBUG_INTERVAL) {
    //motorManager.printStatus();
    asinusManager.printCompactSerial();
    lastDebugPrint = now;
  }

  delay(1000 / UpdateRate);
}
