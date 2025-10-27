#include <Arduino.h>
#include <Wire.h>
#include "motor_handler.h"
#include "GPSManager.h"
#include "ICMManager.h"

ICMManager icmManager;

GPSManager gpsManager;

void setup(void)
{
  Serial.begin(115200);
  setup_motor();
  icmManager.initialize();
  gpsManager.initializeGPS();
  //setup_icm();
}

void loop(void)
{
  //update_motor();
  delay(1);
}