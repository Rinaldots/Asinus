#include "AsinusManager.h"
#include <math.h>

AsinusManager asinusManager;

AsinusManager::AsinusManager() {
}

void AsinusManager::init(size_t motorCount, int motorOffset) {
  _motorOffset = motorOffset;
  _motors.clear();
  _motors.resize(motorCount);
  for (size_t i = 0; i < motorCount; ++i) {
    _motors[i].id = (int)(i + motorOffset);
    _motors[i].volt = NAN;
  }
}

void AsinusManager::updateMotorFromPacket(int slaveId, int16_t speed100, uint16_t volt100, unsigned long nowMillis) {
  int idx = slaveId - _motorOffset;
  if (idx < 0) return;
  if ((size_t)idx >= _motors.size()) return;
  MotorTelemetry &m = _motors[idx];
  m.id = slaveId;
  m.rawSpeed = speed100;
  m.speed = (float)speed100 / 100.0f;
  // treat zero as unavailable if voltage field is zero in your protocol
  m.volt = (volt100 == 0) ? NAN : ((float)volt100 / 100.0f);
  m.slaveState = 0; // user code can set this from slave_state array if desired
  m.ts = nowMillis;
}

void AsinusManager::updateMotorByIndex(int motorIndex, int16_t speed100, int32_t odom100, uint16_t volt100, unsigned long nowMillis, int idOverride) {
  if (motorIndex < 0) return;
  if ((size_t)motorIndex >= _motors.size()) return;
  
  MotorTelemetry &m = _motors[motorIndex];
  m.id = motorIndex + _motorOffset;  // Assign logical ID based on index
  m.rawSpeed = speed100;
  m.speed = (float)speed100 / 100.0f;
  m.odom = odom100;
  m.volt = (volt100 == 0) ? NAN : ((float)volt100 / 100.0f);
  m.slaveState = 0;
  m.ts = nowMillis;
}

MotorTelemetry AsinusManager::getMotor(size_t idx) const {
  if (idx < _motors.size()) return _motors[idx];
  return MotorTelemetry();
}

void AsinusManager::printDebug() {
  Serial.println("\n========== AsinusManager Debug ==========");
  
  // Motor telemetry
  Serial.println("--- Motors ---");
  for (size_t i = 0; i < _motors.size(); i++) {
    const MotorTelemetry &m = _motors[i];
    Serial.print("Motor[");
    Serial.print(i);
    Serial.print("] ID:");
    Serial.print(m.id);
    Serial.print(" Speed:");
    Serial.print(m.speed);
    Serial.print(" Odom:");
    Serial.print(m.odom);
    Serial.print(" Volt:");
    if (!isnan(m.volt)) {
      Serial.print(m.volt);
      Serial.print("V");
    } else {
      Serial.print("N/A");
    }
    Serial.print(" State:");
    Serial.print(m.slaveState);
    Serial.print(" TS:");
    Serial.println(m.ts);
  }
  
  // IMU telemetry
  Serial.println("--- IMU ---");
  Serial.print("Accel(m/s²): X:");
  Serial.print(imu.accel_x);
  Serial.print(" Y:");
  Serial.print(imu.accel_y);
  Serial.print(" Z:");
  Serial.println(imu.accel_z);
  Serial.print("Gyro(rad/s): X:");
  Serial.print(imu.gyro_x);
  Serial.print(" Y:");
  Serial.print(imu.gyro_y);
  Serial.print(" Z:");
  Serial.println(imu.gyro_z);
  Serial.print("Mag(uT): X:");
  Serial.print(imu.mag_x);
  Serial.print(" Y:");
  Serial.print(imu.mag_y);
  Serial.print(" Z:");
  Serial.println(imu.mag_z);
  Serial.print("Temp: ");
  if (!isnan(imu.temp)) {
    Serial.print(imu.temp);
    Serial.println("°C");
  } else {
    Serial.println("N/A");
  }
  Serial.print("IMU TS: ");
  Serial.println(imu.ts);
  
  // GPS telemetry
  Serial.println("--- GPS ---");
  Serial.print("Lat: ");
  if (!isnan(gps.lat)) {
    Serial.print(gps.lat, 6);
  } else {
    Serial.print("N/A");
  }
  Serial.print(" Lng: ");
  if (!isnan(gps.lng)) {
    Serial.print(gps.lng, 6);
  } else {
    Serial.print("N/A");
  }
  Serial.print(" HDOP: ");
  if (!isnan(gps.hdop)) {
    Serial.print(gps.hdop);
  } else {
    Serial.print("N/A");
  }
  Serial.print(" GPS TS: ");
  Serial.println(gps.ts);
  
  Serial.println("=========================================\n");
}

void AsinusManager::printCompactSerial() {
  // Compact format optimized for ROS2 parsing
  // Format: ASINUS,ts,m0_id,m0_spd,m0_odom,m0_volt,m1_id,m1_spd,m1_odom,m1_volt,ax,ay,az,gx,gy,gz,mx,my,mz,temp,lat,lng,hdop
  
  Serial.print("ASINUS,");
  Serial.print(millis());
  
  // Motor telemetry (2 motors)
  for (size_t i = 0; i < 2; i++) {
    Serial.print(",");
    if (i < _motors.size()) {
      const MotorTelemetry &m = _motors[i];
      Serial.print(m.id);
      Serial.print(",");
      Serial.print(m.speed, 2);
      Serial.print(",");
      Serial.print(m.odom);
      Serial.print(",");
      if (!isnan(m.volt)) {
        Serial.print(m.volt, 2);
      } else {
        Serial.print("0.00");
      }
    } else {
      Serial.print("0,0.00,0,0.00");
    }
  }
  
  // IMU telemetry
  Serial.print(",");
  Serial.print(imu.accel_x, 3);
  Serial.print(",");
  Serial.print(imu.accel_y, 3);
  Serial.print(",");
  Serial.print(imu.accel_z, 3);
  Serial.print(",");
  Serial.print(imu.gyro_x, 3);
  Serial.print(",");
  Serial.print(imu.gyro_y, 3);
  Serial.print(",");
  Serial.print(imu.gyro_z, 3);
  Serial.print(",");
  Serial.print(imu.mag_x, 2);
  Serial.print(",");
  Serial.print(imu.mag_y, 2);
  Serial.print(",");
  Serial.print(imu.mag_z, 2);
  Serial.print(",");
  if (!isnan(imu.temp)) {
    Serial.print(imu.temp, 2);
  } else {
    Serial.print("0.00");
  }
  
  // GPS telemetry
  Serial.print(",");
  if (!isnan(gps.lat)) {
    Serial.print(gps.lat, 6);
  } else {
    Serial.print("0.000000");
  }
  Serial.print(",");
  if (!isnan(gps.lng)) {
    Serial.print(gps.lng, 6);
  } else {
    Serial.print("0.000000");
  }
  Serial.print(",");
  if (!isnan(gps.hdop)) {
    Serial.print(gps.hdop, 2);
  } else {
    Serial.print("99.99");
  }
  
  Serial.println();
}