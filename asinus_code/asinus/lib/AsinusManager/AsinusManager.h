#ifndef ASINUS_MANAGER_H
#define ASINUS_MANAGER_H

#include <vector>
#include <cstddef>
#include <Arduino.h>

struct MotorTelemetry {
  int id = -1;
  uint8_t slaveState = 0;
  int16_t rawSpeed = 0;
  float speed = 0.0f;
  int32_t odom = 0;
  float volt = NAN;
  unsigned long ts = 0;
};

struct IMUTelemetry {
  float accel_x=0, accel_y=0, accel_z=0;
  float gyro_x=0, gyro_y=0, gyro_z=0;
  float mag_x=0, mag_y=0, mag_z=0;
  float temp = NAN;
  unsigned long ts = 0;
};

struct GPSTelemetry {
  double lat = NAN, lng = NAN;
  float hdop = NAN;
  unsigned long ts = 0;
};

class AsinusManager {
public:
  AsinusManager();
  // initialize storage for N motors and optional offset
  void init(size_t motorCount, int motorOffset = 0);

  // update from packet fields (decoupled from hover struct types)
  void updateMotorFromPacket(int slaveId, int16_t speed100, uint16_t volt100, unsigned long nowMillis);

  // update for a given motor index (0-based); idOverride=-1 uses index+offset
  void updateMotorByIndex(int motorIndex, int16_t speed100, int32_t odom100, uint16_t volt100, unsigned long nowMillis, int idOverride = -1);

  // accessors
  const std::vector<MotorTelemetry>& motors() const { return _motors; }
  MotorTelemetry getMotor(size_t idx) const;
  
  // imu/gps setters
  void setIMU(const IMUTelemetry m) { imu = m; }
  void setGPS(const GPSTelemetry g) { gps = g; }

  // debug print telemetry
  void printDebug();
  
  // Compact serial output for ROS2 (CSV-like format, easy to parse)
  void setupRos2();

  void printCompactSerial();

  void setupPublishers();

  void setupSubscribers();

  void publishCallbacks();

  void subscribeCallbacks();


  IMUTelemetry imu;
  GPSTelemetry gps;
  

private:
  std::vector<MotorTelemetry> _motors;
  int _motorOffset = 0;
};

extern AsinusManager asinusManager;

#endif // ASINUS_MANAGER_H
