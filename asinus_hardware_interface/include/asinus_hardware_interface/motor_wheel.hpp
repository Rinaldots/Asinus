// Copyright 2023 Robert Gruberski (Viola Robotics Sp. z o.o. Poland)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>
#include <cmath>
#include <cstdint>

#define ENCODER_MIN_VALUE 0
#define ENCODER_MAX_VALUE 65535
#define ENCODER_LOW_WRAP_FACTOR 0.3
#define ENCODER_HIGH_WRAP_FACTOR 0.7

namespace asinus_hardware_interface
{

    struct IMUTelemetry {
        float accel_x=0, accel_y=0, accel_z=0;
        float gyro_x=0, gyro_y=0, gyro_z=0;
        float mag_x=0, mag_y=0, mag_z=0;
        float temp = NAN;
        unsigned long ts = 0;
    };

    class IMU {

        public:
        void setTelemetry(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz,
                      float temperature, unsigned long timestamp) {
            imu.accel_x = ax;
            imu.accel_y = ay;
            imu.accel_z = az;
            imu.gyro_x = gx;
            imu.gyro_y = gy;
            imu.gyro_z = gz;
            imu.mag_x = mx;
            imu.mag_y = my;
            imu.mag_z = mz;
            imu.temp = temperature;
            imu.ts = timestamp;
        }
        IMUTelemetry getTelemetry() {
            return imu;
        }

        private:
        IMUTelemetry imu;
    };

    struct GPSTelemetry
    {
        double lat = NAN, lng = NAN;
        float hdop = NAN;
        unsigned long ts = 0;
    };
    class GPS {
        public:
        void setTelemetry(double latitude, double longitude, float hdop_value, unsigned long timestamp) {
            gps.lat = latitude;
            gps.lng = longitude;
            gps.hdop = hdop_value;
            gps.ts = timestamp;
        }
        GPSTelemetry getTelemetry() {
            return gps;
        }
        private:
        GPSTelemetry gps;
    };

    struct MotorTelemetry {
        int id = 0;
        uint8_t slaveState = 0;
        int16_t rawSpeed = 0;
        float speed = 0.0f;
        int32_t odom = 0;
        float volt = NAN;
        unsigned long ts = 0;
    };

    class MotorWheel {
    public:
        std::string name = "";

        int64_t encoderTicks = 0;
        int16_t encoderTicksPrevious = 0;
        int16_t encoderOverflowCount = 0;

        int32_t encoderLowWrap;
        int32_t encoderHighWrap;

        double position = 0.0;
        double velocity = 0.0;
        double command = 0;
        
        MotorTelemetry telemetry;

        double radiansPerRevolution = 0.0;

        MotorWheel() = default;

        MotorWheel(const std::string &wheelJointName, int encoderTicksPerRevolution)
        {
            name = wheelJointName;
            radiansPerRevolution = ((2 * M_PI) / encoderTicksPerRevolution);

            encoderLowWrap = ENCODER_LOW_WRAP_FACTOR * (ENCODER_MAX_VALUE - ENCODER_MIN_VALUE) + ENCODER_MIN_VALUE;
            encoderHighWrap = ENCODER_HIGH_WRAP_FACTOR * (ENCODER_MAX_VALUE - ENCODER_MIN_VALUE) + ENCODER_MIN_VALUE;
        }

        double calculateEncoderAngle()
        {
            return encoderTicks * radiansPerRevolution;
        }

    private:

    };
}
