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

#include <cstddef>

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "motor_wheel.hpp"
// sensor messages
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
// async serial
#include "asinus_hardware_interface/async_serial.hpp"





namespace asinus_hardware_interface
{
    class AsinusHardwareInterface : public hardware_interface::SystemInterface
    {
        struct HardwareConfig
        {
            std::string typeName = "asinus_two_wheel";
            std::string frontleftWheelJointName = "fl_wheel_joint";
            std::string frontrightWheelJointName = "fr_wheel_joint";
            std::string backleftWheelJointName = "bl_wheel_joint";
            std::string backrightWheelJointName = "br_wheel_joint";

            float loopRate = 10.0;
            int encoderTicksPerRevolution = 1024;
            std::string serial_device = "/dev/ttyUSB0";
            unsigned int serial_baud = 115200;
            double wheelRadius = 0.03;   // meters
            double wheelBase = 0.20;     // meters
        };

        struct PID {
            double kp, ki, kd;
            double integral = 0.0;
            double last_error = 0.0;

            PID(double p=0, double i=0, double d=0)
                : kp(p), ki(i), kd(d) {}

            double update(double error, double dt) {
                integral += error * dt;
                double derivative = (error - last_error) / dt;
                last_error = error;
                return kp * error + ki * integral + kd * derivative;
            }
        };


    public:
    RCLCPP_SHARED_PTR_DEFINITIONS(AsinusHardwareInterface)
        
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        PID pid_bl, pid_br, pid_fl, pid_fr;

    private:
        HardwareConfig hardwareConfig;

        MotorWheel frontleftWheel;
        MotorWheel frontrightWheel;
        MotorWheel backleftWheel;
        MotorWheel backrightWheel;

    /* IMU and GPS telemetry and publishers */
        IMU imu;
        GPS gps;
        rclcpp::Node::SharedPtr pub_node_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr quat_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;

        double wheel_radius_m_ = 0.03;
        double wheel_base_m_ = 0.20;
        double odom_x_ = 0.0;
        double odom_y_ = 0.0;
        double odom_yaw_ = 0.0;

    // serial reader
        std::unique_ptr<AsyncSerialReader> serial_reader_;
        std::string serial_rx_buffer_;
        std::mutex telemetry_mutex_;

        bool connect();
        bool disconnect();
        double normalize_angle(double angle);
    };
}