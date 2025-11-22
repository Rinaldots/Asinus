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

#include "asinus_hardware_interface/asinus_hardware_interface.hpp"
// sensor messages
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <cmath>
#include <vector>
#include <limits>


namespace asinus_hardware_interface
{
    hardware_interface::CallbackReturn AsinusHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        // Read hardware parameters safely (use defaults when missing)
        auto it = info.hardware_parameters.find("typeName");
        if (it != info.hardware_parameters.end()) {
            hardwareConfig.typeName = it->second;
        } else {
            RCLCPP_FATAL(
                rclcpp::get_logger("AsinusHardwareInterface"),
                "Missing required hardware parameter 'typeName' (expected 'asinus_two_wheel' or 'asinus_four_wheel').");
            return hardware_interface::CallbackReturn::ERROR;
        }

        it = info.hardware_parameters.find("front_left_wheel_joint_name");
        if (it != info.hardware_parameters.end()) {
            hardwareConfig.frontleftWheelJointName = it->second;
        }
        it = info.hardware_parameters.find("front_right_wheel_joint_name");
        if (it != info.hardware_parameters.end()) {
            hardwareConfig.frontrightWheelJointName = it->second;
        }
        it = info.hardware_parameters.find("back_left_wheel_joint_name");
        if (it != info.hardware_parameters.end()) {
            hardwareConfig.backleftWheelJointName = it->second;
        }
        it = info.hardware_parameters.find("back_right_wheel_joint_name");
        if (it != info.hardware_parameters.end()) {
            hardwareConfig.backrightWheelJointName = it->second;
        }

        it = info.hardware_parameters.find("serial_device");
        if (it != info.hardware_parameters.end()) {
            hardwareConfig.serial_device = it->second;
        }

        it = info.hardware_parameters.find("loop_rate");
        if (it != info.hardware_parameters.end()) {
            hardwareConfig.loopRate = std::stof(it->second);
        }
        it = info.hardware_parameters.find("encoder_ticks_per_revolution");
        if (it != info.hardware_parameters.end()) {
            hardwareConfig.encoderTicksPerRevolution = std::stoi(it->second);
        }
        it = info.hardware_parameters.find("serial_baud");
        if (it != info.hardware_parameters.end()) {
            hardwareConfig.serial_baud = static_cast<unsigned int>(std::stoul(it->second));
        }

        it = info.hardware_parameters.find("wheel_radius_m");
        if (it != info.hardware_parameters.end()) {
            hardwareConfig.wheelRadius = std::stod(it->second);
        }
        wheel_radius_m_ = hardwareConfig.wheelRadius;

        it = info.hardware_parameters.find("wheel_base_m");
        if (it != info.hardware_parameters.end()) {
            hardwareConfig.wheelBase = std::stod(it->second);
        }
        wheel_base_m_ = hardwareConfig.wheelBase;
        


    if (hardwareConfig.typeName == "asinus_four_wheel")
    {
    if (hardwareConfig.frontleftWheelJointName.empty() ||
        hardwareConfig.frontrightWheelJointName.empty() ||
        hardwareConfig.backleftWheelJointName.empty() ||
        hardwareConfig.backrightWheelJointName.empty())
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("AsinusHardwareInterface"),
            "Four-wheel hardware requires all wheel joint names to be set.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    frontleftWheel = MotorWheel(hardwareConfig.frontleftWheelJointName, hardwareConfig.encoderTicksPerRevolution);
    frontrightWheel = MotorWheel(hardwareConfig.frontrightWheelJointName, hardwareConfig.encoderTicksPerRevolution);
    backleftWheel = MotorWheel(hardwareConfig.backleftWheelJointName, hardwareConfig.encoderTicksPerRevolution);
    backrightWheel = MotorWheel(hardwareConfig.backrightWheelJointName, hardwareConfig.encoderTicksPerRevolution);
    }
    else if(hardwareConfig.typeName == "asinus_two_wheel")
    {
    if (hardwareConfig.backleftWheelJointName.empty() ||
        hardwareConfig.backrightWheelJointName.empty())
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("AsinusHardwareInterface"),
            "Two-wheel hardware requires back left/right wheel joint names to be set.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    backleftWheel = MotorWheel(hardwareConfig.backleftWheelJointName, hardwareConfig.encoderTicksPerRevolution);
    backrightWheel = MotorWheel(hardwareConfig.backrightWheelJointName, hardwareConfig.encoderTicksPerRevolution);
    }
    else
    {
    RCLCPP_FATAL(
        rclcpp::get_logger("AsinusHardwareInterface"),
        "Unknown hardware typeName '%s'. Expected 'asinus_two_wheel' or 'asinus_four_wheel'.",
        hardwareConfig.typeName.c_str());
    return hardware_interface::CallbackReturn::ERROR;
    }

        for (const hardware_interface::ComponentInfo & joint : info.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AsinusHardwareInterface"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());

                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AsinusHardwareInterface"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AsinusHardwareInterface"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(), joint.state_interfaces.size());
                
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AsinusHardwareInterface"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AsinusHardwareInterface"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> AsinusHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        RCLCPP_INFO(
            rclcpp::get_logger("AsinusHardwareInterface"),
            "export_state_interfaces called, typeName='%s', fl='%s', fr='%s', bl='%s', br='%s'",
            hardwareConfig.typeName.c_str(),
            frontleftWheel.name.c_str(),
            frontrightWheel.name.c_str(),
            backleftWheel.name.c_str(),
            backrightWheel.name.c_str());
        if (hardwareConfig.typeName == "asinus_four_wheel"){
        state_interfaces.emplace_back(hardware_interface::StateInterface(frontleftWheel.name, hardware_interface::HW_IF_POSITION, &frontleftWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(frontleftWheel.name, hardware_interface::HW_IF_VELOCITY, &frontleftWheel.velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(frontrightWheel.name, hardware_interface::HW_IF_POSITION, &frontrightWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(frontrightWheel.name, hardware_interface::HW_IF_VELOCITY, &frontrightWheel.velocity));
        }
        state_interfaces.emplace_back(hardware_interface::StateInterface(backleftWheel.name, hardware_interface::HW_IF_POSITION, &backleftWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(backleftWheel.name, hardware_interface::HW_IF_VELOCITY, &backleftWheel.velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(backrightWheel.name, hardware_interface::HW_IF_POSITION, &backrightWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(backrightWheel.name, hardware_interface::HW_IF_VELOCITY, &backrightWheel.velocity));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> AsinusHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        if (hardwareConfig.typeName == "asinus_four_wheel"){
        command_interfaces.emplace_back(hardware_interface::CommandInterface(frontleftWheel.name, hardware_interface::HW_IF_VELOCITY, &frontleftWheel.command));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(frontrightWheel.name, hardware_interface::HW_IF_VELOCITY, &frontrightWheel.command));
        }
        command_interfaces.emplace_back(hardware_interface::CommandInterface(backleftWheel.name, hardware_interface::HW_IF_VELOCITY, &backleftWheel.command));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(backrightWheel.name, hardware_interface::HW_IF_VELOCITY, &backrightWheel.command));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn AsinusHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("AsinusHardwareInterface"), "Configuring... please wait a moment...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn AsinusHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("AsinusHardwareInterface"), "Cleaning up... please wait a moment...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn AsinusHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {

        // create an internal node to host publishers
        if (!pub_node_) {
            pub_node_ = rclcpp::Node::make_shared("asinus_hardware_interface_pubs");
        }

    imu_pub_ = pub_node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::QoS(10));
    gps_pub_ = pub_node_->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", rclcpp::QoS(10));
    mag_pub_ = pub_node_->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", rclcpp::QoS(10));
    odom_pub_ = pub_node_->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(20));
    quat_pub_ = pub_node_->create_publisher<geometry_msgs::msg::QuaternionStamped>("imu/orientation", rclcpp::QoS(10));
    battery_pub_ = pub_node_->create_publisher<sensor_msgs::msg::BatteryState>("battery/state", rclcpp::QoS(10));

    RCLCPP_INFO(rclcpp::get_logger("AsinusHardwareInterface"), "IMU, GPS, Magnetic field, Odometry, Quaternion and Battery publishers initialized.");

        try {
            //RCLCPP_INFO(rclcpp::get_logger("AsinusHardwareInterface"), "Starting async serial reader on %s @ %u", hardwareConfig.serial_device.c_str(), hardwareConfig.serial_baud);
            serial_reader_ = std::make_unique<AsyncSerialReader>(hardwareConfig.serial_device, hardwareConfig.serial_baud);
            serial_reader_->set_callback([this](const std::vector<uint8_t> &data) {
                std::string chunk(reinterpret_cast<const char*>(data.data()), data.size());
                //RCLCPP_DEBUG(rclcpp::get_logger("AsinusHardwareInterface"), "Serial chunk (%zu): '%s'", data.size(), chunk.c_str());
                this->serial_rx_buffer_.append(chunk);
                size_t pos = 0;
                while ((pos = this->serial_rx_buffer_.find('\n')) != std::string::npos) {
                    std::string line = this->serial_rx_buffer_.substr(0, pos);
                    if (!line.empty() && line.back() == '\r') line.pop_back();
                        try {
                            unsigned long now = static_cast<unsigned long>(rclcpp::Clock().now().nanoseconds() / 1000000);
                        if (line.rfind("ASINUS,", 0) == 0) {
                               
                            std::vector<std::string> toks;
                            size_t start = 7; // after "ASINUS,"


                            while (true) {
                                size_t comma = line.find(',', start);
                                if (comma == std::string::npos) { toks.push_back(line.substr(start)); break; }
                                toks.push_back(line.substr(start, comma - start));
                                start = comma + 1;
                            }
                            // Motor 1
                            if (toks.size() > 5)
                            {
                                try {
                                    
                                    int mo_id = std::stoi(toks[1]);
                                    float m0_spd = std::stof(toks[2]);
                                    int m0_odom = std::stoi(toks[3]);
                                    float m0_volt = std::stof(toks[4]);
                                    unsigned long ts = now;
                                    std::lock_guard<std::mutex> lk(this->telemetry_mutex_);

                                    backrightWheel.telemetry.id = mo_id;
                                    backrightWheel.telemetry.speed = m0_spd;
                                    backrightWheel.telemetry.odom = m0_odom;
                                    backrightWheel.telemetry.volt = m0_volt;
                                    backrightWheel.telemetry.ts = ts;
                                    //std::cerr<<"debug motor BR: id="<<mo_id<<" spd="<<m0_spd<<" odom="<<m0_odom<<" volt="<<m0_volt<<std::endl;
                                } 
                                catch (...) {
                                    // ignore parse errors for telemetry
                                }
                            }

                            if (toks.size() > 9)
                            {
                                try {
                                    int m1_id = std::stoi(toks[5]);
                                    float m1_spd = std::stof(toks[6]);
                                    int m1_odom = std::stoi(toks[7]);
                                    float m1_volt = std::stof(toks[8]);
                                    unsigned long ts = now;
                                    std::lock_guard<std::mutex> lk(this->telemetry_mutex_);

                                    backleftWheel.telemetry.id = m1_id;
                                    backleftWheel.telemetry.speed = m1_spd;
                                    backleftWheel.telemetry.odom = m1_odom;
                                    backleftWheel.telemetry.volt = m1_volt;
                                    backleftWheel.telemetry.ts = ts;
                                } 
                                catch (...) {
                                    // ignore parse errors for telemetry
                                }
                            }
                            
                            // GPS
                            if (toks.size() > 21) {
                                try {
                                    float lat = std::stof(toks[19]);
                                    float lng = std::stof(toks[20]);
                                    float hdop = std::stof(toks[21]);
                                    unsigned long ts = now;
                                    std::lock_guard<std::mutex> lk(this->telemetry_mutex_);
                                    this->gps.setTelemetry(lat, lng, hdop, ts);
                                } catch (...) {
                                    // ignore parse errors for GPS
                                }
                            }

                            // IMU accel/gyro (best-effort)
                            if (toks.size() > 18) {
                                try {
                                    float ax = std::stof(toks[9]);
                                    float ay = std::stof(toks[10]);
                                    float az = std::stof(toks[11]);
                                    float gx = std::stof(toks[12]);
                                    float gy = std::stof(toks[13]);
                                    float gz = std::stof(toks[14]);
                                    float mx = std::stof(toks[15]);
                                    float my = std::stof(toks[16]);
                                    float mz = std::stof(toks[17]);
                                    float temp = std::stof(toks[18]);
                                    unsigned long ts = now;
                                    std::lock_guard<std::mutex> lk(this->telemetry_mutex_);
                                    this->imu.setTelemetry(ax, ay, az, gx, gy, gz, mx, my, mz, temp, ts);
                                } catch (...) {
                                    // ignore parse errors for IMU
                                }
                            }

                            // Optional quaternion + velocity tokens
                            if (toks.size() > 25) {
                                try {
                                    std::lock_guard<std::mutex> lk(this->telemetry_mutex_);
                                    IMUTelemetry t = this->imu.getTelemetry();
                                    t.qx = std::stof(toks[22]);
                                    t.qy = std::stof(toks[23]);
                                    t.qz = std::stof(toks[24]);
                                    t.qw = std::stof(toks[25]);

                                    if (toks.size() > 26) {
                                        t.vx = std::stof(toks[26]);
                                    }
                                    if (toks.size() > 27) {
                                        t.wz = std::stof(toks[27]);
                                    }
                                    if (toks.size() > 30) {
                                        t.yaw = std::stof(toks[28]);
                                        t.pitch = std::stof(toks[29]);
                                        t.roll = std::stof(toks[30]);
                                    }

                                    this->imu.setTelemetry(t);
                                } catch (...) {
                                    // ignore parse errors for quaternion/velocity tokens
                                }
                            }
                        }
                    } catch (const std::exception &e) {
                        RCLCPP_WARN(rclcpp::get_logger("AsinusHardwareInterface"), "Failed to parse serial line: %s", e.what());
                    }

                    // erase processed line and newline
                    this->serial_rx_buffer_.erase(0, pos + 1);
                }

            });

            bool started = serial_reader_->start();
            if (started) {
                RCLCPP_INFO(rclcpp::get_logger("AsinusHardwareInterface"), "Async serial reader started on %s @ %u", hardwareConfig.serial_device.c_str(), hardwareConfig.serial_baud);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("AsinusHardwareInterface"), "Async serial reader failed to open %s @ %u", hardwareConfig.serial_device.c_str(), hardwareConfig.serial_baud);
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(rclcpp::get_logger("AsinusHardwareInterface"), "Failed to start serial reader: %s", e.what());
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn AsinusHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("AsinusHardwareInterface"), "Deactivating... please wait a moment...");

        // stop serial reader if running
        if (serial_reader_) {
            serial_reader_->stop();
            serial_reader_.reset();
            RCLCPP_INFO(rclcpp::get_logger("AsinusHardwareInterface"), "Async serial reader stopped.");
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type AsinusHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
    

        double frontleftlastPosition = frontleftWheel.position;
        frontleftWheel.encoderTicks = frontleftWheel.telemetry.odom;
        frontleftWheel.position = frontleftWheel.calculateEncoderAngle();
        frontleftWheel.velocity = (frontleftWheel.position - frontleftlastPosition) / period.seconds();

        
        double frontrightlastPosition = frontrightWheel.position;
        // sensor hall ta invertido
        frontrightWheel.encoderTicks = frontrightWheel.telemetry.odom;
        frontrightWheel.position = frontrightWheel.calculateEncoderAngle();
        frontrightWheel.velocity = (frontrightWheel.position - frontrightlastPosition) / period.seconds();

        double backleftlastPosition = backleftWheel.position;
        backleftWheel.encoderTicks = backleftWheel.telemetry.odom;
        backleftWheel.position = backleftWheel.calculateEncoderAngle();
        backleftWheel.velocity = (backleftWheel.position - backleftlastPosition) / period.seconds();

        double backrightlastPosition = backrightWheel.position;
        backrightWheel.encoderTicks = backrightWheel.telemetry.odom;
        backrightWheel.position = backrightWheel.calculateEncoderAngle();
        backrightWheel.velocity = (backrightWheel.position - backrightlastPosition) / period.seconds();

        const double delta_left = (backleftWheel.position - backleftlastPosition) * wheel_radius_m_;
        const double delta_right = (backrightWheel.position - backrightlastPosition) * wheel_radius_m_;
        const double delta_s = (delta_right + delta_left) / 2.0;
        double delta_theta = 0.0;
        if (wheel_base_m_ > 1e-6) {
            delta_theta = (delta_right - delta_left) / wheel_base_m_;
        }
        const double yaw_mid = odom_yaw_ + delta_theta / 2.0;
        odom_x_ += delta_s * std::cos(yaw_mid);
        odom_y_ += delta_s * std::sin(yaw_mid);
        odom_yaw_ = normalize_angle(odom_yaw_ + delta_theta);

        const double v_left = backleftWheel.velocity;
        const double v_right = backrightWheel.velocity;
        const double linear_left = v_left * wheel_radius_m_;
        const double linear_right = v_right * wheel_radius_m_;
        const double vx = (linear_right + linear_left) / 2.0;
        double wz = 0.0;
        if (wheel_base_m_ > 1e-6) {
            wz = (linear_right - linear_left) / wheel_base_m_;
        }

        IMUTelemetry t_imu;
        {
            std::lock_guard<std::mutex> lk(this->telemetry_mutex_);
            t_imu = imu.getTelemetry();
        }
        const rclcpp::Time now = rclcpp::Clock().now();

        if (imu_pub_ && pub_node_) {
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = now;
            imu_msg.header.frame_id = "imu";
            imu_msg.orientation.w = 1.0;
            imu_msg.linear_acceleration.x = t_imu.accel_x;
            imu_msg.linear_acceleration.y = t_imu.accel_y;
            imu_msg.linear_acceleration.z = t_imu.accel_z;
            imu_msg.angular_velocity.x = t_imu.gyro_x;
            imu_msg.angular_velocity.y = t_imu.gyro_y;
            imu_msg.angular_velocity.z = t_imu.gyro_z;

            const bool has_quaternion = !std::isnan(t_imu.qx) && !std::isnan(t_imu.qy) &&
                                        !std::isnan(t_imu.qz) && !std::isnan(t_imu.qw);
            if (has_quaternion) {
                imu_msg.orientation.x = t_imu.qx;
                imu_msg.orientation.y = t_imu.qy;
                imu_msg.orientation.z = t_imu.qz;
                imu_msg.orientation.w = t_imu.qw;
            }
            imu_msg.orientation_covariance[0] = has_quaternion ? 1e-3 : -1.0;
            imu_msg.orientation_covariance[4] = has_quaternion ? 1e-3 : 0.0;
            imu_msg.orientation_covariance[8] = has_quaternion ? 1e-3 : 0.0;
            imu_msg.angular_velocity_covariance[0] = 1e-3;
            imu_msg.angular_velocity_covariance[4] = 1e-3;
            imu_msg.angular_velocity_covariance[8] = 1e-3;
            imu_msg.linear_acceleration_covariance[0] = 1e-2;
            imu_msg.linear_acceleration_covariance[4] = 1e-2;
            imu_msg.linear_acceleration_covariance[8] = 1e-2;

            imu_pub_->publish(imu_msg);

            if (mag_pub_) {
                sensor_msgs::msg::MagneticField mag_msg;
                mag_msg.header = imu_msg.header;
                constexpr double micro_tesla_to_tesla = 1e-6;
                if (!std::isnan(t_imu.mag_x)) {
                    mag_msg.magnetic_field.x = t_imu.mag_x * micro_tesla_to_tesla;
                }
                if (!std::isnan(t_imu.mag_y)) {
                    mag_msg.magnetic_field.y = t_imu.mag_y * micro_tesla_to_tesla;
                }
                if (!std::isnan(t_imu.mag_z)) {
                    mag_msg.magnetic_field.z = t_imu.mag_z * micro_tesla_to_tesla;
                }
                mag_msg.magnetic_field_covariance[0] = -1.0;
                mag_pub_->publish(mag_msg);
            }

            if (quat_pub_ && has_quaternion) {
                geometry_msgs::msg::QuaternionStamped qs;
                qs.header = imu_msg.header;
                qs.quaternion = imu_msg.orientation;
                quat_pub_->publish(qs);
            }
        }

        if (odom_pub_) {
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = now;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";
            odom_msg.pose.pose.position.x = odom_x_;
            odom_msg.pose.pose.position.y = odom_y_;
            odom_msg.pose.pose.position.z = 0.0;

            if (!std::isnan(t_imu.qx) && !std::isnan(t_imu.qy) &&
                !std::isnan(t_imu.qz) && !std::isnan(t_imu.qw)) {
                odom_msg.pose.pose.orientation.x = t_imu.qx;
                odom_msg.pose.pose.orientation.y = t_imu.qy;
                odom_msg.pose.pose.orientation.z = t_imu.qz;
                odom_msg.pose.pose.orientation.w = t_imu.qw;
            } else {
                odom_msg.pose.pose.orientation.x = 0.0;
                odom_msg.pose.pose.orientation.y = 0.0;
                odom_msg.pose.pose.orientation.z = std::sin(odom_yaw_ / 2.0);
                odom_msg.pose.pose.orientation.w = std::cos(odom_yaw_ / 2.0);
            }

            const double vx_source = std::isnan(t_imu.vx) ? vx : static_cast<double>(t_imu.vx);
            const double wz_source = std::isnan(t_imu.wz) ? wz : static_cast<double>(t_imu.wz);

            odom_msg.twist.twist.linear.x = vx_source;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.linear.z = 0.0;
            odom_msg.twist.twist.angular.x = 0.0;
            odom_msg.twist.twist.angular.y = 0.0;
            odom_msg.twist.twist.angular.z = wz_source;

            odom_msg.pose.covariance[0] = 1e-3;
            odom_msg.pose.covariance[7] = 1e-3;
            odom_msg.pose.covariance[14] = 1e-3;
            odom_msg.twist.covariance[0] = 1e-3;
            odom_msg.twist.covariance[7] = 1e-3;
            odom_msg.twist.covariance[35] = 1e-3;

            odom_pub_->publish(odom_msg);
        }

        // Publish GPS data if available
        if (gps_pub_ && pub_node_) {
            sensor_msgs::msg::NavSatFix gps_msg;
            gps_msg.header.stamp = now;
            gps_msg.header.frame_id = "gps";
            GPSTelemetry t_gps;
            {
                std::lock_guard<std::mutex> lk(this->telemetry_mutex_);
                t_gps = gps.getTelemetry();
            }
            if (!std::isnan(t_gps.lat) && !std::isnan(t_gps.lng)) {
                gps_msg.latitude = t_gps.lat;
                gps_msg.longitude = t_gps.lng;
                gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
            }
            gps_pub_->publish(gps_msg);
        }

        if (battery_pub_) {
            std::vector<float> cell_voltages;
            cell_voltages.reserve(4);
            auto consider_voltage = [&](float v) {
                if (std::isfinite(v) && std::fabs(v) > 1e-3) {
                    cell_voltages.emplace_back(v);
                }
            };
            consider_voltage(backleftWheel.telemetry.volt);
            consider_voltage(backrightWheel.telemetry.volt);
            consider_voltage(frontleftWheel.telemetry.volt);
            consider_voltage(frontrightWheel.telemetry.volt);

            sensor_msgs::msg::BatteryState battery_msg;
            battery_msg.header.stamp = now;
            battery_msg.header.frame_id = "base_link";
            battery_msg.present = !cell_voltages.empty();
            if (!cell_voltages.empty()) {
                double sum = 0.0;
                for (const auto &v : cell_voltages) {
                    sum += static_cast<double>(v);
                    battery_msg.cell_voltage.push_back(v);
                }
                battery_msg.voltage = sum / static_cast<double>(cell_voltages.size());
            } else {
                battery_msg.voltage = 0.0;
            }
            battery_msg.percentage = std::numeric_limits<float>::quiet_NaN();
            battery_msg.current = std::numeric_limits<float>::quiet_NaN();
            battery_msg.charge = std::numeric_limits<float>::quiet_NaN();
            battery_msg.capacity = std::numeric_limits<float>::quiet_NaN();
            battery_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
            battery_pub_->publish(battery_msg);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type AsinusHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration & period)
    {
       

        if (!serial_reader_) {
            RCLCPP_WARN(rclcpp::get_logger("AsinusHardwareInterface"), "Serial não inicializada");
            return hardware_interface::return_type::ERROR;
        }
        //std::cerr<<"debug commands: BL="<<backleftWheel.command<<" BR="<<backrightWheel.command<<std::endl;
       
        int cmd_bl = static_cast<int>(std::round(backleftWheel.command*10));
        int cmd_br = static_cast<int>(std::round(backrightWheel.command*10));

        std::string msg1 = "h|" + std::to_string(backleftWheel.telemetry.id) +
                        "|" + std::to_string(cmd_bl) + "|1\n";

        std::string msg2 = "h|" + std::to_string(backrightWheel.telemetry.id) +
                        "|" + std::to_string(cmd_br) + "|1\n";
        //std::cerr<<"bl raw command: "<<backleftWheel.command<<", br raw command: "<<backrightWheel.command<<std::endl;
        std::cerr<<"bl command: "<<cmd_bl<<", br command: "<<cmd_br<<std::endl;
        std::cerr<<"fl command: "<<frontleftWheel.command*100<<", fr command: "<<frontrightWheel.command*100<<std::endl;
        
        //std::cerr<<"debug msg1: "<<msg1;
        //std::cerr<<"debug msg2: "<<msg2;
        serial_reader_->write(std::vector<uint8_t>(msg1.begin(), msg1.end()));
        serial_reader_->write(std::vector<uint8_t>(msg2.begin(), msg2.end()));

        // --- Se for modelo 4x4 ---
        if (hardwareConfig.typeName == "asinus_four_wheel") {
            

            int cmd_fl = static_cast<int>(std::round(frontleftWheel.command*36));
            int cmd_fr = static_cast<int>(std::round(-frontrightWheel.command*36));

            std::string msg3 = "h|" +
                            std::to_string(frontrightWheel.telemetry.id) + "|" +
                            std::to_string(cmd_fr) + "|1\n";

            std::string msg4 = "h|" +
                            std::to_string(frontleftWheel.telemetry.id) + "|" +
                            std::to_string(cmd_fl) + "|1\n";

            serial_reader_->write(std::vector<uint8_t>(msg3.begin(), msg3.end()));
            serial_reader_->write(std::vector<uint8_t>(msg4.begin(), msg4.end()));
        }

        return hardware_interface::return_type::OK;
        }

    double AsinusHardwareInterface::normalize_angle(double angle)
    {
        constexpr double pi = 3.14159265358979323846;
        constexpr double two_pi = 2.0 * pi;
        while (angle > pi) {
            angle -= two_pi;
        }
        while (angle < -pi) {
            angle += two_pi;
        }
        return angle;
    }

    } // namespace asinus_hardware_interface

    PLUGINLIB_EXPORT_CLASS(asinus_hardware_interface::AsinusHardwareInterface, hardware_interface::SystemInterface)
