// Copyright 2023 Robert Gruberski
// (Viola Robotics Sp. z o.o. Poland)
//
// Modificado em 2025 por Rinaldo Tavares para adicionar publicadores de IMU e GPS
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

#include "asinus_hardware_interface/serial_port_service.hpp"

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>

// ROS 2 includes
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#define TWO_WHEEL
#define DEBUG

using namespace asinus_hardware_interface;

bool SerialPortService::connect(const std::string &serial_device, int baud_rate, int /*timeout*/)
{
    boost::system::error_code ec;

    if (port) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Port is already opened...");
        return false;
    }

    port = serial_port_ptr(new boost::asio::serial_port(io_service));
    port->open(serial_device, ec);

    if (ec) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"),
            "Connection to the %s failed..., error: %s",
            serial_device.c_str(), ec.message().c_str());
        return false;
    }

    port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    port->set_option(boost::asio::serial_port_base::character_size(8));
    port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::hardware));

    // ADICIONADO: Inicia a thread de I/O assíncrona
    work_ptr_.reset(new boost::asio::io_service::work(io_service));
    io_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
    asyncRead(); // Inicia a primeira leitura

    return true;
}

using namespace asinus_hardware_interface;


bool SerialPortService::disconnect()
{
    // 1. Para a thread de I/O primeiro
    work_ptr_.reset(); // Permite que io_service.run() saia
    if (io_thread_.joinable()) {
        io_thread_.join(); // Espera a thread terminar
    }

    // 2. AGORA adquire a trava (APENAS UMA VEZ) para acessar o 'port'
    boost::mutex::scoped_lock look(mutex);

    if (port) {
        port->cancel();
        port->close();
        port.reset();
    }

    io_service.stop();
    io_service.reset();

    return true;
}

void SerialPortService::initializePublishers(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    mag_pub_ = node_->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
    gps_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);

    RCLCPP_INFO(node_->get_logger(), "IMU e GPS publishers inicializados.");
}

void SerialPortService::read()
{
    boost::mutex::scoped_lock look(mutex);

    size_t bytes_transferred = port->read_some(boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE));
    #ifdef DEBUG
    RCLCPP_INFO(rclcpp::get_logger("SerialPortService"), "read() called, bytes_transferred=%zu", bytes_transferred);
    #endif
    static std::string rx_line_buffer;

#ifdef TWO_WHEEL
    for (unsigned int i = 0; i < bytes_transferred; ++i) {
        char c = (char)read_buf_raw[i];

        if (std::isprint(static_cast<unsigned char>(c)) || c == ',' || c == '\r' || c == '\n') {
            rx_line_buffer.push_back(c);

            if (c == '\n') {
                while (!rx_line_buffer.empty() && (rx_line_buffer.back() == '\n' || rx_line_buffer.back() == '\r')) {
                    rx_line_buffer.pop_back();
                }

                const std::string prefix = "ASINUS,";
                if (rx_line_buffer.size() >= prefix.size() &&
                    rx_line_buffer.compare(0, prefix.size(), prefix) == 0) {

                    std::vector<std::string> tokens;
                    std::string token;
                    std::istringstream iss(rx_line_buffer);
                    while (std::getline(iss, token, ',')) {
                        tokens.push_back(token);
                    }

                    if (tokens.size() >= 23) {
                        const std::string &ts = tokens[1];
                        const std::string &m0_id = tokens[2];
                        const std::string &m0_spd = tokens[3];
                        const std::string &m0_odom = tokens[4];
                        const std::string &m0_volt = tokens[5];
                        const std::string &m1_id = tokens[6];
                        const std::string &m1_spd = tokens[7];
                        const std::string &m1_odom = tokens[8];
                        const std::string &m1_volt = tokens[9];
                        const std::string &ax = tokens[10];
                        const std::string &ay = tokens[11];
                        const std::string &az = tokens[12];
                        const std::string &gx = tokens[13];
                        const std::string &gy = tokens[14];
                        const std::string &gz = tokens[15];
                        const std::string &mx = tokens[16];
                        const std::string &my = tokens[17];
                        const std::string &mz = tokens[18];
                        const std::string &temp = tokens[19];
                        const std::string &lat = tokens[20];
                        const std::string &lng = tokens[21];
                        const std::string &hdop = tokens[22];

                        twoMotorWheelFeedback.command1 = std::stoi(m0_id);
                        twoMotorWheelFeedback.backrightMotorSpeed = std::stof(m0_spd);
                        twoMotorWheelFeedback.backrightMotorEncoderCumulativeCount = std::stol(m0_odom);
                        twoMotorWheelFeedback.batteryVoltage = std::stof(m0_volt);
                        twoMotorWheelFeedback.command2 = std::stoi(m1_id);
                        twoMotorWheelFeedback.backleftMotorSpeed = std::stof(m1_spd);
                        twoMotorWheelFeedback.backleftMotorEncoderCumulativeCount = std::stol(m1_odom);

                        
                        if (node_) {
                            auto imu_msg = sensor_msgs::msg::Imu();
                            imu_msg.header.stamp = node_->now();
                            imu_msg.header.frame_id = "imu_link";

                            imu_msg.linear_acceleration.x = std::stof(ax);
                            imu_msg.linear_acceleration.y = std::stof(ay);
                            imu_msg.linear_acceleration.z = std::stof(az);

                            imu_msg.angular_velocity.x = std::stof(gx);
                            imu_msg.angular_velocity.y = std::stof(gy);
                            imu_msg.angular_velocity.z = std::stof(gz);
                            imu_pub_->publish(imu_msg);

                            auto mag_msg = sensor_msgs::msg::MagneticField();
                            mag_msg.header = imu_msg.header;
                            mag_msg.magnetic_field.x = std::stof(mx);
                            mag_msg.magnetic_field.y = std::stof(my);
                            mag_msg.magnetic_field.z = std::stof(mz);
                            mag_pub_->publish(mag_msg);

                            auto gps_msg = sensor_msgs::msg::NavSatFix();
                            gps_msg.header.stamp = imu_msg.header.stamp;
                            gps_msg.header.frame_id = "gps_link";
                            gps_msg.latitude = std::stod(lat);
                            gps_msg.longitude = std::stod(lng);
                            gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                            gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
                            // exemplo em C++
                            double hdop_val = std::stod(hdop); // converte string para double
                            gps_msg.position_covariance[0] = pow(hdop_val * 2.5, 2); // var_x
                            gps_msg.position_covariance[4] = pow(hdop_val * 2.5, 2); // var_y
                            gps_msg.position_covariance[8] = pow(hdop_val * 2.5, 2); // var_z

                            gps_msg.position_covariance_type =  sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

                            gps_pub_->publish(gps_msg);
                        }
                    } else {
                        RCLCPP_WARN(rclcpp::get_logger("SerialPortService"),
                            "ASINUS line has unexpected token count: %zu", tokens.size());
                    }
                }

                rx_line_buffer.clear();
            }
            continue;
        }

        prev_byte = read_buf_raw[i];
    }
#endif
}

void SerialPortService::processBuffer(size_t bytes_transferred)
{
    #ifdef DEBUG
    RCLCPP_INFO(rclcpp::get_logger("SerialPortService"), "processBuffer() called, bytes_transferred=%zu", bytes_transferred);
    #endif

    static std::string rx_line_buffer;

#ifdef TWO_WHEEL
    for (unsigned int i = 0; i < bytes_transferred; ++i) {
        char c = (char)read_buf_raw[i];

        if (std::isprint(static_cast<unsigned char>(c)) || c == ',' || c == '\r' || c == '\n') {
            rx_line_buffer.push_back(c);

            if (c == '\n') {
                while (!rx_line_buffer.empty() && (rx_line_buffer.back() == '\n' || rx_line_buffer.back() == '\r')) {
                    rx_line_buffer.pop_back();
                }

                const std::string prefix = "ASINUS,";
                if (rx_line_buffer.size() >= prefix.size() &&
                    rx_line_buffer.compare(0, prefix.size(), prefix) == 0) {

                    std::vector<std::string> tokens;
                    std::string token;
                    std::istringstream iss(rx_line_buffer);
                    while (std::getline(iss, token, ',')) {
                        tokens.push_back(token);
                    }

                    if (tokens.size() >= 23) {
                        const std::string &ts = tokens[1];
                        const std::string &m0_id = tokens[2];
                        const std::string &m0_spd = tokens[3];
                        const std::string &m0_odom = tokens[4];
                        const std::string &m0_volt = tokens[5];
                        const std::string &m1_id = tokens[6];
                        const std::string &m1_spd = tokens[7];
                        const std::string &m1_odom = tokens[8];
                        const std::string &m1_volt = tokens[9];
                        const std::string &ax = tokens[10];
                        const std::string &ay = tokens[11];
                        const std::string &az = tokens[12];
                        const std::string &gx = tokens[13];
                        const std::string &gy = tokens[14];
                        const std::string &gz = tokens[15];
                        const std::string &mx = tokens[16];
                        const std::string &my = tokens[17];
                        const std::string &mz = tokens[18];
                        const std::string &temp = tokens[19];
                        const std::string &lat = tokens[20];
                        const std::string &lng = tokens[21];
                        const std::string &hdop = tokens[22];

                        twoMotorWheelFeedback.command1 = std::stoi(m0_id);
                        twoMotorWheelFeedback.backrightMotorSpeed = std::stof(m0_spd);
                        twoMotorWheelFeedback.backrightMotorEncoderCumulativeCount = std::stol(m0_odom);
                        twoMotorWheelFeedback.batteryVoltage = std::stof(m0_volt);
                        twoMotorWheelFeedback.command2 = std::stoi(m1_id);
                        twoMotorWheelFeedback.backleftMotorSpeed = std::stof(m1_spd);
                        twoMotorWheelFeedback.backleftMotorEncoderCumulativeCount = std::stol(m1_odom);

                        
                        if (node_) {
                            auto imu_msg = sensor_msgs::msg::Imu();
                            imu_msg.header.stamp = node_->now();
                            imu_msg.header.frame_id = "imu_link";

                            imu_msg.linear_acceleration.x = std::stof(ax);
                            imu_msg.linear_acceleration.y = std::stof(ay);
                            imu_msg.linear_acceleration.z = std::stof(az);

                            imu_msg.angular_velocity.x = std::stof(gx);
                            imu_msg.angular_velocity.y = std::stof(gy);
                            imu_msg.angular_velocity.z = std::stof(gz);
                            imu_pub_->publish(imu_msg);

                            auto mag_msg = sensor_msgs::msg::MagneticField();
                            mag_msg.header = imu_msg.header;
                            mag_msg.magnetic_field.x = std::stof(mx);
                            mag_msg.magnetic_field.y = std::stof(my);
                            mag_msg.magnetic_field.z = std::stof(mz);
                            mag_pub_->publish(mag_msg);

                            auto gps_msg = sensor_msgs::msg::NavSatFix();
                            gps_msg.header.stamp = imu_msg.header.stamp;
                            gps_msg.header.frame_id = "gps_link";
                            gps_msg.latitude = std::stod(lat);
                            gps_msg.longitude = std::stod(lng);
                            gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                            gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
                            // exemplo em C++
                            double hdop_val = std::stod(hdop); // converte string para double
                            gps_msg.position_covariance[0] = pow(hdop_val * 2.5, 2); // var_x
                            gps_msg.position_covariance[4] = pow(hdop_val * 2.5, 2); // var_y
                            gps_msg.position_covariance[8] = pow(hdop_val * 2.5, 2); // var_z

                            gps_msg.position_covariance_type =  sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

                            gps_pub_->publish(gps_msg);
                        }
                    } else {
                        RCLCPP_WARN(rclcpp::get_logger("SerialPortService"),
                            "ASINUS line has unexpected token count: %zu", tokens.size());
                    }
                }

                rx_line_buffer.clear();
            }
            continue;
        }

        prev_byte = read_buf_raw[i];
    }
#endif
}


void SerialPortService::asyncRead()
{
    if (port.get() == nullptr || !port->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Port is already closed...");
        return;
    }

    port->async_read_some(
        boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE),
        boost::bind(&SerialPortService::onReceive,
            this, boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
}


void SerialPortService::onReceive(const boost::system::error_code& ec, size_t bytes_transferred)
{
    if (!ec)
    {
        // Processa os dados que acabaram de chegar
        processBuffer(bytes_transferred);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "onReceive error: %s", ec.message().c_str());
    }
    
    // Se a porta ainda estiver aberta, agenda a próxima leitura
    if (port && port->is_open()) {
        asyncRead();
    }
}

int SerialPortService::write(const char * message, const int & size)
{
    boost::system::error_code ec;

    if (port.get() == nullptr || !port->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Port is already closed...");
        return 0;
    }

    if (size == 0) {
        return 0;
    }

    return port->write_some(boost::asio::buffer(message, size), ec);
}

#ifdef TWO_WHEEL
void SerialPortService::BindTwoMotorWheelFeedbackCallback(std::function<void(TwoMotorWheelFeedback)> fn) {
    twoMotorWheelFeedbackCallback = fn;
}
#endif

#ifdef FOUR_WHEEL
void SerialPortService::BindFourMotorWheelFeedbackCallback(std::function<void(FourMotorWheelFeedback)> fn) {
    FourMotorWheelFeedbackCallback = fn;
}
#endif
