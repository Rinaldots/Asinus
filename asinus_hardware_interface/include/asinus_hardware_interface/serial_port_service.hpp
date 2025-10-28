// Copyright 2023
// Robert Gruberski (Viola Robotics Sp. z o.o. Poland)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at:
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
#include <vector>
#include <functional>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"
#include "serial_port_protocol.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"



#define SERIAL_PORT_READ_BUF_SIZE 130

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

namespace asinus_hardware_interface
{

/**
 * @brief Serviço para comunicação serial assíncrona com o hardware.
 * 
 * Essa classe gerencia a conexão, leitura e escrita de dados via porta serial,
 * incluindo o despacho de mensagens binárias (TwoMotorWheelFeedback / FourMotorWheelFeedback)
 * e também linhas ASCII formatadas no protocolo ASINUS.
 */
class SerialPortService
{
public:
    SerialPortService() = default;

    /**
     * @brief Conecta a uma porta serial.
     * 
     * @param serial_device Caminho do dispositivo, ex: "/dev/ttyUSB0"
     * @param baud_rate Taxa de transmissão (ex: 115200)
     * @param timeout Tempo limite em ms
     * @return true se conectar com sucesso
     */
    bool connect(const std::string &serial_device, int baud_rate, int timeout);

    /**
     * @brief Fecha a conexão serial.
     * @return true se desconectado com sucesso
     */
    bool disconnect();

    /**
     * @brief Realiza leitura síncrona (bloqueante).
     */
    void read();

    /**
     * @brief Realiza leitura assíncrona (não bloqueante).
     */
    void asyncRead();

    /**
     * @brief Escreve dados na porta serial.
     * @param data Ponteiro para buffer
     * @param size Número de bytes
     * @return int Número de bytes enviados
     */
    int write(const char *data, const int &size);

    /**
     * @brief Associa callback para mensagens de feedback de dois motores.
     */
    void BindTwoMotorWheelFeedbackCallback(std::function<void(TwoMotorWheelFeedback)> fn);

    /**
     * @brief Associa callback para mensagens de feedback de quatro motores.
     */
    void BindFourMotorWheelFeedbackCallback(std::function<void(FourMotorWheelFeedback)> fn);

    /**
     * @brief Associa callback para receber linhas ASCII (protocolo ASINUS textual).
     * 
     * As linhas são divididas em tokens CSV.
     */
    void BindAsciiAsinusCallback(std::function<void(const std::vector<std::string>&)> fn);

    void initializePublishers(rclcpp::Node::SharedPtr node);

    void processBuffer(size_t bytes_transferred);

private:
    // Serviço IO da Boost
    boost::asio::io_service io_service;
    serial_port_ptr port;
    boost::shared_ptr<boost::asio::io_service::work> work_ptr_; // ADICIONADO
    boost::thread io_thread_; // ADICIONADO

    // Mutex de proteção
    boost::mutex mutex;

    // Cabeçalho e estado de frame
    uint16_t head_frame = 0;
    uint16_t msg_counter = 0;
    uint8_t msg_command = 0;

    char prev_byte = 0;
    char *p {};

    rclcpp::Node::SharedPtr node_;

    // Publishers ROS
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

    // Buffer de leitura
    char read_buf_raw[SERIAL_PORT_READ_BUF_SIZE] {};

    // Função de callback chamada a cada leitura
    void onReceive(const boost::system::error_code &error, size_t bytes_transferred);

    // Callbacks para mensagens binárias
    std::function<void(TwoMotorWheelFeedback)> twoMotorWheelFeedbackCallback;
    std::function<void(FourMotorWheelFeedback)> fourMotorWheelFeedbackCallback;

    // Estruturas de dados para feedback
    TwoMotorWheelFeedback twoMotorWheelFeedback {};
    FourMotorWheelFeedback fourMotorWheelFeedback {};

    // Callback para mensagens ASCII
    std::function<void(const std::vector<std::string>&)> asciiAsinusCallback;
};

} // namespace asinus_hardware_interface
