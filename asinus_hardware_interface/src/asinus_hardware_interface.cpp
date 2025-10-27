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

namespace asinus_hardware_interface
{
    hardware_interface::CallbackReturn AsinusHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hardwareConfig.leftWheelJointName = info.hardware_parameters.at("left_wheel_joint_name");
        hardwareConfig.rightWheelJointName = info.hardware_parameters.at("right_wheel_joint_name");
        hardwareConfig.loopRate = std::stof(info.hardware_parameters.at("loop_rate"));
        // hardwareConfig.encoderTicksPerRevolution = std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution"));

        serialPortConfig.device = info.hardware_parameters.at("device");
        serialPortConfig.baudRate = std::stoi(info.hardware_parameters.at("baud_rate"));
        serialPortConfig.timeout = std::stoi(info.hardware_parameters.at("timeout"));

        frontleftWheel = MotorWheel(info.hardware_parameters.at("left_wheel_joint_name"), 
                                std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution")));
        frontrightWheel = MotorWheel(info.hardware_parameters.at("right_wheel_joint_name"), 
                                std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution")));
        backleftWheel = MotorWheel(info.hardware_parameters.at("left_wheel_joint_name"), 
                                std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution")));
        backrightWheel = MotorWheel(info.hardware_parameters.at("right_wheel_joint_name"), 
                                std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution")));

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

        state_interfaces.emplace_back(hardware_interface::StateInterface(frontleftWheel.name, hardware_interface::HW_IF_POSITION, &frontleftWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(frontleftWheel.name, hardware_interface::HW_IF_VELOCITY, &frontleftWheel.velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(frontrightWheel.name, hardware_interface::HW_IF_POSITION, &frontrightWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(frontrightWheel.name, hardware_interface::HW_IF_VELOCITY, &frontrightWheel.velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(backleftWheel.name, hardware_interface::HW_IF_POSITION, &backleftWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(backleftWheel.name, hardware_interface::HW_IF_VELOCITY, &backleftWheel.velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(backrightWheel.name, hardware_interface::HW_IF_POSITION, &backrightWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(backrightWheel.name, hardware_interface::HW_IF_VELOCITY, &backrightWheel.velocity));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> AsinusHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(frontleftWheel.name, hardware_interface::HW_IF_VELOCITY, &frontleftWheel.command));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(frontrightWheel.name, hardware_interface::HW_IF_VELOCITY, &frontrightWheel.command));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(backleftWheel.name, hardware_interface::HW_IF_VELOCITY, &backleftWheel.command));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(backrightWheel.name, hardware_interface::HW_IF_VELOCITY, &backrightWheel.command));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn AsinusHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("AsinusHardwareInterface"), "Configuring... please wait a moment...");

        if (!serialPortService.connect(serialPortConfig.device, serialPortConfig.baudRate, serialPortConfig.timeout))
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        serialPortService.BindMotorWheelFeedbackCallback(
            std::bind(&AsinusHardwareInterface::motorWheelFeedbackCallback, this, std::placeholders::_1)
        );

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn AsinusHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("AsinusHardwareInterface"), "Cleaning up... please wait a moment...");

        if (!serialPortService.disconnect())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn AsinusHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        // TODO: add some logic
        RCLCPP_INFO(rclcpp::get_logger("AsinusHardwareInterface"), "Activating... please wait a moment...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn AsinusHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // TODO: add some logic
        RCLCPP_INFO(rclcpp::get_logger("AsinusHardwareInterface"), "Deactivating... please wait a moment...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type AsinusHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration & period)
    {
        serialPortService.read();

        double frontleftlastPosition = frontleftWheel.position;
        frontleftWheel.position = frontleftWheel.calculateEncoderAngle();
        frontleftWheel.velocity = (frontleftWheel.position - frontleftlastPosition) / period.seconds();

        double frontrightlastPosition = frontrightWheel.position;
        frontrightWheel.position = frontrightWheel.calculateEncoderAngle();
        frontrightWheel.velocity = (frontrightWheel.position - frontrightlastPosition) / period.seconds();

        double backleftlastPosition = backleftWheel.position;
        backleftWheel.position = backleftWheel.calculateEncoderAngle();
        backleftWheel.velocity = (backleftWheel.position - backleftlastPosition) / period.seconds();

        double backrightlastPosition = backrightWheel.position;
        backrightWheel.position = backrightWheel.calculateEncoderAngle();
        backrightWheel.velocity = (backrightWheel.position - backrightlastPosition) / period.seconds();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type AsinusHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        MotorWheelDriveControl motorWheelDriveControl;

        const double left_speed = (frontleftWheel.command + backleftWheel.command) / 2.0;
        const double right_speed = (frontrightWheel.command + backrightWheel.command) / 2.0;

        const double speed = ((left_speed / 0.10472) + (right_speed / 0.10472)) / 2.0;
        const double steer = ((left_speed / 0.10472) - speed) * 2.0;

        motorWheelDriveControl.speed = (int16_t) (speed);
        motorWheelDriveControl.steer = (int16_t) (steer);
        motorWheelDriveControl.wStateMaster = 32;
        motorWheelDriveControl.wStateSlave = 0;
        
        uint8_t* p = (uint8_t*)&motorWheelDriveControl;
        uint8_t checksum = 0;
        for (size_t i = 0; i < sizeof(MotorWheelDriveControl) - 1; i++) {
            checksum ^= p[i];
        }
        motorWheelDriveControl.checksum = checksum;

        serialPortService.write((const char*) &motorWheelDriveControl, sizeof(MotorWheelDriveControl));

        return hardware_interface::return_type::OK;
    }

    void AsinusHardwareInterface::motorWheelFeedbackCallback(MotorWheelFeedback motorWheelFeedback) 
    {
        frontleftWheel.updateEncoderTicks(motorWheelFeedback.frontleftMotorEncoderCumulativeCount);
        frontrightWheel.updateEncoderTicks(motorWheelFeedback.frontrightMotorEncoderCumulativeCount);
        backleftWheel.updateEncoderTicks(motorWheelFeedback.backleftMotorEncoderCumulativeCount);
        backrightWheel.updateEncoderTicks(motorWheelFeedback.backrightMotorEncoderCumulativeCount);
    }
}

PLUGINLIB_EXPORT_CLASS(asinus_hardware_interface::AsinusHardwareInterface, hardware_interface::SystemInterface)
