// Copyright (c) 2023, Clearpath Robotics, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Clearpath Robotics, Inc nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 *
 *  \file
 *  \brief      Main entry point for UM7 driver. Handles serial connection
 *              details, as well as all ROS message stuffing, parameters,
 *              topics, etc.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com> (original code for UM6)
 *  \author     Alex Brown <rbirac@cox.net> (adapted to UM7)
 *  \copyright  Copyright (c) 2015, Alex Brown.
 *  \author     Hilary Luo <hluo@clearpathrobotics.com> (updated to ROS 2 and combined UM6 and UM7)
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 */

/**  Modifications by 
* \author   Kars Ninteman <k.ninteman@st.hanze.nl>
* \date     08/07/2024
*
* Description of modifications:
* - Rewritten as a hardware interface for ROS2_control.
*/

#include "redshift_um7_hardware/redshift_um7_hardware.hpp"
#include <vector>

namespace redshift_um7_hardware
{
    /**
    * @brief Initialises the hardware with the provided configuration.
    *
    * This method initialises the hardware with the configuration provided in the hardware info parameters.
    * It calls the base class's on_init method and checks for success.
    * It extracts hardware parameters from the hardware info object and initialises internal
    * variables. It initialises hardware state vectors with NaNs and returns a success status.
    *
    * @param info The hardware info object containing configuration parameters.
    * @return hardware_interface::CallbackReturn The initialisation status.
    */
    hardware_interface::CallbackReturn RedshiftUm7Hardware::on_init(const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Extract hardware parameters from info
        try {
            conf_.serial_port = info_.hardware_parameters["serial_port"];
            conf_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
            conf_.update_rate = std::stoi(info_.hardware_parameters["update_rate"]);
            conf_.frame_id = info_.hardware_parameters["frame_id"];
            conf_.mag_updates = (info_.hardware_parameters["mag_updates"] == "true");
            conf_.quat_mode = (info_.hardware_parameters["quat_mode"] == "true");
            conf_.zero_gyros = (info_.hardware_parameters["zero_gyros"] == "true");
            conf_.tf_ned_to_enu = (info_.hardware_parameters["tf_ned_to_enu"] == "true"); 
            conf_.orientation_in_robot_frame = (info_.hardware_parameters["orientation_in_robot_frame"] == "true"); 
        } catch (const std::out_of_range& e) {
            RCLCPP_ERROR(rclcpp::get_logger("RedshiftUm7Hardware"), "Hardware parameter missing: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        } catch (const std::invalid_argument& e) {
            RCLCPP_ERROR(rclcpp::get_logger("RedshiftUm7Hardware"), "Invalid parameter value: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialise hardware state vectors with NaNs
        hw_orientation_.resize(4, std::numeric_limits<double>::quiet_NaN());
        hw_angular_velocity_.resize(3, std::numeric_limits<double>::quiet_NaN());
        hw_linear_acceleration_.resize(3, std::numeric_limits<double>::quiet_NaN());
        hw_magnetic_field_.resize(3, std::numeric_limits<double>::quiet_NaN());
        hw_vector_.resize(3, std::numeric_limits<double>::quiet_NaN());

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    /**
    * @brief Exports state interfaces for sensor data.
    *
    * This method exports state interfaces for sensor data. 
    *
    * @return std::vector<hardware_interface::StateInterface> A vector containing state interfaces
    * for sensor data.
    */
    std::vector<hardware_interface::StateInterface> RedshiftUm7Hardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Export sensor state interfaces
        const std::string& sensor_name = info_.sensors[0].name;

        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "orientation.x", &hw_orientation_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "orientation.y", &hw_orientation_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "orientation.z", &hw_orientation_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "orientation.w", &hw_orientation_[3]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "angular_velocity.x", &hw_angular_velocity_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "angular_velocity.y", &hw_angular_velocity_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "angular_velocity.z", &hw_angular_velocity_[2]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "linear_acceleration.x", &hw_linear_acceleration_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "linear_acceleration.y", &hw_linear_acceleration_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "linear_acceleration.z", &hw_linear_acceleration_[2]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "magnetic_field.x", &hw_magnetic_field_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "magnetic_field.y", &hw_magnetic_field_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "magnetic_field.z", &hw_magnetic_field_[2]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "vector.x", &hw_vector_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "vector.y", &hw_vector_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "vector.z", &hw_vector_[2]));

        return state_interfaces;
    }

    /**
    * @brief Configures the hardware based on provided parameters.
    *
    * This method configures the hardware based on the parameters provided in the configuration object.
    * It sets up the serial port with the specified parameters and opens the port.
    *
    * @param previous_state The previous state of the hardware lifecycle.
    * @return hardware_interface::CallbackReturn The configuration status, indicating whether the
    * configuration was successful or if an error occurred.
    */
    hardware_interface::CallbackReturn RedshiftUm7Hardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        try {
            axes_ = OutputAxisOptions::DEFAULT;
            ser_.setPort(conf_.serial_port);
            ser_.setBaudrate(conf_.baud_rate);
            serial::Timeout timeout = serial::Timeout(100, 100, 0, 100, 0);
            ser_.setTimeout(timeout);
            ser_.open();

            if (conf_.tf_ned_to_enu && conf_.orientation_in_robot_frame) {
                RCLCPP_ERROR(rclcpp::get_logger("RedshiftUm7Hardware"), "Requested IMU data in two separate frames.");
            } else if (conf_.tf_ned_to_enu) {
                axes_ = OutputAxisOptions::ENU;
            } else if (conf_.orientation_in_robot_frame) {
                axes_ = OutputAxisOptions::ROBOT_FRAME;
            }
        } catch (serial::IOException &e) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (ser_.isOpen()) {
            // If the port is open, log a message indicating successful initialisation
            RCLCPP_INFO_STREAM(rclcpp::get_logger("RedshiftUm7Hardware"), "Serial Port " << conf_.serial_port << " initialised.");
        } else {
            // If the port is not open, log a message indicating failure and shutdown the ROS node
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("RedshiftUm7Hardware"), "Serial Port " << conf_.serial_port << " is not open.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    } 

    /**
    * @brief Cleans up resources and performs finalization tasks.
    *
    * This method closes the serial port.
    *
    * @param previous_state The previous lifecycle state of the hardware component.
    * @return CallbackReturn Returns a status indicating the success or failure of the cleanup process.
    */
    hardware_interface::CallbackReturn RedshiftUm7Hardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
        try {
            if (ser_.isOpen()) {
                ser_.close();
                RCLCPP_INFO_STREAM(rclcpp::get_logger("RedshiftUm7Hardware"), "Serial Port " << conf_.serial_port << " closed.");
            } else {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("RedshiftUm7Hardware"), "Serial Port " << conf_.serial_port << " was not open.");
            }
        } catch (const serial::IOException &e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("RedshiftUm7Hardware"), "Error while closing serial port: " << e.what());
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("RedshiftUm7Hardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
    * @brief Configures the sensor with the provided settings.
    *
    * This method configures the sensor with the provided settings such as communication rate,
    * update rate, broadcast rate, etc. It constructs command packets based on the settings and
    * sends them to the sensor, waiting for acknowledgment after each command. If any command fails
    * to be acknowledged, a runtime error is thrown. 
    */
    void RedshiftUm7Hardware::configure_sensor()
    {
        redshift_um7_hardware::Registers r;

        uint32_t comm_reg = (BAUD_115200 << COM_BAUD_START);
        r.communication.set(0, comm_reg);
        if (!sensor_->sendWaitAck(r.comrate2)) {
            throw std::runtime_error("Unable to set CREG_COM_SETTINGS.");
        }

        int rate = conf_.update_rate;

        uint32_t rate_bits = static_cast<uint32_t>(rate);
        uint32_t raw_rate = (rate_bits << RATE2_ALL_RAW_START);
        r.comrate2.set(0, raw_rate);
        if (!sensor_->sendWaitAck(r.comrate2)) {
            throw std::runtime_error("Unable to set CREG_COM_RATES2.");
        }

        uint32_t proc_rate = (rate_bits << RATE4_ALL_PROC_START);
        r.comrate4.set(0, proc_rate);
        if (!sensor_->sendWaitAck(r.comrate4)) {
            throw std::runtime_error("Unable to set CREG_COM_RATES4.");
        }

        uint32_t misc_rate = (rate_bits << RATE5_EULER_START) | (rate_bits << RATE5_QUAT_START);
        r.comrate5.set(0, misc_rate);
        if (!sensor_->sendWaitAck(r.comrate5)) {
            throw std::runtime_error("Unable to set CREG_COM_RATES5.");
        }

        uint32_t health_rate = (5 << RATE6_HEALTH_START);  // 5 gives 2 hz rate
        r.comrate6.set(0, health_rate);
        if (!sensor_->sendWaitAck(r.comrate6)) {
            throw std::runtime_error("Unable to set CREG_COM_RATES6.");
        }

        uint32_t misc_config_reg = 0; 

        if (conf_.mag_updates) {
            misc_config_reg |= MAG_UPDATES_ENABLED;
        }

        if (conf_.quat_mode) {
            misc_config_reg |= QUATERNION_MODE_ENABLED;
        }

        r.misc_config.set(0, misc_config_reg);
        if (!sensor_->sendWaitAck(r.misc_config)) {
            throw std::runtime_error("Unable to set CREG_MISC_SETTINGS.");
        }

        if (conf_.zero_gyros) {
            send_command(r.cmd_zero_gyros, "zero gyroscopes");
        }
    }

    /**
    * @brief Activates the hardware for operation.
    *
    * This method activates the hardware for operation by resetting the sensor communication object,
    * creating a new communication object with the serial port, and configuring the sensor using the
    * provided settings.
    *
    * @param previous_state The previous lifecycle state (unused).
    * @return hardware_interface::CallbackReturn The activation status, indicating whether the activation
    * was successful or if an error occurred.
    */
    hardware_interface::CallbackReturn RedshiftUm7Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (!ser_.isOpen()) {
            RCLCPP_ERROR(
                rclcpp::get_logger("RedshiftUm7Hardware"), 
                "Serial port is not open. Unable to activate.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        try {
            sensor_.reset(new redshift_um7_hardware::Comms(&ser_));
            configure_sensor();
        } catch (const serial::IOException &e) {
            RCLCPP_ERROR_STREAM(
                rclcpp::get_logger("RedshiftUm7Hardware"), 
                "Error while configuring hardware: " << e.what()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Deactivates the hardware components.
     * 
     * Currently not implemented.
     * 
     */
    hardware_interface::CallbackReturn RedshiftUm7Hardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("RedshiftUm7Hardware"), "Deactivating ...please wait...");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("RedshiftUm7Hardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
    * @brief Sends a command to the UM7 device.
    *
    * This method generalizes sending a command to the UM7 device.
    *
    * @tparam RegT The type of register to which the command is sent.
    * @param reg The register accessor containing the command to be sent.
    * @param human_name A human-readable name for the command (used for error reporting).
    * @throws std::runtime_error If the command to the device fails.
    */
    template<typename RegT>
    void RedshiftUm7Hardware::send_command(const redshift_um7_hardware::Accessor<RegT> & reg, std::string human_name)
    {
        if (!sensor_->sendWaitAck(reg)) {
            throw std::runtime_error(human_name + " command to device failed.");
        }
    }

    /**
    * @brief Processes sensor data based on the specified output axis option.
    *
    * This method processes sensor data based on the specified output axis option.
    *
    * @param registers The sensor registers containing the data to be processed.
    */
    void RedshiftUm7Hardware::process_sensor_data(const redshift_um7_hardware::Registers& registers)
    {
        switch (axes_) {
            case OutputAxisOptions::ENU:
                process_ENU_orientation(registers);
                break;
            case OutputAxisOptions::ROBOT_FRAME:
                process_robot_frame_orientation(registers);
                break;
            case OutputAxisOptions::DEFAULT:
                process_default_orientation(registers);
                break;
            default:
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("RedshiftUm7Hardware"), "OuputAxes enum value invalid");
        }
    }

    /**
    * @brief Processes orientation data for the ENU axis option.
    *
    * This method processes orientation data received from the sensor for the East-North-Up (ENU)
    * axis option. 
    *
    * @param registers The sensor registers containing the raw data to be processed.
    */
    void RedshiftUm7Hardware::process_ENU_orientation(const redshift_um7_hardware::Registers& registers)
    {
        // Process orientation data for ENU axis option
        hw_orientation_[0] = registers.quat.get_scaled(1);
        hw_orientation_[1] = -registers.quat.get_scaled(3);
        hw_orientation_[2] = registers.quat.get_scaled(0);
        hw_orientation_[3] = registers.quat.get_scaled(2);
      
        // body-fixed frame
        hw_angular_velocity_[0] = registers.gyro.get_scaled(0);
        hw_angular_velocity_[1] = -registers.gyro.get_scaled(1);
        hw_angular_velocity_[2] = -registers.gyro.get_scaled(2);
       
        // body-fixed frame
        hw_linear_acceleration_[0] = registers.accel.get_scaled(0);
        hw_linear_acceleration_[1] = -registers.accel.get_scaled(1);
        hw_linear_acceleration_[2] = -registers.accel.get_scaled(2);
       
        // Magnetometer.  transform to ROS axes
        hw_magnetic_field_[0] = registers.mag.get_scaled(1);
        hw_magnetic_field_[1] = registers.mag.get_scaled(0);
        hw_magnetic_field_[2] = -registers.mag.get_scaled(2);

        // Euler attitudes.  transform to ROS axes
        hw_vector_[0] = registers.euler.get_scaled(1);
        hw_vector_[1] = registers.euler.get_scaled(0);
        hw_vector_[2] = -registers.euler.get_scaled(2);
    }

    /**
    * @brief Processes orientation data for the robot frame axis option.
    *
    * This method processes orientation data received from the sensor for the robot frame
    * axis option. 
    *
    * @param registers The sensor registers containing the raw data to be processed.
    */
    void RedshiftUm7Hardware::process_robot_frame_orientation(const redshift_um7_hardware::Registers& registers)
    {
        // Process orientation data for robot frame axis option
        hw_orientation_[0] = -registers.quat.get_scaled(1);
        hw_orientation_[1] = registers.quat.get_scaled(2);
        hw_orientation_[2] = registers.quat.get_scaled(3);
        hw_orientation_[3] = -registers.quat.get_scaled(0);
       
        // body-fixed frame
        hw_angular_velocity_[0] = registers.gyro.get_scaled(0);
        hw_angular_velocity_[1] = -registers.gyro.get_scaled(1);
        hw_angular_velocity_[2] = -registers.gyro.get_scaled(2);
   
        // body-fixed frame
        hw_linear_acceleration_[0] = registers.accel.get_scaled(0);
        hw_linear_acceleration_[1] = -registers.accel.get_scaled(1);
        hw_linear_acceleration_[2] = -registers.accel.get_scaled(2);

        // Magnetometer.  transform to ROS axes
        hw_magnetic_field_[0] = registers.mag.get_scaled(0);
        hw_magnetic_field_[1] = -registers.mag.get_scaled(1);
        hw_magnetic_field_[2] = -registers.mag.get_scaled(2);
    
        // Euler attitudes.  transform to ROS axes
        hw_vector_[0] = registers.euler.get_scaled(0);
        hw_vector_[1] = -registers.euler.get_scaled(1);
        hw_vector_[2] = -registers.euler.get_scaled(2);
    }

    /**
    * @brief Processes orientation data for the robot frame axis option.
    *
    * This method processes orientation data received from the sensor for the default
    * axis option.
    *
    * @param registers The sensor registers containing the raw data to be processed.
    */
    void RedshiftUm7Hardware::process_default_orientation(const redshift_um7_hardware::Registers& registers)
    {
        // Process orientation data for default axis option
        hw_orientation_[0] = registers.quat.get_scaled(1);
        hw_orientation_[1] = registers.quat.get_scaled(2);
        hw_orientation_[2] = registers.quat.get_scaled(3);
        hw_orientation_[3] = registers.quat.get_scaled(0);

        hw_angular_velocity_[0] = registers.gyro.get_scaled(0);
        hw_angular_velocity_[1] = registers.gyro.get_scaled(1);
        hw_angular_velocity_[2] = registers.gyro.get_scaled(2);
    
        hw_linear_acceleration_[0] = registers.accel.get_scaled(0);
        hw_linear_acceleration_[1] = registers.accel.get_scaled(1);
        hw_linear_acceleration_[2] = registers.accel.get_scaled(2);
 
        // Magnetometer.  transform to ROS axes
        hw_magnetic_field_[0] = registers.mag.get_scaled(0);
        hw_magnetic_field_[1] = registers.mag.get_scaled(1);
        hw_magnetic_field_[2] = registers.mag.get_scaled(2);
     
        // Euler attitudes.  transform to ROS axes
        hw_vector_[0] = registers.euler.get_scaled(0);
        hw_vector_[1] = registers.euler.get_scaled(1);
        hw_vector_[2] = registers.euler.get_scaled(2);
    }

    /**
    * @brief Reads data from the sensor and processes it.
    *
    * This method reads data from the sensor connected via the serial port.
    *
    * @param time The current time.
    * @param period The duration between reads.
    * @return hardware_interface::return_type The status of the read operation, indicating whether it was
    * successful or if an error occurred.
    */
    hardware_interface::return_type RedshiftUm7Hardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        if (!ser_.isOpen()) {
            RCLCPP_ERROR(
                rclcpp::get_logger("RedshiftUm7Hardware"),
                "Serial port is not open. Unable to read."
            );
            return hardware_interface::return_type::ERROR;
        }

        int16_t input = 0;
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            input = sensor_->receive(&registers_);   
        }

        if (input == TRIGGER_PACKET) {
            process_sensor_data(registers_);
        }
        return hardware_interface::return_type::OK;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(redshift_um7_hardware::RedshiftUm7Hardware, hardware_interface::SensorInterface)