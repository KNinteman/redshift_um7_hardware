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
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com> (original code for UM6/7 in ROS1)
 *  \author     Alex Brown <rbirac@cox.net>		    (adapted to UM7)
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

#ifndef REDSHIFT_UM7_HARDWARE_HPP
#define REDSHIFT_UM7_HARDWARE_HPP

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "hardware_interface/sensor_interface.hpp"
#include "redshift_um7_hardware/visibility_control.h"
#include "redshift_um7_hardware/redshift_um7_comms.hpp"
#include "redshift_um7_hardware/redshift_um7_registers.hpp"
#include <serial/serial.h>

namespace redshift_um7_hardware
{
    namespace OutputAxisOptions
    {
        enum OutputAxisOption
        {
            DEFAULT, ENU, ROBOT_FRAME
        };
    }
    typedef OutputAxisOptions::OutputAxisOption OutputAxisOption;

    class RedshiftUm7Hardware : public hardware_interface::SensorInterface
    {
        struct Config  
        {
            std::string serial_port = "";
            int baud_rate = 57600;
            int update_rate = 0;
            std::string frame_id = "";
            bool mag_updates = true;
            bool quat_mode = true;
            bool zero_gyros = true;
            bool tf_ned_to_enu = false;
            bool orientation_in_robot_frame = false;
        };

        public:
            REDSHIFT_UM7_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo & info) override;

            REDSHIFT_UM7_HARDWARE_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            REDSHIFT_UM7_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;

            REDSHIFT_UM7_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State & previous_state) override;

            REDSHIFT_UM7_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;

            REDSHIFT_UM7_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;

            REDSHIFT_UM7_HARDWARE_PUBLIC
            hardware_interface::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

            REDSHIFT_UM7_HARDWARE_PUBLIC
                void configure_sensor();
            
            REDSHIFT_UM7_HARDWARE_PUBLIC
                void process_sensor_data(const redshift_um7_hardware::Registers& registers);
            
            REDSHIFT_UM7_HARDWARE_PUBLIC
                void process_ENU_orientation(const redshift_um7_hardware::Registers& registers);

            REDSHIFT_UM7_HARDWARE_PUBLIC
                void process_robot_frame_orientation(const redshift_um7_hardware::Registers& registers);

            REDSHIFT_UM7_HARDWARE_PUBLIC
                void process_default_orientation(const redshift_um7_hardware::Registers& registers);

            template<typename RegT>
            REDSHIFT_UM7_HARDWARE_PUBLIC
                void send_command(const redshift_um7_hardware::Accessor<RegT> & reg, std::string human_name);


        private:
            Config                                          conf_;
            OutputAxisOption                                axes_;
            sensor_msgs::msg::Imu                           imu_msg_;
            std::shared_ptr<redshift_um7_hardware::Comms>   sensor_;
            redshift_um7_hardware::Registers                registers_;
            serial::Serial                                  ser_;
            std::mutex                                      mutex_;
            const uint8_t TRIGGER_PACKET =                  DREG_EULER_PHI_THETA;

            std::vector<double>     hw_orientation_;
            std::vector<double>     hw_angular_velocity_;
            std::vector<double>     hw_linear_acceleration_;
            std::vector<double>     hw_magnetic_field_;
            std::vector<double>     hw_vector_;
    };
} // namespace redshift_um7_hardware

#endif // REDSHIFT_UM7_HARDWARE