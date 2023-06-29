#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */



class FollowerDroneController : public rclcpp::Node
{
public:
	FollowerDroneController() : Node("follower_drone_controller")
	{

		follower_drone_controller_publisher_ = this->create_publisher<OffboardControlMode>("drone_follower/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("drone_follower/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("drone_follower/fmu/in/vehicle_command", 10);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        main_vehicle_local_position_subscriber_ = this->create_subscription<VehicleLocalPosition>("drone_camera/fmu/out/vehicle_local_position",qos,
            std::bind(&FollowerDroneController::mainVehicleLocalPositionCallback, this, std::placeholders::_1));
        follower_vehicle_local_position_subscriber_ = this->create_subscription<VehicleLocalPosition>("drone_follower/fmu/out/vehicle_local_position",qos,
            std::bind(&FollowerDroneController::followerVehicleLocalPositionCallback, this, std::placeholders::_1)); 

        offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_follower_drone_controller();
			publish_trajectory_setpoint(main_drone_position_, follower_drone_position_);

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

   

    void arm();
    void disarm();
    void mainVehicleLocalPositionCallback(const VehicleLocalPosition::SharedPtr msg);
	void followerVehicleLocalPositionCallback(const VehicleLocalPosition::SharedPtr msg);
private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr follower_drone_controller_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr main_vehicle_local_position_subscriber_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr follower_vehicle_local_position_subscriber_;
    std::atomic<uint64_t> timestamp_;  //!< common synced timestamped
    uint64_t offboard_setpoint_counter_;  //!< counter for the number of setpoints sent
    VehicleLocalPosition::SharedPtr main_drone_position_; 
	VehicleLocalPosition::SharedPtr follower_drone_position_; 

    void publish_follower_drone_controller();
    void publish_trajectory_setpoint(const VehicleLocalPosition::SharedPtr main_drone_position, const VehicleLocalPosition::SharedPtr follower_drone_position);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void FollowerDroneController::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void FollowerDroneController::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void FollowerDroneController::publish_follower_drone_controller()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    follower_drone_controller_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */

void FollowerDroneController::publish_trajectory_setpoint(const VehicleLocalPosition::SharedPtr main_drone_position, const VehicleLocalPosition::SharedPtr follower_drone_position)
{
	const double target_distance = 0.5;  // Desired distance to maintain between drones

    // Calculate the vector between the drones
    double dx = main_drone_position->x - follower_drone_position->x;
    double dy = main_drone_position->y - follower_drone_position->y;
    double dz = main_drone_position->z - follower_drone_position->z;

    // Calculate the distance between the drones
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Calculate the desired velocity vector
    double vx = dx * target_distance / distance;
    double vy = dy * target_distance / distance;
    double vz = dz * target_distance / distance;

    // Calculate the desired acceleration vector
    double ax = (vx - main_drone_position_->vx) * 2.0;  // Adjust the scaling factor as needed
    double ay = (vy - main_drone_position_->vy) * 2.0;
    double az = (vz - main_drone_position_->vz) * 2.0;
    TrajectorySetpoint msg{};
	msg.position = {(float)main_drone_position->x, (float)main_drone_position->y, (float)main_drone_position->z};
	msg.velocity = {(float)vx, (float)vy, (float)vz};
    msg.acceleration = {(float)ax, (float)ay, (float)az};
  //  msg.position = {(float)main_drone_position->lat, (float)main_drone_position->lon, (float)main_drone_position->alt};
    msg.yaw = -3.14;  // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void FollowerDroneController::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 2;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void FollowerDroneController::mainVehicleLocalPositionCallback(const VehicleLocalPosition::SharedPtr msg)
{
    main_drone_position_ = msg;
}

void FollowerDroneController::followerVehicleLocalPositionCallback(const VehicleLocalPosition::SharedPtr msg)
{
   follower_drone_position_ = msg;
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowerDroneController>());
    rclcpp::shutdown();
    return 0;
}