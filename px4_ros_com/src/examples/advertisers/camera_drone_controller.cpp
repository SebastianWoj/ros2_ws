#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include <chrono>

using namespace std::chrono_literals;

class CameraDroneController : public rclcpp::Node
{
public:
  CameraDroneController()
      : Node("camera_drone_controller")
  {
    publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>("drone_camera/fmu/in/vehicle_command", 10);

    // Create a timer to send the command repeatedly
    timer_ = create_wall_timer(1s, std::bind(&CameraDroneController::sendCommand, this));
  }

private:
  void sendCommand()
  {
    auto command_msg = std::make_unique<px4_msgs::msg::VehicleCommand>();
    command_msg->target_system = 1;
    command_msg->command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_ORBIT;
    command_msg->param1 = 3.0;       // Smaller radius of the circle in meters
    command_msg->param2 = 5.0;       // Velocity of the drone in m/s
    command_msg->param3 = 0;         // Yaw behavior (0: Hold the current yaw, 1: Face the center of the circle)
    command_msg->param4 = 0;         // Empty
    command_msg->param5 = 47.397751; // Latitude of the center of the circle
    command_msg->param6 = 8.5455893; // Longitude of the center of the circle
    command_msg->param7 = 490.0;     // Altitude of the center of the circle

    publisher_->publish(std::move(command_msg));
  }

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraDroneController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
