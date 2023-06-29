#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/object_coordinates.hpp>

class ObjectDetection : public rclcpp::Node
{
public:
 explicit DetectionNode() : Node("object_detection")
  {
    // Create the subscriber
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/depth_camera", 10,
      std::bind(&ObjectDetection::depthCameraCallback, this, std::placeholders::_1));
  }

private:
  void depthCameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {


      auto object_coords = std::make_shared<object_detection::msg::ObjectCoordinates>();
      object_coords->x = /* calculated x coordinate */;
      object_coords->y = /* calculated y coordinate */;
      object_coords->z = /* calculated z coordinate */;
     // publisher_->publish(object_coords);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  //rclcpp::Publisher<object_detection::msg::ObjectCoordinates>::SharedPtr publisher_;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}