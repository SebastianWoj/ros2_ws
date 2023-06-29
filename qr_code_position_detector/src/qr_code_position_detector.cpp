#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <px4_msgs/msg/vehicle_local_position.hpp>
class QrCodePositionDetector : public rclcpp::Node
{
public:
  explicit QrCodePositionDetector() : Node("qr_code_position_detector")
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera",
      qos,
      std::bind(&QrCodePositionDetector::imageCallback, this, std::placeholders::_1)
    );


  depth_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
    "depth_camera",
    qos,
    std::bind(&QrCodePositionDetector::depthImageCallback, this, std::placeholders::_1)
  );

 drone_pose_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/drone_camera/fmu/out/vehicle_local_position",
      qos,
      std::bind(&QrCodePositionDetector::dronePoseCallback, this, std::placeholders::_1)
    );
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr drone_pose_subscriber_;

  cv::Point qr_code_center_;
  float depth_;
  bool qr_code_detected_;

void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Process the depth camera image and calculate distance to the center of the drone using the QR code's location
  // You can use cv_bridge and OpenCV functions to process the depth image
   if (qr_code_detected_)
      {
  // Example code to get the center of the drone and calculate distance:
  cv_bridge::CvImagePtr cvDepthImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  
  
  // Access depth value at the center point
  float depth = cvDepthImage->image.at<float>(qr_code_center_.y, qr_code_center_.x);
  
  // Calculate distance to the center of the drone using the depth value
  //float distance = ... // Calculate the distance based on your depth camera's characteristics
   }
  // Print or use the calculated distance as needed
  //RCLCPP_INFO(get_logger(), "Distance to center of drone: %f", distance);
}   


  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       qr_code_detected_ = false;
      // Convert the image to grayscale for QR code detection
      cv::Mat grayImage;
      cv::cvtColor(cvImage->image, grayImage, cv::COLOR_BGR2GRAY);

      // Create a zbar image scanner
      zbar::ImageScanner scanner;
      scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

      // Wrap the grayscale image data with a zbar image
      zbar::Image image(cvImage->image.cols, cvImage->image.rows, "Y800", grayImage.data, cvImage->image.cols * cvImage->image.rows);

      // Scan for QR codes in the image
      scanner.scan(image);

      // Iterate over the detected symbols
      for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
      {

        // Get the QR code location
        std::vector<cv::Point> points;
        for (int i = 0; i < symbol->get_location_size(); ++i)
        {
          points.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
        }

        // Draw a green outline around the QR code
        cv::polylines(cvImage->image, points, true, cv::Scalar(0, 255, 0), 2);
        qr_code_detected_=true;
 if (qr_code_detected_)
      {
        // Get the center of the QR code
        qr_code_center_ = cv::Point(symbol->get_location_x(0) + symbol->get_location_x(2) / 2,
                                     symbol->get_location_y(0) + symbol->get_location_y(2) / 2);

        // Display the location of the center of the QR code
        std::string centerText = "Center: (" + std::to_string(qr_code_center_.x) + ", " + std::to_string(qr_code_center_.y) + ")";
        cv::putText(cvImage->image, centerText, qr_code_center_, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
      }
}
      // Display the image with QR code outlines
      cv::imshow("QR Code Detection", cvImage->image);
      cv::waitKey(1);
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "cv_bridge exception: " << e.what());
    
    }
  }

   void dronePoseCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
        if (qr_code_detected_)
    {
    // Access the position information of the drone
    float drone_x = msg->x;
    float drone_y = msg->y;
    float drone_z = msg->z;

    // Calculate the Euclidean distance between the drone's position and the QR code's center
    float drone_object_distance = std::sqrt(std::pow(drone_x - qr_code_center_.x, 2) +
                                            std::pow(drone_y - qr_code_center_.y, 2) +
                                            std::pow(drone_z - depth_, 2));

    // Print or use the calculated distance as needed
    RCLCPP_INFO(get_logger(), "Distance from drone to object: %f", drone_object_distance);
  }
  }
};


int main(int argc, char **argv)
{
 rclcpp::init(argc, argv);
  auto node = std::make_shared<QrCodePositionDetector>();

  // Create an async spinner to handle callbacks
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Start the event loop
  while (rclcpp::ok())
  {
    executor.spin_some();
    cv::waitKey(1);  // Process GUI events and update the window
  }

  rclcpp::shutdown();
 
}

