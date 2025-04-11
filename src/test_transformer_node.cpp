/**
 * @file test_transformer_node.cpp
 * @brief Реализация ROS 2 node для тестирования класса CoordinateTransformer через Topic`и
 *
 * @author Ruslan Mukhametsafin
 * @date   2025-04-10
 */

#include <rclcpp/rclcpp.hpp>
#include <coordinate_transformer/coordinate_transformer.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <filesystem>

namespace coordinate_transformer
{

class TestTransformerNode : public rclcpp::Node
{
public:
  TestTransformerNode()
  : Node("test_transformer_node")
  {
    RCLCPP_INFO(this->get_logger(), "Declaring parameters...");
    this->declare_parameter<std::string>("target_frame", "target_map");
  }

  bool init()
  {
    RCLCPP_INFO(this->get_logger(), "Initializing TestTransformerNode internals...");

    target_frame_ = this->get_parameter("target_frame").as_string();

    try {
      transformer_ = std::make_shared<CoordinateTransformer>(
                this->shared_from_this()
      );

    } catch (const std::bad_weak_ptr & e) {
      RCLCPP_FATAL(this->get_logger(),
          "Failed to initialize CoordinateTransformer (bad_weak_ptr): %s", e.what());
      return false;
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize CoordinateTransformer: %s", e.what());
      return false;
    }


    output_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/test/output_pose", 10);

    rclcpp::QoS status_qos_profile(rclcpp::KeepLast(10));
    status_qos_profile.reliable();

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/test/transform_status", status_qos_profile);

    input_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/test/input_pose",
            10,
            std::bind(&TestTransformerNode::inputPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
        "TestTransformerNode initialized successfully. Waiting for input on /test/input_pose");
    RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
    return true;
  }

private:
  void inputPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received input pose in frame '%s'",
        msg->header.frame_id.c_str());
    geometry_msgs::msg::PoseStamped output_pose;
    std_msgs::msg::String status_msg;

    ResultStatus status = transformer_->convert(*msg, output_pose, target_frame_);

    switch (status) {
      case ResultStatus::SUCCESS:
        status_msg.data = "SUCCESS";
        RCLCPP_INFO(this->get_logger(),
          "Conversion successful. Publishing output pose in frame '%s'",
          output_pose.header.frame_id.c_str());
        output_pose_pub_->publish(output_pose);
        break;
      case ResultStatus::OUT_OF_BOUNDS:
        status_msg.data = "OUT_OF_BOUNDS";
        RCLCPP_WARN(this->get_logger(), "Conversion resulted in pose outside bounds for frame '%s'",
          target_frame_.c_str());
        break;
      case ResultStatus::TRANSFORM_NOT_FOUND:
        status_msg.data = "TRANSFORM_NOT_FOUND";
        RCLCPP_ERROR(this->get_logger(), "Transform not found from '%s' to '%s'",
          msg->header.frame_id.c_str(), target_frame_.c_str());
        break;
      case ResultStatus::INVALID_INPUT:
        status_msg.data = "INVALID_INPUT";
        RCLCPP_ERROR(this->get_logger(), "Invalid input pose received.");
        break;
      case ResultStatus::CONFIGURATION_ERROR:
        status_msg.data = "CONFIGURATION_ERROR";
        RCLCPP_ERROR(this->get_logger(),
          "Configuration error encountered during conversion attempt.");
        break;
      default:
        status_msg.data = "UNKNOWN_ERROR";
        RCLCPP_ERROR(this->get_logger(), "Unknown error during conversion.");
        break;
    }
    status_pub_->publish(status_msg);
  }

  std::shared_ptr<CoordinateTransformer> transformer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr input_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr output_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::string target_frame_;
};

} // namespace coordinate_transformer

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<coordinate_transformer::TestTransformerNode>();

  if (!node->init()) {
    RCLCPP_FATAL(node->get_logger(), "Node initialization failed. Shutting down.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
