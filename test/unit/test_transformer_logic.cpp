/**
 * @file test_transformer_logic.cpp
 * @brief Unit tests для валидации логики класса CoordinateTransformer, используя GTest.
 *
 * @author Ruslan Mukhametsafin
 * @date   2025-04-10
 */

#include "coordinate_transformer/coordinate_transformer.hpp"
#include "coordinate_transformer/result_status.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <thread>

// --- Вспомогательные функции ---
/**
 * @brief Создает объект PoseStamped с заданной позицией и единичной ориентацией.
 * @param frame_id Идентификатор системы координат.
 * @param x Координата X.
 * @param y Координата Y.
 * @param z Координата Z.
 * @return Объект PoseStamped.
 */
geometry_msgs::msg::PoseStamped create_pose(
  const std::string & frame_id, double x, double y,
  double z)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

/**
 * @brief Создает объект Point с заданными координатами.
 * @param x Координата X.
 * @param y Координата Y.
 * @param z Координата Z.
 * @return Объект Point.
 */
geometry_msgs::msg::Point create_point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

/**
 * @brief Ожидает доступности трансформации между двумя системами координат в буфере TF2.
 * @param buffer Указатель на буфер TF2.
 * @param target_frame Целевая система координат.
 * @param source_frame Исходная система координат.
 * @param time Временная точка, для которой проверяется трансформация.
 * @param timeout Максимальное время ожидания.
 * @param node_to_spin Указатель на узел ROS2 для прокрутки во время ожидания (опционально).
 * @return true, если трансформация доступна в течение таймаута, иначе false.
 */
bool wait_for_transform(
  const std::shared_ptr<tf2_ros::Buffer> & buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  const std::chrono::duration<double> & timeout,
  const rclcpp::Node::SharedPtr & node_to_spin = nullptr)
{
  rclcpp::Clock::SharedPtr clock = node_to_spin ? node_to_spin->get_clock() : std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::Time start_time = clock->now();
  rclcpp::Time end_time = start_time + rclcpp::Duration(timeout);

  while (rclcpp::ok() && clock->now() < end_time) {
    rclcpp::Duration remaining_rcl_time = end_time - clock->now();
    if (remaining_rcl_time < rclcpp::Duration(0, 0)) {
      remaining_rcl_time = rclcpp::Duration(0, 0);
    }
    tf2::Duration remaining_tf2_time = tf2::durationFromSec(remaining_rcl_time.seconds());

    if (buffer->canTransform(target_frame, source_frame, time, remaining_tf2_time)) {
      return true;
    }
    if (node_to_spin) {
      rclcpp::spin_some(node_to_spin);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  rclcpp::Duration final_remaining_rcl = end_time - clock->now();
  if (final_remaining_rcl < rclcpp::Duration(0, 0)) {final_remaining_rcl = rclcpp::Duration(0, 0);}
  tf2::Duration final_remaining_tf2 = tf2::durationFromSec(final_remaining_rcl.seconds());
  return buffer->canTransform(target_frame, source_frame, time, final_remaining_tf2);
}

class TestCoordinateTransformer : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<coordinate_transformer::CoordinateTransformer> transformer;
  std::shared_ptr<tf2_ros::TransformListener> test_tf_listener_;
  std::filesystem::path test_dir;
  bool external_buffer_provided = false;

  void InitializeTransformer(bool provide_external_buffer)
  {
    node = std::make_shared<rclcpp::Node>("test_coordinate_transformer_node");
    test_dir = std::filesystem::temp_directory_path() / "coord_transformer_test";
    if (std::filesystem::exists(test_dir)) {
      std::filesystem::remove_all(test_dir);
    }
    std::filesystem::create_directories(test_dir);

    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_buffer->setUsingDedicatedThread(true);

    test_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, true);

    if (provide_external_buffer) {
      transformer = std::make_unique<coordinate_transformer::CoordinateTransformer>(node,
        tf_buffer);
      external_buffer_provided = true;
      RCLCPP_INFO(node->get_logger(), "Test setup: Using EXTERNAL TF buffer (shared with test).");
    } else {
      transformer = std::make_unique<coordinate_transformer::CoordinateTransformer>(node);
      external_buffer_provided = false;
      RCLCPP_INFO(node->get_logger(), "Test setup: Using INTERNAL TF buffer (transformer side).");
    }
    ASSERT_NE(transformer, nullptr) << "Transformer failed to initialize.";
    ASSERT_NE(tf_buffer, nullptr) << "Test verification TF buffer is null after initialization.";
    ASSERT_NE(test_tf_listener_, nullptr) << "Test TF listener failed to initialize.";
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    InitializeTransformer(true);
  }

  void TearDown() override
  {
    test_tf_listener_.reset();
    transformer.reset();
    if (tf_buffer) {
      if (tf_buffer->isUsingDedicatedThread()) {
        tf_buffer->setUsingDedicatedThread(false);
      }
      tf_buffer.reset();
    }
    node.reset();
    rclcpp::shutdown();
    if (std::filesystem::exists(test_dir)) {
      try {
        std::filesystem::remove_all(test_dir);
      } catch (const std::filesystem::filesystem_error & e) {
        std::cerr << "Warning: Could not remove test directory " << test_dir << ": " << e.what() <<
          std::endl;
      }
    }
  }

  void add_tf_to_buffer_direct(
    const std::string & parent, const std::string & child,
    double x, double y, double z,
    double qx = 0, double qy = 0, double qz = 0, double qw = 1,
    bool is_static = false)
  {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = node->get_clock()->now();
    tf_stamped.header.frame_id = parent;
    tf_stamped.child_frame_id = child;
    tf_stamped.transform.translation.x = x;
    tf_stamped.transform.translation.y = y;
    tf_stamped.transform.translation.z = z;
    tf_stamped.transform.rotation.x = qx;
    tf_stamped.transform.rotation.y = qy;
    tf_stamped.transform.rotation.z = qz;
    tf_stamped.transform.rotation.w = qw;
    tf_buffer->setTransform(tf_stamped, "test_authority_direct", is_static);
    ASSERT_TRUE(wait_for_transform(tf_buffer, parent, child, tf2::TimePointZero,
      std::chrono::milliseconds(200), node))
                << "Timed out waiting for DIRECT transform " << parent << " -> " << child;
  }

  void publish_static_tf(
    const std::string & parent, const std::string & child,
    double x, double y, double z,
    double qx = 0, double qy = 0, double qz = 0, double qw = 1)
  {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = node->get_clock()->now();
    tf_stamped.header.frame_id = parent;
    tf_stamped.child_frame_id = child;
    tf_stamped.transform.translation.x = x;
    tf_stamped.transform.translation.y = y;
    tf_stamped.transform.translation.z = z;
    tf_stamped.transform.rotation.x = qx;
    tf_stamped.transform.rotation.y = qy;
    tf_stamped.transform.rotation.z = qz;
    tf_stamped.transform.rotation.w = qw;

    auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    static_broadcaster->sendTransform(tf_stamped);

    bool transform_received = wait_for_transform(tf_buffer, parent, child, tf2::TimePointZero,
      std::chrono::seconds(1), node);
    if (!transform_received) {
      FAIL() << "Timed out waiting for PUBLISHED STATIC transform " << parent << " -> " << child;
    }
  }

  void check_conversion_status(
    const std::string & source_frame,
    const geometry_msgs::msg::Point & source_point,
    const std::string & target_frame,
    coordinate_transformer::ResultStatus expected_status,
    const std::string & message = "")
  {
    geometry_msgs::msg::PoseStamped input_pose = create_pose(source_frame, source_point.x,
      source_point.y, source_point.z);
    input_pose.header.stamp = tf2_ros::toMsg(tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped output_pose;

    rclcpp::sleep_for(std::chrono::milliseconds(10));
    rclcpp::spin_some(node);

    coordinate_transformer::ResultStatus actual_status = transformer->convert(input_pose,
      output_pose, target_frame);
    EXPECT_EQ(actual_status, expected_status)
                << "Conversion from " << source_frame << " to " << target_frame
                << " for point (" << source_point.x << ", " << source_point.y << ", " <<
      source_point.z << ")"
                << " expected status " << static_cast<int>(expected_status)
                << " but got " << static_cast<int>(actual_status) << ". " << message;
  }

  void add_identity_tf(const std::string & frame_id, bool use_publisher = false)
  {
    if (use_publisher) {
      publish_static_tf(frame_id, frame_id, 0, 0, 0);
    } else {
      add_tf_to_buffer_direct(frame_id, frame_id, 0, 0, 0, 0, 0, 0, 1, true);
    }
    ASSERT_TRUE(tf_buffer->canTransform(frame_id, frame_id, tf2::TimePointZero,
      std::chrono::milliseconds(100)))
                << "Failed to add identity transform for frame: " << frame_id;
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> add_static_tf_helper(
    const geometry_msgs::msg::TransformStamped & tf)
  {
    auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    static_broadcaster->sendTransform(tf);
    return static_broadcaster;
  }

};

// --- Тестовые случаи ---

TEST_F(TestCoordinateTransformer, ConstructorWithNodeOnly) {
    TearDown();
    rclcpp::init(0, nullptr);
    InitializeTransformer(false);

    ASSERT_NE(transformer, nullptr);
    EXPECT_FALSE(external_buffer_provided);

    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = node->get_clock()->now();
    tf_stamped.header.frame_id = "frame_a";
    tf_stamped.child_frame_id = "frame_b";
    tf_stamped.transform.translation.x = 1.0;
    tf_stamped.transform.rotation.w = 1.0;
    auto broadcaster = add_static_tf_helper(tf_stamped);

    ASSERT_TRUE(wait_for_transform(tf_buffer, "frame_a", "frame_b", tf2::TimePointZero,
    std::chrono::seconds(1), node))
         << "Test listener did not receive the published transform.";

    geometry_msgs::msg::PoseStamped input_pose = create_pose("frame_b", 0, 0, 0);
    input_pose.header.stamp = tf2_ros::toMsg(tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped output_pose;

    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);

    coordinate_transformer::ResultStatus status = transformer->convert(input_pose, output_pose,
    "frame_a");

    EXPECT_EQ(status, coordinate_transformer::ResultStatus::SUCCESS)
        << "Transformer with internal listener failed to convert using externally published TF.";
    EXPECT_NEAR(output_pose.pose.position.x, 1.0, 1e-6);
}

TEST_F(TestCoordinateTransformer, ConstructorWithNodeAndBuffer) {
     ASSERT_NE(transformer, nullptr);
     ASSERT_NE(tf_buffer, nullptr);
     EXPECT_TRUE(external_buffer_provided);

      add_tf_to_buffer_direct("frame_c", "frame_d", 1, 0, 0);

      EXPECT_TRUE(tf_buffer->canTransform("frame_c", "frame_d", tf2::TimePointZero));
}

TEST_F(TestCoordinateTransformer, ParameterInitialization) {
    ASSERT_NE(transformer, nullptr);
    EXPECT_NO_THROW(transformer->getLogger());
}

TEST_F(TestCoordinateTransformer, SetAndCheckBoundsViaConvert) {
    InitializeTransformer(true);

    std::string frame = "test_bounds_frame";
    auto min_p = create_point(-1.0, -1.0, -0.1);
    auto max_p = create_point(1.0, 1.0, 0.1);

    std::string prefix = "boundaries." + frame + ".";

    if (!node->has_parameter(prefix + "min_x")) {
    RCLCPP_WARN(node->get_logger(),
      "Test manually declaring bounds parameters for frame '%s'. CoordinateTransformer should handle this.",
      frame.c_str());
    node->declare_parameter(prefix + "min_x", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "min_y", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "min_z", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "max_x", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "max_y", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "max_z", rclcpp::ParameterValue(0.0));
    rclcpp::spin_some(node);
    }

    std::vector<rclcpp::Parameter> params_to_set = {
    rclcpp::Parameter(prefix + "min_x", min_p.x),
    rclcpp::Parameter(prefix + "min_y", min_p.y),
    rclcpp::Parameter(prefix + "min_z", min_p.z),
    rclcpp::Parameter(prefix + "max_x", max_p.x),
    rclcpp::Parameter(prefix + "max_y", max_p.y),
    rclcpp::Parameter(prefix + "max_z", max_p.z)
    };

    if (!node->has_parameter("boundary_frame_names")) {
    RCLCPP_WARN(node->get_logger(),
      "Test manually declaring 'boundary_frame_names'. The CoordinateTransformer should declare this.");
    node->declare_parameter("boundary_frame_names",
      rclcpp::ParameterValue(std::vector<std::string>{}));
    }
    std::vector<std::string> current_frames =
    node->get_parameter("boundary_frame_names").as_string_array();
    if (std::find(current_frames.begin(), current_frames.end(), frame) == current_frames.end()) {
    current_frames.push_back(frame);
    params_to_set.push_back(rclcpp::Parameter("boundary_frame_names", current_frames));
    }

     if (!node->has_parameter(prefix + "min_x")) {
    RCLCPP_WARN(node->get_logger(),
      "Test manually declaring bounds parameters for frame '%s'. CoordinateTransformer should handle this.",
      frame.c_str());
    node->declare_parameter(prefix + "min_x", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "min_y", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "min_z", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "max_x", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "max_y", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "max_z", rclcpp::ParameterValue(0.0));
    rclcpp::spin_some(node);
    }

    rcl_interfaces::msg::SetParametersResult result =
    node->set_parameters_atomically(params_to_set);

    ASSERT_TRUE(result.successful) << "Failed to set boundary parameters: " << result.reason;

    rclcpp::spin_some(node);

    add_identity_tf(frame, false);

    check_conversion_status(frame, create_point(0.0, 0.0, 0.0), frame,
    coordinate_transformer::ResultStatus::SUCCESS, "Point inside bounds");
    check_conversion_status(frame, create_point(1.5, 0.0, 0.0), frame,
    coordinate_transformer::ResultStatus::OUT_OF_BOUNDS, "Point outside X");
    check_conversion_status(frame, create_point(0.0, -1.1, 0.0), frame,
    coordinate_transformer::ResultStatus::OUT_OF_BOUNDS, "Point outside Y");
    check_conversion_status(frame, create_point(0.0, 0.0, 0.2), frame,
    coordinate_transformer::ResultStatus::OUT_OF_BOUNDS, "Point outside Z");
    check_conversion_status(frame, create_point(-1.0, -1.0, -0.1), frame,
    coordinate_transformer::ResultStatus::SUCCESS, "Point on min boundary");
    check_conversion_status(frame, create_point(1.0, 1.0, 0.1), frame,
    coordinate_transformer::ResultStatus::SUCCESS, "Point on max boundary");

    std::string frame_no_bounds = "no_bounds_frame";
    add_tf_to_buffer_direct(frame, frame_no_bounds, 1, 1, 1);

    check_conversion_status(frame_no_bounds, create_point(10.0, 10.0, 10.0), frame,
    coordinate_transformer::ResultStatus::OUT_OF_BOUNDS,
    "Convert from unbound frame TO bound frame (point outside target bounds)");

    check_conversion_status(frame, create_point(0.5, 0.5, 0.0), frame_no_bounds,
    coordinate_transformer::ResultStatus::SUCCESS, "Convert from bound frame TO unbound frame");
}

TEST_F(TestCoordinateTransformer, SetInvalidBounds) {
    InitializeTransformer(true);
    std::string frame = "invalid_bounds_frame";
    auto min_p = create_point(1.0, 0.0, 0.0);
    auto max_p = create_point(-1.0, 1.0, 1.0);

    std::string prefix = "boundaries." + frame + ".";

     if (!node->has_parameter(prefix + "min_x")) {
    RCLCPP_WARN(node->get_logger(),
      "Test manually declaring bounds parameters for frame '%s'. CoordinateTransformer should handle this.",
      frame.c_str());
    node->declare_parameter(prefix + "min_x", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "min_y", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "min_z", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "max_x", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "max_y", rclcpp::ParameterValue(0.0));
    node->declare_parameter(prefix + "max_z", rclcpp::ParameterValue(0.0));
    rclcpp::spin_some(node);
    }

    if (!node->has_parameter("boundary_frame_names")) {
    FAIL() <<
      "Test setup issue: 'boundary_frame_names' parameter not found. CoordinateTransformer should declare this.";
    }
    std::vector<std::string> current_frames =
    node->get_parameter("boundary_frame_names").as_string_array();
    rcl_interfaces::msg::SetParametersResult result;
    bool frame_added = false;
    if (std::find(current_frames.begin(), current_frames.end(), frame) == current_frames.end()) {
    current_frames.push_back(frame);

    result = node->set_parameters_atomically({rclcpp::Parameter("boundary_frame_names",
        current_frames)});
    ASSERT_TRUE(result.successful) << "Failed to add frame to boundary_frame_names list: " <<
      result.reason;
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(20));
    rclcpp::spin_some(node);
    frame_added = true;
    }

    auto set_param = [&](const std::string & name, double value) {
      result = node->set_parameters_atomically({rclcpp::Parameter(name, value)});

      ASSERT_TRUE(result.successful) << "Failed to set parameter '" << name << "': " <<
        result.reason;
      rclcpp::spin_some(node);
      rclcpp::sleep_for(std::chrono::milliseconds(5));
    };

    set_param(prefix + "min_x", min_p.x);
    set_param(prefix + "min_y", min_p.y);
    set_param(prefix + "min_z", min_p.z);
    set_param(prefix + "max_x", max_p.x);
    set_param(prefix + "max_y", max_p.y);
    set_param(prefix + "max_z", max_p.z);

    rclcpp::spin_some(node);

    add_identity_tf(frame, false);

    check_conversion_status(frame, create_point(2.0, 2.0, 2.0), frame,
    coordinate_transformer::ResultStatus::SUCCESS,
    "Convert after attempting to set invalid bounds (should succeed as if no bounds)");
}

TEST_F(TestCoordinateTransformer, RemoveBounds) {
    InitializeTransformer(true);
    std::string frame = "removable_frame";
    auto min_p = create_point(0.0, 0.0, 0.0);
    auto max_p = create_point(1.0, 1.0, 1.0);
    auto point_inside = create_point(0.5, 0.5, 0.5);
    auto point_outside = create_point(2.0, 2.0, 2.0);

    std::string prefix = "boundaries." + frame + ".";

    std::vector<rclcpp::Parameter> params_to_set = {
    rclcpp::Parameter(prefix + "min_x", min_p.x),
    rclcpp::Parameter(prefix + "min_y", min_p.y),
    rclcpp::Parameter(prefix + "min_z", min_p.z),
    rclcpp::Parameter(prefix + "max_x", max_p.x),
    rclcpp::Parameter(prefix + "max_y", max_p.y),
    rclcpp::Parameter(prefix + "max_z", max_p.z)
    };
    if (!node->has_parameter(prefix + "min_x")) {
    node->declare_parameter(prefix + "min_x", 0.0); node->declare_parameter(prefix + "min_y", 0.0);
    node->declare_parameter(prefix + "min_z", 0.0);
    node->declare_parameter(prefix + "max_x", 0.0); node->declare_parameter(prefix + "max_y", 0.0);
    node->declare_parameter(prefix + "max_z", 0.0);
    }
    if (!node->has_parameter("boundary_frame_names")) {
    node->declare_parameter("boundary_frame_names",
      rclcpp::ParameterValue(std::vector<std::string>{}));
    }
    std::vector<std::string> current_frames =
    node->get_parameter("boundary_frame_names").as_string_array();
    if (std::find(current_frames.begin(), current_frames.end(), frame) == current_frames.end()) {
    current_frames.push_back(frame);
    params_to_set.push_back(rclcpp::Parameter("boundary_frame_names", current_frames));
    }
    rcl_interfaces::msg::SetParametersResult result =
    node->set_parameters_atomically(params_to_set);
    ASSERT_TRUE(result.successful) << "Failed to set initial boundary parameters: " <<
    result.reason;
    rclcpp::spin_some(node);

    add_identity_tf(frame, false);

    check_conversion_status(frame, point_inside, frame,
    coordinate_transformer::ResultStatus::SUCCESS, "Inside bounds before removal");
    check_conversion_status(frame, point_outside, frame,
    coordinate_transformer::ResultStatus::OUT_OF_BOUNDS, "Outside bounds before removal");

    transformer->removeBounds(frame);

    check_conversion_status(frame, point_outside, frame,
    coordinate_transformer::ResultStatus::SUCCESS,
    "Outside bounds after internal removal (should succeed now)");

    EXPECT_TRUE(node->has_parameter(prefix + "min_x"));

     std::vector<std::string> updated_frames =
    node->get_parameter("boundary_frame_names").as_string_array();
     auto it = std::remove(updated_frames.begin(), updated_frames.end(), frame);
     if (it != updated_frames.end()) {
    updated_frames.erase(it, updated_frames.end());
    result = node->set_parameters_atomically({rclcpp::Parameter("boundary_frame_names",
        updated_frames)});
    ASSERT_TRUE(result.successful) << "Failed to remove frame from boundary_frame_names list.";
    rclcpp::spin_some(node);

    check_conversion_status(frame, point_outside, frame,
      coordinate_transformer::ResultStatus::SUCCESS,
      "Outside bounds after removing from param list (should succeed)");

     } else {
    RCLCPP_WARN(node->get_logger(),
      "Frame '%s' not found in boundary_frame_names parameter for removal test.", frame.c_str());
     }

}

TEST_F(TestCoordinateTransformer, ConvertSuccessWithTf) {
    TearDown();
    rclcpp::init(0, nullptr);
    InitializeTransformer(false);

    std::string frame1 = "conv_frame_1";
    std::string frame2 = "conv_frame_2";
    std::string target_frame = "conv_target";

    rclcpp::Time common_time = node->get_clock()->now();
    geometry_msgs::msg::TransformStamped tf1, tf2;

    tf1.header.stamp = common_time;
    tf1.header.frame_id = target_frame;
    tf1.child_frame_id = frame1;
    tf1.transform.translation.x = 1.0;
    tf1.transform.rotation.w = 1.0;

    tf2.header.stamp = common_time;
    tf2.header.frame_id = frame1;
    tf2.child_frame_id = frame2;
    tf2.transform.translation.y = 2.0;
    tf2.transform.rotation.w = 1.0;

    auto broadcaster1 = add_static_tf_helper(tf1);
    auto broadcaster2 = add_static_tf_helper(tf2);

    RCLCPP_INFO(node->get_logger(), "Waiting for internal TF listener...");

    bool transform_ready = false;
    for (int i = 0; i < 200; ++i) {
    rclcpp::spin_some(node);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    geometry_msgs::msg::PoseStamped temp_input_pose = create_pose(frame2, 0.0, 0.0, 0.0);
    temp_input_pose.header.stamp = common_time;
    geometry_msgs::msg::PoseStamped temp_output_pose;
    if (transformer->convert(temp_input_pose, temp_output_pose,
      target_frame) == coordinate_transformer::ResultStatus::SUCCESS)
    {
      transform_ready = true;
      RCLCPP_INFO(node->get_logger(), "Internal transform chain seems ready after %d ms.", i * 10);
      break;
    }
    }

    if (!transform_ready) {
    RCLCPP_ERROR(node->get_logger(),
      "Timed out waiting for internal TF listener to receive transforms.");
    }

    geometry_msgs::msg::PoseStamped input_pose = create_pose(frame2, 0.5, 0.5, 0.0);
    input_pose.header.stamp = common_time;
    geometry_msgs::msg::PoseStamped output_pose;

    coordinate_transformer::ResultStatus final_status = transformer->convert(input_pose,
    output_pose, target_frame);

    ASSERT_EQ(final_status, coordinate_transformer::ResultStatus::SUCCESS)
       << "Conversion failed. Status: " << static_cast<int>(final_status);

    ASSERT_EQ(output_pose.header.frame_id, target_frame);

    EXPECT_NEAR(output_pose.pose.position.x, 1.5, 1e-6);
    EXPECT_NEAR(output_pose.pose.position.y, 2.5, 1e-6);
    EXPECT_NEAR(output_pose.pose.position.z, 0.0, 1e-6);
}

TEST_F(TestCoordinateTransformer, ConvertTfErrorLookup) {
    InitializeTransformer(true);
    std::string frame_known = "known_frame";
    std::string frame_unknown = "unknown_target_frame";

    add_identity_tf(frame_known, false);

    check_conversion_status(frame_known, create_point(1.0, 1.0, 1.0), frame_unknown,
    coordinate_transformer::ResultStatus::TRANSFORM_NOT_FOUND);
}

TEST_F(TestCoordinateTransformer, ConvertTfErrorExtrapolation) {
     InitializeTransformer(true);
    std::string frame_a = "extrap_frame_a";
    std::string frame_b = "extrap_frame_b";

    rclcpp::Time tf_time = node->get_clock()->now();
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = tf_time;
    tf_stamped.header.frame_id = frame_a;
    tf_stamped.child_frame_id = frame_b;
    tf_stamped.transform.translation.x = 1.0;
    tf_stamped.transform.rotation.w = 1.0;
    tf_buffer->setTransform(tf_stamped, "test_authority_direct", false);
    ASSERT_TRUE(wait_for_transform(tf_buffer, frame_a, frame_b, tf2_ros::fromMsg(tf_time),
    std::chrono::seconds(1), node));

    geometry_msgs::msg::PoseStamped input_pose = create_pose(frame_b, 0.0, 0.0, 0.0);
    geometry_msgs::msg::PoseStamped output_pose;

    input_pose.header.stamp = tf_time - rclcpp::Duration::from_seconds(1.0);
    coordinate_transformer::ResultStatus status = transformer->convert(input_pose, output_pose,
    frame_a);
    EXPECT_EQ(status, coordinate_transformer::ResultStatus::TRANSFORM_NOT_FOUND);

    input_pose.header.stamp = tf_time + rclcpp::Duration::from_seconds(1.0);
    status = transformer->convert(input_pose, output_pose, frame_a);
    EXPECT_EQ(status, coordinate_transformer::ResultStatus::TRANSFORM_NOT_FOUND);

    check_conversion_status(frame_b, create_point(0.0, 0.0, 0.0), frame_a,
    coordinate_transformer::ResultStatus::SUCCESS);

    geometry_msgs::msg::PoseStamped final_input = create_pose(frame_b, 0.0, 0.0, 0.0);
    final_input.header.stamp = tf2_ros::toMsg(tf2::TimePointZero);
    status = transformer->convert(final_input, output_pose, frame_a);
    ASSERT_EQ(status, coordinate_transformer::ResultStatus::SUCCESS);
    EXPECT_NEAR(output_pose.pose.position.x, 1.0, 1e-6);
}

TEST_F(TestCoordinateTransformer, AddTransformTest) {
     InitializeTransformer(true);
    std::string parent = "addtf_parent";
    std::string child = "addtf_child";

    geometry_msgs::msg::TransformStamped tf_to_add;
    tf_to_add.header.stamp = node->get_clock()->now();
    tf_to_add.header.frame_id = parent;
    tf_to_add.child_frame_id = child;
    tf_to_add.transform.translation.x = 5.0;
    tf_to_add.transform.rotation.w = 1.0;

    transformer->addTransform(tf_to_add);

    ASSERT_TRUE(wait_for_transform(tf_buffer, parent, child, tf2::TimePointZero,
    std::chrono::seconds(1), node))
        << "Transform added via addTransform was not found in the buffer by test listener.";
}

/**
 * @brief Основная функция для запуска тестов GTest.
 * @param argc Количество аргументов командной строки.
 * @param argv Массив аргументов командной строки.
 * @return Статус выполнения тестов (0 в случае успеха).
 */
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
