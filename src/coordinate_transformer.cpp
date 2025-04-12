/**
 * @file coordinate_transformer.cpp
 * @brief Файл реализации для класса CoordinateTransformer.
 *
 * @author Ruslan Mukhametsafin
 * @date   2025-04-10
 */

#include "coordinate_transformer/coordinate_transformer.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <filesystem>
#include <chrono>
#include <stdexcept>
#include <set>
#include <vector>
#include <string>
#include <map>
#include <cmath>

namespace coordinate_transformer
{

// --- Конструкторы ---
CoordinateTransformer::CoordinateTransformer(rclcpp::Node::SharedPtr node)
: node_(node),
  logger_(node ? node->get_logger() : rclcpp::get_logger("CoordinateTransformerDefault"))
{
  if (!node_) {
    RCLCPP_FATAL(logger_, "Node pointer cannot be null!");
    throw std::invalid_argument("Node pointer cannot be null!");
  }
  initialize(nullptr);
}

CoordinateTransformer::CoordinateTransformer(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
: node_(node),
  logger_(node ? node->get_logger() : rclcpp::get_logger("CoordinateTransformerDefault")),
  tf_buffer_(tf_buffer)
{
  if (!node_) {
    RCLCPP_FATAL(logger_, "Node pointer cannot be null!");
    throw std::invalid_argument("Node pointer cannot be null!");
  }
  if (!tf_buffer_) {
    RCLCPP_FATAL(logger_, "TF Buffer pointer cannot be null for this constructor!");
    throw std::invalid_argument("TF Buffer pointer cannot be null for this constructor!");
  }
  initialize(tf_buffer_);
}

// --- Метод инициализации ---

void CoordinateTransformer::initialize(
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  RCLCPP_INFO(logger_, "Initializing CoordinateTransformer internals using ROS parameters...");

  if (tf_buffer) {
    tf_buffer_ = tf_buffer;
    RCLCPP_INFO(logger_, "Using provided TF2 buffer.");
  } else {
    if (!tf_buffer_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      tf_buffer_->setUsingDedicatedThread(true);
      RCLCPP_INFO(logger_, "Created internal TF2 buffer.");
    } else {
      RCLCPP_WARN(logger_, "Using pre-existing internal TF2 buffer (unexpected scenario?).");
    }
    try {
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
      RCLCPP_INFO(logger_, "Created TF2 listener for internal buffer.");
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(logger_, "Failed to create internal TransformListener: %s", e.what());
      tf_buffer_.reset();
      throw;
    }
  }

  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

  try {
    auto static_tf_desc = rcl_interfaces::msg::ParameterDescriptor();
    static_tf_desc.description =
      "List of symbolic names for static transforms defined under 'static_transforms.<name>'.";
    node_->declare_parameter<std::vector<std::string>>("static_transform_names",
        std::vector<std::string>{}, static_tf_desc);

    auto boundary_frames_desc = rcl_interfaces::msg::ParameterDescriptor();
    boundary_frames_desc.description =
      "List of frame_ids for which boundaries are defined under 'boundaries.<frame_id>.";
    node_->declare_parameter<std::vector<std::string>>("boundary_frame_names",
        std::vector<std::string>{}, boundary_frames_desc);

    auto initialized_desc = rcl_interfaces::msg::ParameterDescriptor();
    initialized_desc.description =
      "Indicates if boundaries have been successfully loaded or set at least once.";
    initialized_desc.read_only = true;
    node_->declare_parameter<bool>("boundaries.initialized", false, initialized_desc);

    RCLCPP_INFO(logger_, "Declared core configuration parameters.");
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
    RCLCPP_WARN(logger_, "Core parameters were already declared: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error declaring core parameters: %s", e.what());
    throw;
  }

  try {
    std::vector<std::string> transform_names =
      node_->get_parameter("static_transform_names").as_string_array();

    if (!transform_names.empty()) {
      std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
      RCLCPP_INFO(logger_, "Processing %zu static transforms from parameters...",
          transform_names.size());

      for (const auto & name : transform_names) {
        std::string prefix = "static_transforms." + name + ".";
        try {
          rcl_interfaces::msg::ParameterDescriptor transform_param_desc;
          transform_param_desc.description = "Parameter for static transform '" + name + "'.";
          transform_param_desc.dynamic_typing = true;

          node_->declare_parameter<std::string>(prefix + "parent_frame", "", transform_param_desc);
          node_->declare_parameter<std::string>(prefix + "child_frame", "", transform_param_desc);
          node_->declare_parameter<double>(prefix + "translation.x", 0.0, transform_param_desc);
          node_->declare_parameter<double>(prefix + "translation.y", 0.0, transform_param_desc);
          node_->declare_parameter<double>(prefix + "translation.z", 0.0, transform_param_desc);
          node_->declare_parameter<double>(prefix + "rotation.x", 0.0, transform_param_desc);
          node_->declare_parameter<double>(prefix + "rotation.y", 0.0, transform_param_desc);
          node_->declare_parameter<double>(prefix + "rotation.z", 0.0, transform_param_desc);
          node_->declare_parameter<double>(prefix + "rotation.w", 1.0, transform_param_desc);

          std::string parent_frame = node_->get_parameter(prefix + "parent_frame").as_string();
          std::string child_frame = node_->get_parameter(prefix + "child_frame").as_string();

          if (parent_frame.empty() || child_frame.empty()) {
            RCLCPP_WARN(logger_,
                "Skipping static transform '%s': parent_frame or child_frame is empty.",
                name.c_str());
            continue;
          }
          if (parent_frame == child_frame) {
            RCLCPP_WARN(logger_,
                "Skipping static transform '%s': parent_frame and child_frame are the same ('%s').",
                name.c_str(), parent_frame.c_str());
            continue;
          }

          geometry_msgs::msg::TransformStamped tf;
          tf.header.frame_id = parent_frame;
          tf.child_frame_id = child_frame;
          tf.transform.translation.x = node_->get_parameter(prefix + "translation.x").as_double();
          tf.transform.translation.y = node_->get_parameter(prefix + "translation.y").as_double();
          tf.transform.translation.z = node_->get_parameter(prefix + "translation.z").as_double();
          tf.transform.rotation.x = node_->get_parameter(prefix + "rotation.x").as_double();
          tf.transform.rotation.y = node_->get_parameter(prefix + "rotation.y").as_double();
          tf.transform.rotation.z = node_->get_parameter(prefix + "rotation.z").as_double();
          tf.transform.rotation.w = node_->get_parameter(prefix + "rotation.w").as_double();

          tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
            tf.transform.rotation.z, tf.transform.rotation.w);
          if (std::abs(q.length2() - 1.0) > 1e-3) {
            RCLCPP_WARN(logger_,
                "Quaternion for static transform '%s' (%s -> %s) is not normalized (length^2=%.4f). Normalizing.",
                                   name.c_str(), parent_frame.c_str(), child_frame.c_str(),
                q.length2());
            q.normalize();
            tf.transform.rotation = tf2::toMsg(q);
          }

          static_transforms.push_back(tf);
          RCLCPP_DEBUG(logger_, "Loaded static transform '%s': %s -> %s", name.c_str(),
              tf.header.frame_id.c_str(), tf.child_frame_id.c_str());

        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
          RCLCPP_INFO(logger_,
              "Parameters for static transform '%s' already declared, attempting to read: %s",
              name.c_str(), e.what());
          try {
            std::string parent_frame = node_->get_parameter(prefix + "parent_frame").as_string();
            std::string child_frame = node_->get_parameter(prefix + "child_frame").as_string();
            if (parent_frame.empty() || child_frame.empty()) {
              RCLCPP_WARN(logger_,
                  "Skipping already declared static transform '%s': parent or child frame is empty.",
                  name.c_str());
              continue;
            }
            if (parent_frame == child_frame) {
              RCLCPP_WARN(logger_,
                  "Skipping already declared static transform '%s': parent and child frame are the same ('%s').",
                  name.c_str(), parent_frame.c_str());
              continue;
            }
            geometry_msgs::msg::TransformStamped tf;
            tf.header.frame_id = parent_frame;
            tf.child_frame_id = child_frame;
            tf.transform.translation.x = node_->get_parameter(prefix + "translation.x").as_double();
            tf.transform.translation.y = node_->get_parameter(prefix + "translation.y").as_double();
            tf.transform.translation.z = node_->get_parameter(prefix + "translation.z").as_double();
            tf.transform.rotation.x = node_->get_parameter(prefix + "rotation.x").as_double();
            tf.transform.rotation.y = node_->get_parameter(prefix + "rotation.y").as_double();
            tf.transform.rotation.z = node_->get_parameter(prefix + "rotation.z").as_double();
            tf.transform.rotation.w = node_->get_parameter(prefix + "rotation.w").as_double();
            tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
              tf.transform.rotation.z, tf.transform.rotation.w);
            if (std::abs(q.length2() - 1.0) > 1e-3) {
              RCLCPP_WARN(logger_,
                  "Quaternion for already declared static transform '%s' (%s -> %s) is not normalized (length^2=%.4f). Normalizing.",
                                   name.c_str(), parent_frame.c_str(), child_frame.c_str(),
                  q.length2());
              q.normalize();
              tf.transform.rotation = tf2::toMsg(q);
            }
            static_transforms.push_back(tf);
            RCLCPP_DEBUG(logger_, "Loaded already declared static transform '%s': %s -> %s",
                name.c_str(), tf.header.frame_id.c_str(), tf.child_frame_id.c_str());

          } catch (const rclcpp::exceptions::ParameterNotDeclaredException & get_e) {
            RCLCPP_ERROR(logger_,
                "Error getting parameters for already declared static transform '%s': %s",
                name.c_str(), get_e.what());
          } catch (const std::exception & get_e) {
            RCLCPP_ERROR(logger_,
                "Unexpected error getting parameters for already declared static transform '%s': %s",
                name.c_str(), get_e.what());
          }

        } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
          RCLCPP_ERROR(logger_, "Error processing parameters for static transform '%s': %s",
              name.c_str(), e.what());
        } catch (const std::exception & e) {
          RCLCPP_ERROR(logger_, "Unexpected error processing static transform '%s': %s",
              name.c_str(), e.what());
        }
      }

      if (!static_transforms.empty()) {
        static_tf_broadcaster_->sendTransform(static_transforms);
        RCLCPP_INFO(logger_, "Published %zu static transforms from parameters.",
            static_transforms.size());
      } else {
        RCLCPP_INFO(logger_, "No valid static transforms loaded from parameters.");
      }
    } else {
      RCLCPP_INFO(logger_,
          "No static transforms defined in parameters (checked 'static_transform_names').");
    }

  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    RCLCPP_WARN(logger_,
        "Could not retrieve 'static_transform_names' parameter (maybe not provided?): %s",
        e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_,
        "An unexpected error occurred during static transform parameter processing: %s", e.what());
  }

  try {
    std::vector<std::string> initial_boundary_frames =
      node_->get_parameter("boundary_frame_names").as_string_array();
    if (!initial_boundary_frames.empty()) {
      RCLCPP_INFO(logger_,
          "Processing initial boundaries for %zu frames specified in 'boundary_frame_names' parameter...",
          initial_boundary_frames.size());
      for (const auto & frame_id : initial_boundary_frames) {
        if (frame_id.empty()) {
          RCLCPP_WARN(logger_, "Skipping empty frame_id found in initial 'boundary_frame_names'.");
          continue;
        }
        std::string prefix = "boundaries." + frame_id + ".";
        try {
          rcl_interfaces::msg::ParameterDescriptor bounds_param_desc;
          bounds_param_desc.description = "Boundary limit for frame '" + frame_id + "'.";
          bounds_param_desc.read_only = false;         // Must match callback declaration
          double default_val = std::numeric_limits<double>::quiet_NaN();

          if (!node_->has_parameter(prefix + "min_x")) {
            node_->declare_parameter(prefix + "min_x", default_val, bounds_param_desc);
          }
          if (!node_->has_parameter(prefix + "min_y")) {
            node_->declare_parameter(prefix + "min_y", default_val, bounds_param_desc);
          }
          if (!node_->has_parameter(prefix + "min_z")) {
            node_->declare_parameter(prefix + "min_z", default_val, bounds_param_desc);
          }
          if (!node_->has_parameter(prefix + "max_x")) {
            node_->declare_parameter(prefix + "max_x", default_val, bounds_param_desc);
          }
          if (!node_->has_parameter(prefix + "max_y")) {
            node_->declare_parameter(prefix + "max_y", default_val, bounds_param_desc);
          }
          if (!node_->has_parameter(prefix + "max_z")) {
            node_->declare_parameter(prefix + "max_z", default_val, bounds_param_desc);
          }

          geometry_msgs::msg::Point min_p, max_p;
          min_p.x = node_->get_parameter(prefix + "min_x").as_double();
          min_p.y = node_->get_parameter(prefix + "min_y").as_double();
          min_p.z = node_->get_parameter(prefix + "min_z").as_double();
          max_p.x = node_->get_parameter(prefix + "max_x").as_double();
          max_p.y = node_->get_parameter(prefix + "max_y").as_double();
          max_p.z = node_->get_parameter(prefix + "max_z").as_double();

          RCLCPP_INFO(logger_, "Applying initial bounds for frame '%s' from parameters.",
              frame_id.c_str());
          setBoundsInternal(frame_id, min_p, max_p);         // Validate and store internally

        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
          RCLCPP_WARN(logger_,
              "Parameters for initial boundary frame '%s' were already declared (unexpected): %s",
              frame_id.c_str(), e.what());
          try {
            geometry_msgs::msg::Point min_p, max_p;
            min_p.x = node_->get_parameter(prefix + "min_x").as_double();
            min_p.y = node_->get_parameter(prefix + "min_y").as_double();
            min_p.z = node_->get_parameter(prefix + "min_z").as_double();
            max_p.x = node_->get_parameter(prefix + "max_x").as_double();
            max_p.y = node_->get_parameter(prefix + "max_y").as_double();
            max_p.z = node_->get_parameter(prefix + "max_z").as_double();
            RCLCPP_INFO(logger_,
                "Applying initial bounds for already declared frame '%s' from parameters.",
                frame_id.c_str());
            setBoundsInternal(frame_id, min_p, max_p);
          } catch (const std::exception & get_e) {
            RCLCPP_ERROR(logger_,
                "Error getting parameters for already declared initial boundary frame '%s': %s",
                frame_id.c_str(), get_e.what());
          }
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
          RCLCPP_ERROR(logger_,
              "Error declaring/getting initial boundary parameters for frame '%s': %s. Bounds may not be set.",
              frame_id.c_str(), e.what());
        } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
          RCLCPP_ERROR(logger_,
              "Invalid parameter type for initial boundary frame '%s': %s. Bounds may not be set.",
              frame_id.c_str(), e.what());
        } catch (const std::exception & e) {
          RCLCPP_ERROR(logger_, "Unexpected error processing initial boundaries for frame '%s': %s",
              frame_id.c_str(), e.what());
        }
      }
    } else {
      RCLCPP_INFO(logger_,
          "No initial boundary frames defined in 'boundary_frame_names' parameter.");
    }
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    RCLCPP_WARN(logger_,
        "Could not retrieve initial 'boundary_frame_names' parameter (maybe not provided?): %s",
        e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_,
        "An unexpected error occurred during initial boundary parameter processing: %s", e.what());
  }

  param_callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&CoordinateTransformer::parametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "CoordinateTransformer initialized successfully using ROS parameters.");
}


// --- Публичные методы ---

void CoordinateTransformer::addTransform(const geometry_msgs::msg::TransformStamped & transform)
{
  if (transform.header.frame_id.empty() || transform.child_frame_id.empty()) {
    RCLCPP_WARN(logger_, "addTransform: Skipping transform with empty parent or child frame_id.");
    return;
  }
  if (transform.header.frame_id == transform.child_frame_id) {
    RCLCPP_WARN(logger_,
        "addTransform: Skipping transform where parent ('%s') and child ('%s') frame_id are the same.",
                   transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
    return;
  }
  tf2::Quaternion q;
  tf2::fromMsg(transform.transform.rotation, q);
  if (std::abs(q.length2() - 1.0) > 1e-3) {
    RCLCPP_WARN(logger_,
        "addTransform: Quaternion for transform %s -> %s is not normalized (length^2=%.4f). Normalizing before publishing.",
            transform.header.frame_id.c_str(), transform.child_frame_id.c_str(), q.length2());
    q.normalize();
    geometry_msgs::msg::TransformStamped normalized_transform = transform;
    normalized_transform.transform.rotation = tf2::toMsg(q);
    static_tf_broadcaster_->sendTransform(normalized_transform);
  } else {
    static_tf_broadcaster_->sendTransform(transform);
  }
  RCLCPP_DEBUG(logger_, "Published static transform via addTransform: %s -> %s",
      transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
}

void CoordinateTransformer::setBounds(
  const std::string & frame_id,
  const geometry_msgs::msg::Point & min,
  const geometry_msgs::msg::Point & max)
{
  if (frame_id.empty()) {
    RCLCPP_ERROR(logger_, "Cannot set bounds for an empty frame_id.");
    return;
  }

  RCLCPP_INFO(logger_, "Programmatically setting bounds for frame '%s'.", frame_id.c_str());

  if (!setBoundsInternal(frame_id, min, max)) {
    return;
  }
}

void CoordinateTransformer::removeBounds(const std::string & frame_id)
{
  if (frame_id.empty()) {
    RCLCPP_WARN(logger_, "Attempted to remove bounds for an empty frame_id.");
    return;
  }

  std::lock_guard<std::mutex> lock(bounds_mutex_);
  if (bounds_.erase(frame_id) > 0) {
    RCLCPP_INFO(logger_, "Removed bounds for frame '%s'.", frame_id.c_str());
  } else {
    RCLCPP_DEBUG(logger_, "No bounds were set for frame '%s', removal request ignored.",
        frame_id.c_str());
  }
}

ResultStatus CoordinateTransformer::convert(
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output,
  const std::string & target_frame) const
{
  RCLCPP_INFO(logger_, "CONVERT: Input frame='%s', Target frame='%s'",
      input.header.frame_id.c_str(), target_frame.c_str());
  if (input.header.frame_id.empty() || target_frame.empty()) {
    RCLCPP_ERROR(logger_, "Invalid input: Input frame_id ('%s') or target_frame ('%s') is empty.",
                     input.header.frame_id.c_str(), target_frame.c_str());
    return ResultStatus::INVALID_INPUT;
  }
  if (!tf_buffer_) {
    RCLCPP_ERROR(logger_, "TF Buffer is not initialized. Cannot perform conversion.");
    return ResultStatus::CONFIGURATION_ERROR;
  }

  try {
    output = tf_buffer_->transform(input, target_frame,
        tf2::Duration(std::chrono::milliseconds(100)));

    if (!checkBounds(output.pose.position, output.header.frame_id)) {
      RCLCPP_WARN(logger_,
          "Conversion result for frame '%s' is out of bounds. Input: (%s), Output: (%s)",
                         output.header.frame_id.c_str(), input.header.frame_id.c_str(),
          output.header.frame_id.c_str());
      return ResultStatus::OUT_OF_BOUNDS;
    }

    return ResultStatus::SUCCESS;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Could not transform '%s' to '%s': %s",
                     input.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return ResultStatus::TRANSFORM_NOT_FOUND;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "Unexpected exception during transform from '%s' to '%s': %s",
                     input.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return ResultStatus::CONFIGURATION_ERROR;
  }
}

ResultStatus CoordinateTransformer::inverseConvert(
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output,
  const std::string & source_frame) const
{
  RCLCPP_DEBUG(logger_, "INVERSE CONVERT: Input frame='%s', Target (Source) frame='%s'",
      input.header.frame_id.c_str(), source_frame.c_str());
  return convert(input, output, source_frame);
}

const rclcpp::Logger & CoordinateTransformer::getLogger() const
{
  return logger_;
}

// --- Приватные методы ---

bool CoordinateTransformer::checkBounds(
  const geometry_msgs::msg::Point & point,
  const std::string & frame_id) const
{
  std::lock_guard<std::mutex> lock(bounds_mutex_);
  auto it = bounds_.find(frame_id);
  if (it == bounds_.end()) {
    return true;
  }

  const auto & bounds = it->second;
  return point.x >= bounds.min.x && point.x <= bounds.max.x &&
         point.y >= bounds.min.y && point.y <= bounds.max.y &&
         point.z >= bounds.min.z && point.z <= bounds.max.z;
}

rcl_interfaces::msg::SetParametersResult CoordinateTransformer::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::vector<std::string> previous_boundary_frames;
  std::vector<std::string> next_boundary_frames;
  bool boundary_list_changed = false;

  std::set<std::string> frames_added_this_call;
  std::set<std::string> frames_removed_this_call;

  for (const auto & param : parameters) {
    if (param.get_name() == "boundary_frame_names") {
      boundary_list_changed = true;
      try {
        previous_boundary_frames = node_->get_parameter("boundary_frame_names").as_string_array();
        next_boundary_frames = param.as_string_array();

        std::set<std::string> prev_set(previous_boundary_frames.begin(),
          previous_boundary_frames.end());
        std::set<std::string> next_set(next_boundary_frames.begin(), next_boundary_frames.end());

        std::set_difference(next_set.begin(), next_set.end(),
                                    prev_set.begin(), prev_set.end(),
                                    std::inserter(frames_added_this_call,
            frames_added_this_call.begin()));

        std::set_difference(prev_set.begin(), prev_set.end(),
                                    next_set.begin(), next_set.end(),
                                    std::inserter(frames_removed_this_call,
            frames_removed_this_call.begin()));

      } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
        RCLCPP_ERROR(logger_,
            "Critical error: 'boundary_frame_names' accessed before declaration in callback.");
        result.successful = false;
        result.reason = "Internal error: boundary_frame_names not declared.";
        return result;
      } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
        RCLCPP_ERROR(logger_, "Invalid type for 'boundary_frame_names': %s", e.what());
        result.successful = false;
        result.reason = "boundary_frame_names must be a string array.";
        return result;
      }
      break;
    }
  }

  for (const auto & frame_id : frames_added_this_call) {
    if (frame_id.empty()) {
      RCLCPP_WARN(logger_, "Ignoring empty frame_id added to boundary_frame_names.");
      continue;
    }
    std::string prefix = "boundaries." + frame_id + ".";
    RCLCPP_INFO(logger_, "Frame '%s' added to boundary list. Declaring parameters...",
        frame_id.c_str());
    try {
      rcl_interfaces::msg::ParameterDescriptor bounds_param_desc;
      bounds_param_desc.description = "Boundary limit for frame '" + frame_id + "'.";
      bounds_param_desc.read_only = false;

      double default_val = std::numeric_limits<double>::quiet_NaN();
      if (!node_->has_parameter(prefix + "min_x")) {
        node_->declare_parameter(prefix + "min_x", default_val, bounds_param_desc);
      }
      if (!node_->has_parameter(prefix + "min_y")) {
        node_->declare_parameter(prefix + "min_y", default_val, bounds_param_desc);
      }
      if (!node_->has_parameter(prefix + "min_z")) {
        node_->declare_parameter(prefix + "min_z", default_val, bounds_param_desc);
      }
      if (!node_->has_parameter(prefix + "max_x")) {
        node_->declare_parameter(prefix + "max_x", default_val, bounds_param_desc);
      }
      if (!node_->has_parameter(prefix + "max_y")) {
        node_->declare_parameter(prefix + "max_y", default_val, bounds_param_desc);
      }
      if (!node_->has_parameter(prefix + "max_z")) {
        node_->declare_parameter(prefix + "max_z", default_val, bounds_param_desc);
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to declare parameters for newly added frame '%s': %s",
          frame_id.c_str(), e.what());
    }
  }

  for (const auto & frame_id : frames_removed_this_call) {
    if (frame_id.empty()) {continue;}
    RCLCPP_INFO(logger_, "Frame '%s' removed from boundary list. Clearing internal bounds.",
        frame_id.c_str());
    removeBounds(frame_id);
  }


  std::map<std::string, std::map<std::string, rclcpp::Parameter>> grouped_bounds_params;

  for (const auto & param : parameters) {
    const std::string & name = param.get_name();

    if (name.rfind("boundaries.",
        0) == 0 && name != "boundaries.initialized" && name != "boundary_frame_names")
    {
      size_t second_dot = name.find('.', sizeof("boundaries.") - 1);
      if (second_dot != std::string::npos) {
        std::string frame_id = name.substr(sizeof("boundaries.") - 1,
            second_dot - (sizeof("boundaries.") - 1));
        std::string suffix = name.substr(second_dot + 1);

        if (!frame_id.empty() &&
          (suffix == "min_x" || suffix == "min_y" || suffix == "min_z" ||
          suffix == "max_x" || suffix == "max_y" || suffix == "max_z"))
        {
          const auto & effective_frames_list =
            boundary_list_changed ? next_boundary_frames :
            node_->get_parameter("boundary_frame_names").as_string_array();
          if (std::find(effective_frames_list.begin(), effective_frames_list.end(),
              frame_id) != effective_frames_list.end())
          {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
              result.successful = false;
              result.reason = "Parameter " + name + " must be a double.";
              RCLCPP_ERROR(logger_, "%s", result.reason.c_str());
              return result;                   // Fail fast on type error
            }
            grouped_bounds_params[frame_id][suffix] = param;
          } else {
            RCLCPP_WARN(logger_,
                "Ignoring parameter '%s' because frame '%s' is not in the effective 'boundary_frame_names' list.",
                name.c_str(), frame_id.c_str());
          }

        } else {
          RCLCPP_WARN(logger_, "Ignoring unrecognized parameter format under 'boundaries.': %s",
              name.c_str());
        }
      }
    } else if (param.get_name() == "boundaries.initialized") {
      RCLCPP_WARN(logger_, "Attempt to set read-only parameter 'boundaries.initialized' ignored.");
    }
  }

  for (auto const & [frame_id, param_map] : grouped_bounds_params) {
    geometry_msgs::msg::Point current_min, current_max;
    bool current_bounds_found = false;
    {
      std::lock_guard<std::mutex> lock(bounds_mutex_);
      auto it = bounds_.find(frame_id);
      if (it != bounds_.end()) {
        current_min = it->second.min;
        current_max = it->second.max;
        current_bounds_found = true;
      } else {
        double default_val = std::numeric_limits<double>::quiet_NaN();
        current_min.x = current_min.y = current_min.z = default_val;
        current_max.x = current_max.y = current_max.z = default_val;
      }
    }

    geometry_msgs::msg::Point proposed_min = current_min;
    geometry_msgs::msg::Point proposed_max = current_max;

    for (auto const & [suffix, param] : param_map) {
      double value = param.as_double();
      if (suffix == "min_x") {proposed_min.x = value;} else if (suffix == "min_y") {
        proposed_min.y = value;
      } else if (suffix == "min_z") {proposed_min.z = value;} else if (suffix == "max_x") {
        proposed_max.x = value;
      } else if (suffix == "max_y") {proposed_max.y = value;} else if (suffix == "max_z") {
        proposed_max.z = value;
      }
    }

    RCLCPP_DEBUG(logger_, "Validating proposed bounds for frame '%s': min(%f,%f,%f) max(%f,%f,%f)",
                     frame_id.c_str(), proposed_min.x, proposed_min.y, proposed_min.z,
        proposed_max.x, proposed_max.y, proposed_max.z);

    if (!setBoundsInternal(frame_id, proposed_min, proposed_max)) {
      result.successful = false;
      result.reason = "Invalid bounds proposed for frame '" + frame_id +
        "': min must be <= max for all axes, and values cannot be NaN unless explicitly handled.";
      RCLCPP_ERROR(logger_, "%s", result.reason.c_str());
      return result;
    }
  }


  RCLCPP_DEBUG(logger_, "Parameter changes approved and applied.");
  return result;
}

bool CoordinateTransformer::setBoundsInternal(
  const std::string & frame_id,
  const geometry_msgs::msg::Point & min,
  const geometry_msgs::msg::Point & max)
{
  if (min.x > max.x || min.y > max.y || min.z > max.z) {
    RCLCPP_ERROR(logger_, "Invalid bounds for frame '%s': min(%f, %f, %f) > max(%f, %f, %f)",
                     frame_id.c_str(), min.x, min.y, min.z, max.x, max.y, max.z);
    return false;
  }
  if (std::isnan(min.x) || std::isnan(min.y) || std::isnan(min.z) ||
    std::isnan(max.x) || std::isnan(max.y) || std::isnan(max.z))
  {
    RCLCPP_ERROR(logger_, "Invalid bounds for frame '%s': NaN detected in min/max values.",
        frame_id.c_str());
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(bounds_mutex_);
    bounds_[frame_id] = {min, max};
  }
  RCLCPP_DEBUG(logger_,
      "Successfully set/updated internal bounds for frame '%s': min(%f,%f,%f) max(%f,%f,%f)",
                 frame_id.c_str(), min.x, min.y, min.z, max.x, max.y, max.z);

  return true;
}

} // namespace coordinate_transformer
