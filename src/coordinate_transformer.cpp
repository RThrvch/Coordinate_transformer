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
  RCLCPP_INFO(logger_, "Initializing CoordinateTransformer internals...");

  if (tf_buffer) {
      tf_buffer_ = tf_buffer;
      RCLCPP_INFO(logger_, "Using provided TF2 buffer.");
  } else {
      if (!tf_buffer_) {
           tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
           tf_buffer_->setUsingDedicatedThread(true);
           RCLCPP_INFO(logger_, "Created internal TF2 buffer.");
      } else {
           RCLCPP_INFO(logger_, "Using internal TF2 buffer (likely from test constructor).");
      }
      try {
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
        RCLCPP_INFO(logger_, "Created TF2 listener for internal buffer.");
      } catch (const std::runtime_error& e) {
         RCLCPP_ERROR(logger_, "Failed to create internal TransformListener: %s", e.what());
         tf_buffer_.reset();
         throw;
      }
  }

  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  try {
    if (!node_->has_parameter("boundaries.initialized")) {
        node_->declare_parameter<bool>("boundaries.initialized", false, rcl_interfaces::msg::ParameterDescriptor());
    }
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &e) {
      RCLCPP_WARN(logger_, "Parameter 'boundaries.initialized' was already declared: %s", e.what());
  }

  auto callback = [this](const std::vector<rclcpp::Parameter>& parameters) -> rcl_interfaces::msg::SetParametersResult
  {
      return this->parametersCallback(parameters);
  };
  param_callback_handle_ = node_->add_on_set_parameters_callback(callback);

  RCLCPP_INFO(logger_, "CoordinateTransformer initialized successfully.");
}


// --- Публичные методы ---

ResultStatus CoordinateTransformer::loadConfig(const std::string& yaml_path)
{
    RCLCPP_INFO(logger_, "Loading configuration from: %s", yaml_path.c_str());

    if (!std::filesystem::exists(yaml_path)) {
        RCLCPP_ERROR(logger_, "Configuration file not found: %s", yaml_path.c_str());
        return ResultStatus::CONFIGURATION_ERROR;
    }

    try {
        YAML::Node config = YAML::LoadFile(yaml_path);

        if (config["transforms"] && config["transforms"].IsSequence()) {
           std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
            for (const auto& tf_node : config["transforms"]) {
                geometry_msgs::msg::TransformStamped tf;
                if (!tf_node["parent_frame"] || !tf_node["child_frame"] || !tf_node["translation"] || !tf_node["rotation"]) {
                     RCLCPP_WARN(logger_, "Skipping invalid transform entry in YAML: missing required fields.");
                     continue;
                }
                tf.header.stamp = node_->get_clock()->now();
                tf.header.frame_id = tf_node["parent_frame"].as<std::string>();
                tf.child_frame_id = tf_node["child_frame"].as<std::string>();

                if (tf_node["translation"]["x"] && tf_node["translation"]["y"] && tf_node["translation"]["z"] ) {
                    tf.transform.translation.x = tf_node["translation"]["x"].as<double>();
                    tf.transform.translation.y = tf_node["translation"]["y"].as<double>();
                    tf.transform.translation.z = tf_node["translation"]["z"].as<double>();
                } else {
                    RCLCPP_WARN(logger_, "Skipping transform for '%s' -> '%s': invalid translation format.", tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
                    continue;
                }

                 if (tf_node["rotation"]["x"] && tf_node["rotation"]["y"] && tf_node["rotation"]["z"] && tf_node["rotation"]["w"]) {
                    tf.transform.rotation.x = tf_node["rotation"]["x"].as<double>();
                    tf.transform.rotation.y = tf_node["rotation"]["y"].as<double>();
                    tf.transform.rotation.z = tf_node["rotation"]["z"].as<double>();
                    tf.transform.rotation.w = tf_node["rotation"]["w"].as<double>();
                } else {
                     RCLCPP_WARN(logger_, "Skipping transform for '%s' -> '%s': invalid rotation format.", tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
                     continue;
                }
                static_transforms.push_back(tf);
                RCLCPP_DEBUG(logger_, "Loaded static transform: %s -> %s", tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
            }
            if (!static_transforms.empty()) {
                static_tf_broadcaster_->sendTransform(static_transforms);
                RCLCPP_INFO(logger_, "Published %zu static transforms.", static_transforms.size());
            }
        }

        if (config["boundaries"] && config["boundaries"].IsMap()) {
            bool bounds_loaded = false;
            for (const auto& bound_pair : config["boundaries"]) {
                std::string frame_id = bound_pair.first.as<std::string>();
                YAML::Node bound_node = bound_pair.second;

                if (bound_node["min"] && bound_node["max"] &&
                    bound_node["min"].IsMap() && bound_node["max"].IsMap() &&
                    bound_node["min"]["x"] && bound_node["min"]["y"] && bound_node["min"]["z"] &&
                    bound_node["max"]["x"] && bound_node["max"]["y"] && bound_node["max"]["z"])
                {
                    geometry_msgs::msg::Point min_p, max_p;
                    min_p.x = bound_node["min"]["x"].as<double>();
                    min_p.y = bound_node["min"]["y"].as<double>();
                    min_p.z = bound_node["min"]["z"].as<double>();
                    max_p.x = bound_node["max"]["x"].as<double>();
                    max_p.y = bound_node["max"]["y"].as<double>();
                    max_p.z = bound_node["max"]["z"].as<double>();

                    setBounds(frame_id, min_p, max_p);
                    bounds_loaded = true;

                } else {
                    RCLCPP_WARN(logger_, "Skipping invalid boundaries entry for frame '%s': incorrect format.", frame_id.c_str());
                }
            }
             if(bounds_loaded) {
                try {
                  node_->set_parameter(rclcpp::Parameter("boundaries.initialized", true));
                } catch (const rclcpp::exceptions::ParameterNotDeclaredException &e) {
                  RCLCPP_ERROR(logger_, "Failed to set 'boundaries.initialized' parameter: %s", e.what());
                }
             }
        }

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(logger_, "Failed to parse YAML configuration: %s. Error: %s", yaml_path.c_str(), e.what());
        return ResultStatus::CONFIGURATION_ERROR;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "An unexpected error occurred during configuration loading: %s", e.what());
        return ResultStatus::CONFIGURATION_ERROR;
    }

    RCLCPP_INFO(logger_, "Configuration loaded successfully.");
    return ResultStatus::SUCCESS;
}

void CoordinateTransformer::addTransform(const geometry_msgs::msg::TransformStamped& transform)
{
    if (static_tf_broadcaster_) {
      static_tf_broadcaster_->sendTransform(transform);
      RCLCPP_DEBUG(logger_, "Published provided static transform: %s -> %s", transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
    } else {
      RCLCPP_ERROR(logger_, "StaticTransformBroadcaster is not initialized. Cannot add transform.");
    }
}

void CoordinateTransformer::setBounds(
    const std::string& frame_id,
    const geometry_msgs::msg::Point& min,
    const geometry_msgs::msg::Point& max)
{
    if (min.x > max.x || min.y > max.y || min.z > max.z) {
        RCLCPP_ERROR(logger_, "Invalid bounds for frame '%s': min coordinates must be less than or equal to max coordinates. Bounds not set.", frame_id.c_str());
        return;
    }

    {
      std::lock_guard<std::mutex> lock(bounds_mutex_);
      bounds_[frame_id] = Bounds{min, max};
       RCLCPP_INFO(logger_, "Set bounds for frame '%s': min(%.2f, %.2f, %.2f), max(%.2f, %.2f, %.2f)", frame_id.c_str(), min.x, min.y, min.z, max.x, max.y, max.z);
    }

    std::string prefix = "boundaries." + frame_id + ".";
    try {
        std::map<std::string, double> defaults = {
            {prefix + "min_x", min.x}, {prefix + "min_y", min.y}, {prefix + "min_z", min.z},
            {prefix + "max_x", max.x}, {prefix + "max_y", max.y}, {prefix + "max_z", max.z}
        };
        for(const auto& pair : defaults) {
            if (!node_->has_parameter(pair.first)) {
                node_->declare_parameter(pair.first, pair.second);
            }
        }

        std::vector<rclcpp::Parameter> params_to_set;
        for(const auto& pair : defaults) {
            params_to_set.emplace_back(pair.first, pair.second);
        }
        node_->set_parameters(params_to_set);

    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &e) {
        RCLCPP_WARN(logger_, "Parameters for frame '%s' already declared, setting values directly: %s", frame_id.c_str(), e.what());
         try {
             std::vector<rclcpp::Parameter> params_to_set;
             std::map<std::string, double> current_values;
             current_values[prefix + "min_x"] = min.x; current_values[prefix + "min_y"] = min.y; current_values[prefix + "min_z"] = min.z;
             current_values[prefix + "max_x"] = max.x; current_values[prefix + "max_y"] = max.y; current_values[prefix + "max_z"] = max.z;
             for(const auto& pair : current_values) {
                 params_to_set.emplace_back(pair.first, pair.second);
             }
             node_->set_parameters(params_to_set);
         } catch (const std::exception& set_e) {
             RCLCPP_ERROR(logger_, "Failed to set parameters for frame '%s' after declaration issue: %s", frame_id.c_str(), set_e.what());
         }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to declare/set parameters for frame '%s': %s", frame_id.c_str(), e.what());
    }
}

void CoordinateTransformer::removeBounds(const std::string& frame_id)
{
  bool removed = false;
  {
    std::lock_guard<std::mutex> lock(bounds_mutex_);
    auto it = bounds_.find(frame_id);
    if (it != bounds_.end()) {
      bounds_.erase(it);
      removed = true;
    }
  }

  if (removed) {
    RCLCPP_INFO(logger_, "Removed bounds for frame '%s'.", frame_id.c_str());
  } else {
      RCLCPP_WARN(logger_, "Attempted to remove bounds for frame '%s', but no bounds were set.", frame_id.c_str());
  }
}


ResultStatus CoordinateTransformer::convert(
    const geometry_msgs::msg::PoseStamped& input,
    geometry_msgs::msg::PoseStamped& output,
    const std::string& target_frame) const
{
  RCLCPP_INFO(logger_, "CONVERT: Input frame='%s', Target frame='%s'", input.header.frame_id.c_str(), target_frame.c_str());
    if (input.header.frame_id.empty()) {
        RCLCPP_ERROR(logger_, "Conversion failed: Input pose has empty frame_id.");
        return ResultStatus::INVALID_INPUT;
    }
    if (target_frame.empty()) {
        RCLCPP_ERROR(logger_, "Conversion failed: Target frame is empty.");
        return ResultStatus::INVALID_INPUT;
    }
    if (!tf_buffer_) {
        RCLCPP_ERROR(logger_, "Conversion failed: TF buffer is not initialized.");
        return ResultStatus::CONFIGURATION_ERROR;
    }

    try {
        output = tf_buffer_->transform(input, target_frame, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(logger_, "Could not transform '%s' to '%s': %s", input.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        return ResultStatus::TRANSFORM_NOT_FOUND;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "An unexpected error occurred during TF transformation: %s", e.what());
        return ResultStatus::CONFIGURATION_ERROR;
    }

    if (!checkBounds(output.pose.position, target_frame)) {
        RCLCPP_WARN(logger_, "Transformed point in frame '%s' (%.2f, %.2f, %.2f) is out of defined bounds.", target_frame.c_str(), output.pose.position.x, output.pose.position.y, output.pose.position.z);
        return ResultStatus::OUT_OF_BOUNDS;
    }

    return ResultStatus::SUCCESS;
}


ResultStatus CoordinateTransformer::inverseConvert(
    const geometry_msgs::msg::PoseStamped& input,
    geometry_msgs::msg::PoseStamped& output,
    const std::string& source_frame) const
{
    if (input.header.frame_id.empty()) {
        RCLCPP_ERROR(logger_, "Inverse conversion failed: Input pose has empty frame_id.");
        return ResultStatus::INVALID_INPUT;
    }
    if (source_frame.empty()) {
        RCLCPP_ERROR(logger_, "Inverse conversion failed: Target source frame is empty.");
        return ResultStatus::INVALID_INPUT;
    }
     if (!tf_buffer_) {
        RCLCPP_ERROR(logger_, "Inverse conversion failed: TF buffer is not initialized.");
        return ResultStatus::CONFIGURATION_ERROR;
    }

    try {
        output = tf_buffer_->transform(input, source_frame, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(logger_, "Could not perform inverse transform from '%s' to '%s': %s", input.header.frame_id.c_str(), source_frame.c_str(), ex.what());
        return ResultStatus::TRANSFORM_NOT_FOUND;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "An unexpected error occurred during inverse TF transformation: %s", e.what());
        return ResultStatus::CONFIGURATION_ERROR;
    }

    if (!checkBounds(output.pose.position, source_frame)) {
         RCLCPP_WARN(logger_, "Inverse transformed point in frame '%s' (%.2f, %.2f, %.2f) is out of defined bounds.", source_frame.c_str(), output.pose.position.x, output.pose.position.y, output.pose.position.z);
        return ResultStatus::OUT_OF_BOUNDS;
    }

    return ResultStatus::SUCCESS;
}

const rclcpp::Logger & CoordinateTransformer::getLogger() const {
    return logger_;
}

// --- Приватные методы ---

bool CoordinateTransformer::checkBounds(const geometry_msgs::msg::Point& point, const std::string& frame_id) const
{
    Bounds current_bounds;
    bool found = false;
    {
        std::lock_guard<std::mutex> lock(bounds_mutex_);
        auto it = bounds_.find(frame_id);
        if (it != bounds_.end()) {
            current_bounds = it->second;
            found = true;
        }
    }

    if (!found) {
        return true;
    }

    bool in_bounds = (point.x >= current_bounds.min.x && point.x <= current_bounds.max.x &&
                      point.y >= current_bounds.min.y && point.y <= current_bounds.max.y &&
                      point.z >= current_bounds.min.z && point.z <= current_bounds.max.z);

    return in_bounds;
}

rcl_interfaces::msg::SetParametersResult CoordinateTransformer::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::map<std::string, coordinate_transformer::Bounds> pending_bounds;
    std::set<std::string> frames_to_validate;

    // Шаг 1: Собрать изменения
    {
        std::lock_guard<std::mutex> lock(bounds_mutex_);

        for (const auto& param : parameters) {
            const std::string& name = param.get_name();
            if (name.rfind("boundaries.", 0) == 0) {
                size_t frame_start = 11;
                size_t frame_end = name.find('.', frame_start);
                if (frame_end == std::string::npos || frame_end == frame_start) {
                    RCLCPP_WARN(logger_, "Ignoring invalid boundary parameter name format: %s", name.c_str());
                    continue;
                }
                std::string frame_id = name.substr(frame_start, frame_end - frame_start);
                std::string bound_type = name.substr(frame_end + 1);

                if (pending_bounds.find(frame_id) == pending_bounds.end()) {
                    if (bounds_.count(frame_id)) {
                        pending_bounds[frame_id] = bounds_[frame_id];
                    } else {
                        pending_bounds[frame_id] = Bounds{};
                    }
                }

                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                     RCLCPP_ERROR(logger_, "Invalid type for boundary parameter '%s'. Expected double, got %s. Update rejected.", name.c_str(), param.get_type_name().c_str());
                    result.successful = false;
                    result.reason = "Invalid parameter type for " + name;
                    return result;
                }

                double value = param.as_double();
                if (bound_type == "min_x") pending_bounds[frame_id].min.x = value;
                else if (bound_type == "min_y") pending_bounds[frame_id].min.y = value;
                else if (bound_type == "min_z") pending_bounds[frame_id].min.z = value;
                else if (bound_type == "max_x") pending_bounds[frame_id].max.x = value;
                else if (bound_type == "max_y") pending_bounds[frame_id].max.y = value;
                else if (bound_type == "max_z") pending_bounds[frame_id].max.z = value;
                else {
                    RCLCPP_WARN(logger_, "Ignoring unknown boundary parameter suffix: %s", name.c_str());
                    continue;
                }
                frames_to_validate.insert(frame_id);
            }
        }
    }

    // Шаг 2: Проверить логику
    for (const auto& frame_id : frames_to_validate) {
        const auto& current_pending = pending_bounds[frame_id];
        if (current_pending.min.x > current_pending.max.x ||
            current_pending.min.y > current_pending.max.y ||
            current_pending.min.z > current_pending.max.z)
        {
            RCLCPP_ERROR(logger_, "Invalid bounds logic for frame '%s' after parameter update: min > max. Update rejected.", frame_id.c_str());
            result.successful = false;
            result.reason = "Invalid bounds logic (min > max) for frame " + frame_id;
            return result;
        }
    }

    // Шаг 3: Применить изменения
    if (result.successful) {
        std::lock_guard<std::mutex> lock(bounds_mutex_);
        for (const auto& pair : pending_bounds) {
            const std::string& frame_id = pair.first;
            if (frames_to_validate.count(frame_id)) {
                 bounds_[frame_id] = pair.second;
                 RCLCPP_INFO(logger_, "Updated bounds for frame '%s' via parameters.", frame_id.c_str());
            }
        }
    }

    return result;
}

} // namespace coordinate_transformer