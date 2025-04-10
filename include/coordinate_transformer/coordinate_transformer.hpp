/**
 * @file coordinate_transformer.hpp
 * @brief Заголовочный файл для класса CoordinateTransformer, отвечающего за преобразования координат и управление границами.
 * @author Ruslan Mukhametsafin
 * @date   2025-04-10
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>

#include "coordinate_transformer/result_status.hpp" 


class TestCoordinateTransformer;

namespace coordinate_transformer
{

struct Bounds
{
  geometry_msgs::msg::Point min;
  geometry_msgs::msg::Point max;
};


class CoordinateTransformer
{
  friend class ::TestCoordinateTransformer; 

public:
  /**
   * @brief Конструктор по умолчанию.
   * @details Инициализирует CoordinateTransformer, используя переданный узел ROS 2.
   *          Создает внутренний буфер TF2 и слушатель трансформаций.
   * @param node Умный указатель на узел ROS 2. Не должен быть nullptr.
   * @throws std::invalid_argument если node равен nullptr.
   */
  explicit CoordinateTransformer(rclcpp::Node::SharedPtr node);

  /**
   * @brief Конструктор для юнит-тестирования с инъекцией зависимостей.
   * @details Инициализирует CoordinateTransformer, используя переданный узел ROS 2 и
   *          предоставленный буфер TF2. Не создает внутреннего слушателя TF2.
   * @param node Умный указатель на узел ROS 2. Не должен быть nullptr.
   * @param tf_buffer Умный указатель на буфер TF2 (например, мок-объект). Не должен быть nullptr.
   * @throws std::invalid_argument если node или tf_buffer равны nullptr.
   */
  CoordinateTransformer(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /**
   * @brief Загружает конфигурацию из YAML-файла.
   * @details Загружает статические трансформации и определения границ из указанного файла.
   *          Загруженные статические трансформации публикуются через StaticTransformBroadcaster.
   *          Загруженные границы сохраняются внутри объекта и объявляются/устанавливаются как ROS-параметры.
   * @param yaml_path Путь к файлу конфигурации в формате YAML.
   * @return Статус операции: SUCCESS в случае успеха, CONFIGURATION_ERROR при ошибке.
   */
  ResultStatus loadConfig(const std::string& yaml_path);

  /**
   * @brief Добавляет и публикует единичную статическую трансформацию.
   * @details Публикует переданную трансформацию через StaticTransformBroadcaster.
   * @param transform Сообщение geometry_msgs::msg::TransformStamped, содержащее трансформацию.
   */
  void addTransform(const geometry_msgs::msg::TransformStamped& transform);

  /**
   * @brief Устанавливает или обновляет границы для заданной системы координат.
   * @details Сохраняет переданные минимальные и максимальные координаты для указанного frame_id.
   *          Также объявляет и устанавливает соответствующие ROS-параметры для динамического управления.
   *          Если min > max по любой оси, границы не устанавливаются и выводится ошибка.
   * @param frame_id Идентификатор системы координат (TF frame ID).
   * @param min Структура geometry_msgs::msg::Point, содержащая минимальные координаты (x, y, z).
   * @param max Структура geometry_msgs::msg::Point, содержащая максимальные координаты (x, y, z).
   */
  void setBounds(
    const std::string& frame_id,
    const geometry_msgs::msg::Point& min,
    const geometry_msgs::msg::Point& max);

  /**
   * @brief Удаляет ранее установленные границы для указанной системы координат.
   * @details Если для frame_id были установлены границы, они удаляются из внутреннего хранилища.
   *          Соответствующие ROS-параметры не удаляются, но и не используются после вызова этого метода.
   * @param frame_id Идентификатор системы координат (TF frame ID), для которой удаляются границы.
   */
  void removeBounds(const std::string& frame_id);

  /**
   * @brief Преобразует позу из исходной системы координат в целевую.
   * @details Использует буфер TF2 для выполнения преобразования.
   *          Проверяет, находится ли результирующая точка в пределах границ, установленных для target_frame.
   * @param input Входная поза (PoseStamped), frame_id которой указывает исходную систему координат.
   * @param output Выходная поза (PoseStamped), в которую будет записан результат преобразования в системе target_frame.
   * @param target_frame Идентификатор целевой системы координат (TF frame ID).
   * @return Статус операции: SUCCESS, OUT_OF_BOUNDS, TRANSFORM_NOT_FOUND, INVALID_INPUT или CONFIGURATION_ERROR.
   */
  ResultStatus convert(
    const geometry_msgs::msg::PoseStamped& input,
    geometry_msgs::msg::PoseStamped& output,
    const std::string& target_frame) const; 

  /**
   * @brief Преобразует позу из текущей системы координат в заданную исходную систему.
   * @details Использует буфер TF2 для выполнения преобразования из input.header.frame_id в source_frame.
   *          Проверяет, находится ли результирующая точка в пределах границ, установленных для source_frame.
   * @param input Входная поза (PoseStamped) в некоторой системе координат.
   * @param output Выходная поза (PoseStamped), в которую будет записан результат преобразования в системе source_frame.
   * @param source_frame Идентификатор системы координат (TF frame ID), в которую нужно преобразовать.
   * @return Статус операции: SUCCESS, OUT_OF_BOUNDS, TRANSFORM_NOT_FOUND, INVALID_INPUT или CONFIGURATION_ERROR.
   */
  ResultStatus inverseConvert(
    const geometry_msgs::msg::PoseStamped& input,
    geometry_msgs::msg::PoseStamped& output,
    const std::string& source_frame) const; 

  /**
   * @brief Возвращает логгер узла.
   * @details Предоставляет доступ к логгеру, используемому объектом CoordinateTransformer, для вывода сообщений.
   * @return Константная ссылка на объект rclcpp::Logger.
   */
  const rclcpp::Logger & getLogger() const; 

private:
  /**
   * @brief Внутренний метод для инициализации общих компонентов.
   * @details Вызывается из обоих конструкторов. Инициализирует логгер, буфер TF2 (если не предоставлен),
   *          слушатель TF2 (если буфер не предоставлен), статический публикатор TF и колбэк параметров.
   * @param tf_buffer Умный указатель на буфер TF2. Если nullptr, создается внутренний буфер и слушатель.
   */
  void initialize(std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /**
   * @brief Проверяет, находится ли точка в пределах границ для данной системы координат.
   * @details Получает текущие границы для frame_id (если они установлены) и сравнивает с координатами точки.
   *          Метод потокобезопасен благодаря использованию мьютекса bounds_mutex_.
   * @param point Точка (Point), которую нужно проверить.
   * @param frame_id Идентификатор системы координат (TF frame ID), для которой проверяются границы.
   * @return `true`, если точка находится внутри установленных границ или если границы для данной системы координат не заданы.
   *         `false`, если точка находится вне установленных границ.
   */
  bool checkBounds(const geometry_msgs::msg::Point& point, const std::string& frame_id) const; 

  /**
   * @brief Колбэк для обработки изменений ROS-параметров.
   * @details Вызывается при попытке изменить параметры узла, связанные с границами (префикс 'boundaries.').
   *          Проверяет корректность новых значений (min <= max) и обновляет внутреннее хранилище границ.
   * @param parameters Вектор предлагаемых изменений параметров.
   * @return Структура SetParametersResult, указывающая на успех или неудачу применения изменений.
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters);

  rclcpp::Node::SharedPtr node_; 
  rclcpp::Logger logger_;      
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_; 
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; 
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_; 

  std::map<std::string, Bounds> bounds_;
  
  mutable std::mutex bounds_mutex_; 

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

}; // class CoordinateTransformer

} // namespace coordinate_transformer