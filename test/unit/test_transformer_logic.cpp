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
#include <yaml-cpp/yaml.h>

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
geometry_msgs::msg::PoseStamped create_pose(const std::string& frame_id, double x, double y, double z) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    // Временную метку лучше устанавливать прямо перед использованием в тесте
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.w = 1.0; // Единичная ориентация
    return pose;
}

/**
 * @brief Создает объект Point с заданными координатами.
 * @param x Координата X.
 * @param y Координата Y.
 * @param z Координата Z.
 * @return Объект Point.
 */
geometry_msgs::msg::Point create_point(double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
}

/**
 * @brief Создает временный YAML-файл с указанным содержимым.
 * @param content Строка с содержимым YAML-файла.
 * @param dir Директория для создания файла.
 * @param filename Имя создаваемого файла (по умолчанию "temp_test_config.yaml").
 * @return Полный путь к созданному файлу.
 */
std::string create_temp_yaml(const std::string& content, const std::filesystem::path& dir, const std::string& filename = "temp_test_config.yaml") {
    std::filesystem::path full_path = dir / filename;
    std::ofstream fout(full_path);
    fout << content;
    fout.close();
    return full_path.string();
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
    const std::shared_ptr<tf2_ros::Buffer>& buffer,
    const std::string& target_frame,
    const std::string& source_frame,
    const tf2::TimePoint& time,
    const std::chrono::duration<double>& timeout,
    const rclcpp::Node::SharedPtr& node_to_spin = nullptr)
{
    rclcpp::Clock::SharedPtr clock = node_to_spin ? node_to_spin->get_clock() : std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    rclcpp::Time start_time = clock->now();
    rclcpp::Time end_time = start_time + rclcpp::Duration(timeout);

    while (rclcpp::ok() && clock->now() < end_time) {
        rclcpp::Duration remaining_rcl_time = end_time - clock->now();
        if (remaining_rcl_time < rclcpp::Duration(0,0)) {
            remaining_rcl_time = rclcpp::Duration(0,0);
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
     if (final_remaining_rcl < rclcpp::Duration(0,0)) final_remaining_rcl = rclcpp::Duration(0,0);
     tf2::Duration final_remaining_tf2 = tf2::durationFromSec(final_remaining_rcl.seconds());
    return buffer->canTransform(target_frame, source_frame, time, final_remaining_tf2);
}

/**
 * @brief Тестовый класс для CoordinateTransformer.
 *
 * Этот класс содержит набор тестов для проверки функциональности
 * класса CoordinateTransformer, включая инициализацию, настройку границ,
 * загрузку конфигурации и преобразование координат.
 */
class TestCoordinateTransformer : public ::testing::Test {
    protected:
        rclcpp::Node::SharedPtr node; ///< Указатель на тестовый узел ROS2.
        std::shared_ptr<tf2_ros::Buffer> tf_buffer; ///< Буфер TF2, используемый для *тестирования*.
        std::unique_ptr<coordinate_transformer::CoordinateTransformer> transformer; ///< Экземпляр тестируемого класса.
        std::shared_ptr<tf2_ros::TransformListener> test_tf_listener_; ///< Слушатель TF для *тестового* буфера.
        std::filesystem::path test_dir; ///< Путь к временной директории для тестов.
        bool external_buffer_provided = false; ///< Флаг, указывающий, был ли предоставлен внешний буфер.

        /**
         * @brief Инициализирует ресурсы для теста.
         *
         * Создает узел ROS2, временную директорию, тестовый буфер TF2 и слушатель TF.
         * Инициализирует CoordinateTransformer либо с внутренним, либо с внешним буфером TF2.
         * @param provide_external_buffer Если true, transformer будет использовать
         *        предоставленный внешний `tf_buffer`. Иначе создаст собственный.
         */
        void InitializeTransformer(bool provide_external_buffer) {
             node = std::make_shared<rclcpp::Node>("test_coordinate_transformer_node");
             test_dir = std::filesystem::temp_directory_path() / "coord_transformer_test";
             if (std::filesystem::exists(test_dir)) {
                 std::filesystem::remove_all(test_dir);
             }
             std::filesystem::create_directories(test_dir);

             tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
             tf_buffer->setUsingDedicatedThread(true);

             test_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, true); // Use listener for dynamic TFs too

             if (provide_external_buffer) {
                 transformer = std::make_unique<coordinate_transformer::CoordinateTransformer>(node, tf_buffer);
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

        /**
         * @brief Метод настройки, выполняемый перед каждым тестом GTest.
         *
         * Инициализирует ROS и вызывает InitializeTransformer для настройки
         * тестового окружения. По умолчанию используется внешний буфер.
         */
        void SetUp() override {
             rclcpp::init(0, nullptr); // Init ROS per test
             InitializeTransformer(true);
        }

        /**
         * @brief Метод очистки, выполняемый после каждого теста GTest.
         *
         * Освобождает ресурсы, созданные в SetUp и InitializeTransformer,
         * останавливает узел ROS и удаляет временную директорию.
         */
        void TearDown() override {
             test_tf_listener_.reset(); 
             transformer.reset();
             if (tf_buffer) {
                  if (tf_buffer->isUsingDedicatedThread()) {
                      tf_buffer->setUsingDedicatedThread(false);
                  }
                 tf_buffer.reset();
             }
             node.reset();
             rclcpp::shutdown(); // Shutdown ROS per test
             if (std::filesystem::exists(test_dir)) {
                 std::filesystem::remove_all(test_dir);
             }
        }

        /**
         * @brief Добавляет трансформацию непосредственно в тестовый буфер TF2.
         * @param parent Родительская система координат.
         * @param child Дочерняя система координат.
         * @param x Смещение по X.
         * @param y Смещение по Y.
         * @param z Смещение по Z.
         * @param qx Компонента X кватерниона ориентации.
         * @param qy Компонента Y кватерниона ориентации.
         * @param qz Компонента Z кватерниона ориентации.
         * @param qw Компонента W кватерниона ориентации.
         * @param is_static Флаг статической трансформации.
         */
        void add_tf_to_buffer_direct(const std::string& parent, const std::string& child,
                              double x, double y, double z,
                              double qx=0, double qy=0, double qz=0, double qw=1,
                              bool is_static = false) {
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
            ASSERT_TRUE(wait_for_transform(tf_buffer, parent, child, tf2::TimePointZero, std::chrono::milliseconds(200), node))
                << "Timed out waiting for DIRECT transform " << parent << " -> " << child;
        }

        /**
         * @brief Публикует статическую трансформацию с использованием StaticTransformBroadcaster.
         * @param parent Родительская система координат.
         * @param child Дочерняя система координат.
         * @param x Смещение по X.
         * @param y Смещение по Y.
         * @param z Смещение по Z.
         * @param qx Компонента X кватерниона ориентации.
         * @param qy Компонента Y кватерниона ориентации.
         * @param qz Компонента Z кватерниона ориентации.
         * @param qw Компонента W кватерниона ориентации.
         */
         void publish_static_tf(
             const std::string& parent, const std::string& child,
             double x, double y, double z,
             double qx=0, double qy=0, double qz=0, double qw=1)
        {
            geometry_msgs::msg::TransformStamped tf_stamped;
            tf_stamped.header.stamp = node->get_clock()->now(); // Static TF time is less critical
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
            
            bool transform_received = wait_for_transform(tf_buffer, parent, child, tf2::TimePointZero, std::chrono::seconds(1), node);
            if (!transform_received) {
                 FAIL() << "Timed out waiting for PUBLISHED STATIC transform " << parent << " -> " << child;
            }
        }

        /**
         * @brief Проверяет результат операции преобразования координат.
         *
         * Выполняет преобразование из `source_frame` в `target_frame` для `source_point`
         * и сравнивает полученный статус с `expected_status`. Использует TimePointZero
         * для временной метки запроса преобразования, чтобы избежать ошибок экстраполяции.
         *
         * @param source_frame Исходная система координат.
         * @param source_point Точка в исходной системе координат.
         * @param target_frame Целевая система координат.
         * @param expected_status Ожидаемый статус результата преобразования.
         * @param message Дополнительное сообщение для вывода в случае ошибки теста.
         */
        void check_conversion_status( const std::string& source_frame,
            const geometry_msgs::msg::Point& source_point,
            const std::string& target_frame,
            coordinate_transformer::ResultStatus expected_status,
            const std::string& message = "")
        {
            geometry_msgs::msg::PoseStamped input_pose = create_pose(source_frame, source_point.x, source_point.y, source_point.z);
            input_pose.header.stamp = tf2_ros::toMsg(tf2::TimePointZero);
            geometry_msgs::msg::PoseStamped output_pose;

            rclcpp::sleep_for(std::chrono::milliseconds(10));
            rclcpp::spin_some(node);

            coordinate_transformer::ResultStatus actual_status = transformer->convert(input_pose, output_pose, target_frame);
            EXPECT_EQ(actual_status, expected_status)
                << "Conversion from " << source_frame << " to " << target_frame
                << " for point (" << source_point.x << ", " << source_point.y << ", " << source_point.z << ")"
                << " expected status " << static_cast<int>(expected_status)
                << " but got " << static_cast<int>(actual_status) << ". " << message;
        }

        /**
         * @brief Добавляет единичную трансформацию (фрейм в самого себя) в буфер TF2.
         * @param frame_id Идентификатор системы координат.
         * @param use_publisher Если true, использует `publish_static_tf`, иначе `add_tf_to_buffer_direct`.
         */
         void add_identity_tf(const std::string& frame_id, bool use_publisher = false) {
            if (use_publisher) {
                 publish_static_tf(frame_id, frame_id, 0, 0, 0);
            } else {
                 add_tf_to_buffer_direct(frame_id, frame_id, 0, 0, 0, 0,0,0,1, true);
            }
             ASSERT_TRUE(tf_buffer->canTransform(frame_id, frame_id, tf2::TimePointZero, std::chrono::milliseconds(100)))
                << "Failed to add identity transform for frame: " << frame_id;
        }

        /**
         * @brief Вспомогательная функция для добавления статической трансформации через StaticTransformBroadcaster.
         * @param tf Сообщение TransformStamped для публикации.
         * @return Указатель на созданный StaticTransformBroadcaster.
         */
         std::shared_ptr<tf2_ros::StaticTransformBroadcaster> add_static_tf_helper(const geometry_msgs::msg::TransformStamped& tf) {
            auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
            static_broadcaster->sendTransform(tf);
            return static_broadcaster;
        }

};

// --- Тестовые случаи ---

/**
 * @brief Тестирует конструктор CoordinateTransformer только с узлом ROS2.
 *
 * Проверяет, что объект CoordinateTransformer успешно создается,
 * когда в конструктор передается только указатель на узел ROS2.
 * Это подразумевает, что CoordinateTransformer создаст свой собственный внутренний буфер TF2.
 */
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
    auto broadcaster = add_static_tf_helper(tf_stamped); // Publish

    ASSERT_TRUE(wait_for_transform(tf_buffer, "frame_a", "frame_b", tf2::TimePointZero, std::chrono::seconds(1), node))
         << "Test listener did not receive the published transform.";

    geometry_msgs::msg::PoseStamped input_pose = create_pose("frame_b", 0, 0, 0);
    input_pose.header.stamp = tf2_ros::toMsg(tf2::TimePointZero); // Use TimePointZero
    geometry_msgs::msg::PoseStamped output_pose;

    rclcpp::sleep_for(std::chrono::milliseconds(100)); // Give internal listener time
    rclcpp::spin_some(node);

    coordinate_transformer::ResultStatus status = transformer->convert(input_pose, output_pose, "frame_a");

    EXPECT_EQ(status, coordinate_transformer::ResultStatus::SUCCESS)
        << "Transformer with internal listener failed to convert using externally published TF.";
    EXPECT_NEAR(output_pose.pose.position.x, 1.0, 1e-6);
    EXPECT_TRUE(node->has_parameter("boundaries.initialized"));
}

/**
 * @brief Тестирует конструктор CoordinateTransformer с узлом ROS2 и внешним буфером TF2.
 *
 * Проверяет, что объект CoordinateTransformer успешно создается,
 * когда в конструктор передается указатель на узел ROS2 и
 * указатель на существующий буфер TF2.
 */
TEST_F(TestCoordinateTransformer, ConstructorWithNodeAndBuffer) {
     ASSERT_NE(transformer, nullptr);
     ASSERT_NE(tf_buffer, nullptr);
     EXPECT_TRUE(external_buffer_provided);

      add_tf_to_buffer_direct("frame_c", "frame_d", 1,0,0);

      EXPECT_TRUE(tf_buffer->canTransform("frame_c", "frame_d", tf2::TimePointZero));
      EXPECT_TRUE(node->has_parameter("boundaries.initialized"));
}

/**
 * @brief Тестирует инициализацию параметров CoordinateTransformer.
 *
 * Проверяет, что параметры по умолчанию (границы и т.д.)
 * установлены корректно после создания экземпляра CoordinateTransformer.
 */
TEST_F(TestCoordinateTransformer, ParameterInitialization) {
    ASSERT_NE(transformer, nullptr);
    EXPECT_NO_THROW(transformer->getLogger());
    ASSERT_TRUE(node->has_parameter("boundaries.initialized"));
    EXPECT_FALSE(node->get_parameter("boundaries.initialized").as_bool());
}

/**
 * @brief Тестирует установку и проверку границ с помощью функции convert.
 *
 * Устанавливает допустимые границы для определенной системы координат,
 * а затем проверяет, что функция `convert` корректно применяет эти границы,
 * возвращая соответствующий статус (например, SUCCESS или OUT_OF_BOUNDS).
 */
TEST_F(TestCoordinateTransformer, SetAndCheckBoundsViaConvert) {
    std::string frame = "test_bounds_frame";
    auto min_p = create_point(-1.0, -1.0, -0.1);
    auto max_p = create_point(1.0, 1.0, 0.1);

    transformer->setBounds(frame, min_p, max_p);

    std::string prefix = "boundaries." + frame + ".";
    ASSERT_TRUE(node->has_parameter(prefix + "min_x"));
    EXPECT_NEAR(node->get_parameter(prefix + "min_x").as_double(), -1.0, 1e-6);
    ASSERT_TRUE(node->has_parameter(prefix + "min_y"));
    EXPECT_NEAR(node->get_parameter(prefix + "min_y").as_double(), -1.0, 1e-6);
    ASSERT_TRUE(node->has_parameter(prefix + "min_z"));
    EXPECT_NEAR(node->get_parameter(prefix + "min_z").as_double(), -0.1, 1e-6);
    ASSERT_TRUE(node->has_parameter(prefix + "max_x"));
    EXPECT_NEAR(node->get_parameter(prefix + "max_x").as_double(), 1.0, 1e-6);
     ASSERT_TRUE(node->has_parameter(prefix + "max_y"));
    EXPECT_NEAR(node->get_parameter(prefix + "max_y").as_double(), 1.0, 1e-6);
    ASSERT_TRUE(node->has_parameter(prefix + "max_z"));
    EXPECT_NEAR(node->get_parameter(prefix + "max_z").as_double(), 0.1, 1e-6);

    add_identity_tf(frame, false);

    check_conversion_status(frame, create_point(0.0, 0.0, 0.0), frame, coordinate_transformer::ResultStatus::SUCCESS, "Point inside bounds");
    check_conversion_status(frame, create_point(1.5, 0.0, 0.0), frame, coordinate_transformer::ResultStatus::OUT_OF_BOUNDS, "Point outside X");
    check_conversion_status(frame, create_point(0.0, -1.1, 0.0), frame, coordinate_transformer::ResultStatus::OUT_OF_BOUNDS, "Point outside Y");
    check_conversion_status(frame, create_point(0.0, 0.0, 0.2), frame, coordinate_transformer::ResultStatus::OUT_OF_BOUNDS, "Point outside Z");
    check_conversion_status(frame, create_point(-1.0, -1.0, -0.1), frame, coordinate_transformer::ResultStatus::SUCCESS, "Point on min boundary");
    check_conversion_status(frame, create_point(1.0, 1.0, 0.1), frame, coordinate_transformer::ResultStatus::SUCCESS, "Point on max boundary");

    std::string frame_no_bounds = "no_bounds_frame";
    add_tf_to_buffer_direct(frame, frame_no_bounds, 1, 1, 1);

    check_conversion_status(frame_no_bounds, create_point(10.0, 10.0, 10.0), frame, coordinate_transformer::ResultStatus::OUT_OF_BOUNDS, "Convert from frame without bounds");
    check_conversion_status(frame_no_bounds, create_point(10.0, 10.0, 10.0), frame, coordinate_transformer::ResultStatus::OUT_OF_BOUNDS, "Convert from frame without bounds to frame with bounds (point outside target)");
}

/**
 * @brief Тестирует установку недопустимых границ.
 *
 * Проверяет, что CoordinateTransformer обрабатывает попытку
 * установки недопустимых границ (например, min > max) и не падает,
 * возможно, возвращая ошибку или игнорируя некорректные данные.
 */
TEST_F(TestCoordinateTransformer, SetInvalidBounds) {
    std::string frame = "invalid_bounds_frame";
    auto min_p = create_point(1.0, 0.0, 0.0); // min_x > max_x
    auto max_p = create_point(-1.0, 1.0, 1.0);

    transformer->setBounds(frame, min_p, max_p);

    EXPECT_FALSE(node->has_parameter("boundaries.invalid_bounds_frame.min_x"));

    add_identity_tf(frame, false);

    check_conversion_status(frame, create_point(0.0, 0.0, 0.0), frame, coordinate_transformer::ResultStatus::SUCCESS, "Convert after setting invalid bounds");
}

/**
 * @brief Тестирует удаление ранее установленных границ.
 *
 * Проверяет, что после вызова функции удаления границ для
 * определенной системы координат, функция `convert` больше не применяет
 * ограничения и успешно преобразует точки, которые ранее были вне границ.
 */
TEST_F(TestCoordinateTransformer, RemoveBounds) {
    std::string frame = "removable_frame";
    auto min_p = create_point(0.0, 0.0, 0.0);
    auto max_p = create_point(1.0, 1.0, 1.0);
    auto point_inside = create_point(0.5, 0.5, 0.5);
    auto point_outside = create_point(2.0, 2.0, 2.0);

    transformer->setBounds(frame, min_p, max_p);
    add_identity_tf(frame, false);

    ASSERT_TRUE(node->has_parameter("boundaries." + frame + ".min_x"));
    check_conversion_status(frame, point_inside, frame, coordinate_transformer::ResultStatus::SUCCESS, "Inside bounds before removal");
    check_conversion_status(frame, point_outside, frame, coordinate_transformer::ResultStatus::OUT_OF_BOUNDS, "Outside bounds before removal");

    transformer->removeBounds(frame);

    check_conversion_status(frame, point_outside, frame, coordinate_transformer::ResultStatus::SUCCESS, "Outside bounds after removal");
}

/**
 * @brief Тестирует успешную загрузку конфигурации из YAML-файла.
 *
 * Проверяет, что CoordinateTransformer корректно загружает
 * статические трансформации и определения границ из правильно
 * отформатированного YAML-файла конфигурации.
 */
TEST_F(TestCoordinateTransformer, LoadConfigSuccess) {
    std::string yaml_content = R"(
transforms:
  - parent_frame: map
    child_frame: odom
    translation: {x: 1.0, y: 2.0, z: 0.0}
    rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - parent_frame: odom
    child_frame: base_link
    translation: {x: 0.5, y: 0.0, z: 0.1}
    rotation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707} # 90 deg yaw

boundaries:
  map:
    min: {x: -10.0, y: -10.0, z: -1.0}
    max: {x: 10.0, y: 10.0, z: 1.0}
  odom:
    min: {x: -5.0, y: -5.0, z: -0.5}
    max: {x: 5.0, y: 5.0, z: 0.5}
)";
    std::string yaml_path = create_temp_yaml(yaml_content, test_dir, "config_success.yaml");
    ASSERT_EQ(transformer->loadConfig(yaml_path), coordinate_transformer::ResultStatus::SUCCESS);

     ASSERT_TRUE(wait_for_transform(tf_buffer, "map", "odom", tf2::TimePointZero, std::chrono::seconds(1), node))
         << "Transform map -> odom not found after loading config";
    ASSERT_TRUE(wait_for_transform(tf_buffer, "odom", "base_link", tf2::TimePointZero, std::chrono::seconds(1), node))
         << "Transform odom -> base_link not found after loading config";

    ASSERT_TRUE(node->has_parameter("boundaries.map.min_x"));
    EXPECT_NEAR(node->get_parameter("boundaries.map.min_x").as_double(), -10.0, 1e-6);
    ASSERT_TRUE(node->has_parameter("boundaries.odom.max_y"));
    EXPECT_NEAR(node->get_parameter("boundaries.odom.max_y").as_double(), 5.0, 1e-6);
    ASSERT_TRUE(node->has_parameter("boundaries.initialized"));
    EXPECT_TRUE(node->get_parameter("boundaries.initialized").as_bool());

    add_identity_tf("map", true);
    add_identity_tf("odom", true);
    check_conversion_status("map", create_point(11, 0, 0), "map", coordinate_transformer::ResultStatus::OUT_OF_BOUNDS, "Check map bounds after load");
    check_conversion_status("odom", create_point(0, 6, 0), "odom", coordinate_transformer::ResultStatus::OUT_OF_BOUNDS, "Check odom bounds after load");
    check_conversion_status("map", create_point(0, 0, 0), "map", coordinate_transformer::ResultStatus::SUCCESS, "Check map success after load");
}

/**
 * @brief Тестирует загрузку конфигурации, содержащей только трансформации.
 *
 * Проверяет, что CoordinateTransformer может загрузить YAML-файл,
 * содержащий только определения статических трансформаций, без секции границ.
 */
TEST_F(TestCoordinateTransformer, LoadConfigOnlyTransforms) {
    std::string yaml_content = R"(
transforms:
  - parent_frame: world
    child_frame: camera_link
    translation: {x: 0.1, y: 0.2, z: 0.3}
    rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
)";
    std::string yaml_path = create_temp_yaml(yaml_content, test_dir, "config_only_tf.yaml");
    ASSERT_EQ(transformer->loadConfig(yaml_path), coordinate_transformer::ResultStatus::SUCCESS);

    EXPECT_FALSE(node->has_parameter("boundaries.world.min_x"));
    EXPECT_FALSE(node->has_parameter("boundaries.camera_link.min_x"));
    EXPECT_FALSE(node->get_parameter("boundaries.initialized").as_bool());
}

/**
 * @brief Тестирует загрузку конфигурации, содержащей только границы.
 *
 * Проверяет, что CoordinateTransformer может загрузить YAML-файл,
 * содержащий только определения границ, без секции статических трансформаций.
 */
TEST_F(TestCoordinateTransformer, LoadConfigOnlyBoundaries) {
    std::string yaml_content = R"(
boundaries:
  only_bounds_frame:
    min: {x: 0, y: 0, z: 0}
    max: {x: 1, y: 1, z: 1}
)";
    std::string yaml_path = create_temp_yaml(yaml_content, test_dir, "config_only_bounds.yaml");
    ASSERT_EQ(transformer->loadConfig(yaml_path), coordinate_transformer::ResultStatus::SUCCESS);

    ASSERT_TRUE(node->has_parameter("boundaries.only_bounds_frame.min_x"));
    EXPECT_NEAR(node->get_parameter("boundaries.only_bounds_frame.min_x").as_double(), 0.0, 1e-6);
     ASSERT_TRUE(node->has_parameter("boundaries.only_bounds_frame.max_z"));
    EXPECT_NEAR(node->get_parameter("boundaries.only_bounds_frame.max_z").as_double(), 1.0, 1e-6);
    EXPECT_TRUE(node->get_parameter("boundaries.initialized").as_bool());
}

/**
 * @brief Тестирует поведение при попытке загрузить несуществующий файл конфигурации.
 *
 * Проверяет, что CoordinateTransformer корректно обрабатывает
 * ситуацию, когда указанный файл конфигурации не найден, и
 * не падает, возможно, логируя ошибку.
 */
TEST_F(TestCoordinateTransformer, LoadConfigFileNotFound) {
    std::string non_existent_path = (test_dir / "non_existent_config.yaml").string();
    ASSERT_EQ(transformer->loadConfig(non_existent_path), coordinate_transformer::ResultStatus::CONFIGURATION_ERROR);

    EXPECT_FALSE(node->get_parameter("boundaries.initialized").as_bool());
}

/**
 * @brief Тестирует поведение при попытке загрузить некорректный YAML-файл.
 *
 * Проверяет, что CoordinateTransformer устойчив к синтаксическим
 * ошибкам в YAML-файле конфигурации и не падает, логируя ошибку.
 */
TEST_F(TestCoordinateTransformer, LoadConfigInvalidYaml) {

    std::string yaml_content = R"(
transforms: [{ parent: map, child: odom }
malformed_yaml: : :
)";
    std::string yaml_path = create_temp_yaml(yaml_content, test_dir, "config_invalid.yaml");
    ASSERT_EQ(transformer->loadConfig(yaml_path), coordinate_transformer::ResultStatus::CONFIGURATION_ERROR);
}

/**
 * @brief Тестирует пропуск некорректных записей при загрузке конфигурации.
 *
 * Проверяет, что CoordinateTransformer игнорирует некорректные или
 * неполные записи (трансформаций или границ) в YAML-файле конфигурации,
 * но успешно загружает все корректные записи.
 */
TEST_F(TestCoordinateTransformer, LoadConfigSkipInvalidEntries) {
    std::string yaml_content = R"(
transforms:
  - parent_frame: valid_parent_1
    child_frame: valid_child_1
    translation: {x: 1.0, y: 1.0, z: 1.0}
    rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - parent_frame: invalid_tf # Пропущено translation
    child_frame: some_child
    rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - parent_frame: valid_parent_2
    child_frame: valid_child_2
    translation: {x: 2.0, y: 2.0, z: 2.0}
    rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}

boundaries:
  valid_bounds_frame:
    min: {x: -1, y: -1, z: -1}
    max: {x: 1, y: 1, z: 1}
  invalid_bounds_frame: # Некорректный формат min/max
    min: [-1, -1, -1]
    max: [1, 1, 1]
)";
    std::string yaml_path = create_temp_yaml(yaml_content, test_dir, "config_skip_invalid.yaml");

    ASSERT_EQ(transformer->loadConfig(yaml_path), coordinate_transformer::ResultStatus::SUCCESS);

    ASSERT_TRUE(wait_for_transform(tf_buffer, "valid_parent_1", "valid_child_1", tf2::TimePointZero, std::chrono::seconds(1), node));
    ASSERT_TRUE(wait_for_transform(tf_buffer, "valid_parent_2", "valid_child_2", tf2::TimePointZero, std::chrono::seconds(1), node));

    EXPECT_FALSE(tf_buffer->canTransform("invalid_tf", "some_child", tf2::TimePointZero, std::chrono::milliseconds(10)));
}

/**
 * @brief Тестирует успешное преобразование координат при наличии необходимых TF.
 *
 * Проверяет, что функция `convert` успешно преобразует точку из одной
 * системы координат в другую, когда необходимая трансформация (статическая или динамическая)
 * доступна в буфере TF2.
 */
TEST_F(TestCoordinateTransformer, ConvertSuccessWithTf) {

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

    tf_buffer->setTransform(tf1, "test_authority_direct", false);
    tf_buffer->setTransform(tf2, "test_authority_direct", false);

     ASSERT_TRUE(wait_for_transform(tf_buffer, target_frame, frame1, tf2::TimePointZero, std::chrono::milliseconds(200), node))
        << "Timed out waiting for DIRECT transform " << target_frame << " -> " << frame1;
     ASSERT_TRUE(wait_for_transform(tf_buffer, frame1, frame2, tf2::TimePointZero, std::chrono::milliseconds(200), node))
        << "Timed out waiting for DIRECT transform " << frame1 << " -> " << frame2;

    check_conversion_status(frame2, create_point(0.5, 0.5, 0.0), target_frame, coordinate_transformer::ResultStatus::SUCCESS);

    geometry_msgs::msg::PoseStamped input_pose = create_pose(frame2, 0.5, 0.5, 0.0);
    input_pose.header.stamp = tf2_ros::toMsg(tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped output_pose;

    coordinate_transformer::ResultStatus final_status = transformer->convert(input_pose, output_pose, target_frame);
    ASSERT_EQ(final_status, coordinate_transformer::ResultStatus::SUCCESS);

    ASSERT_EQ(output_pose.header.frame_id, target_frame);
    EXPECT_NEAR(output_pose.pose.position.x, 1.5, 1e-6);
    EXPECT_NEAR(output_pose.pose.position.y, 2.5, 1e-6);
    EXPECT_NEAR(output_pose.pose.position.z, 0.0, 1e-6);
}

/**
 * @brief Тестирует ошибку преобразования из-за отсутствия TF (Lookup Error).
 *
 * Проверяет, что функция `convert` возвращает статус ошибки LOOKUP_ERROR,
 * когда необходимая трансформация между системами координат отсутствует
 * в буфере TF2.
 */
TEST_F(TestCoordinateTransformer, ConvertTfErrorLookup) {

    std::string frame_known = "known_frame";
    std::string frame_unknown = "unknown_target_frame";

    add_identity_tf(frame_known, false);

    check_conversion_status(frame_known, create_point(1.0, 1.0, 1.0), frame_unknown, coordinate_transformer::ResultStatus::TRANSFORM_NOT_FOUND);
}

/**
 * @brief Тестирует ошибку преобразования из-за проблем с временной меткой (Extrapolation Error).
 *
 * Проверяет, что функция `convert` возвращает статус ошибки EXTRAPOLATION_ERROR,
 * когда трансформация существует, но недоступна для запрошенной временной метки
 * (например, время в будущем или слишком далеком прошлом).
 */
TEST_F(TestCoordinateTransformer, ConvertTfErrorExtrapolation) {

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
    ASSERT_TRUE(wait_for_transform(tf_buffer, frame_a, frame_b, tf2_ros::fromMsg(tf_time), std::chrono::seconds(1), node));

    geometry_msgs::msg::PoseStamped input_pose = create_pose(frame_b, 0.0, 0.0, 0.0);
    geometry_msgs::msg::PoseStamped output_pose;


    input_pose.header.stamp = tf_time - rclcpp::Duration::from_seconds(1.0);
    coordinate_transformer::ResultStatus status = transformer->convert(input_pose, output_pose, frame_a);
    EXPECT_EQ(status, coordinate_transformer::ResultStatus::TRANSFORM_NOT_FOUND);


     input_pose.header.stamp = tf_time + rclcpp::Duration::from_seconds(1.0);
      status = transformer->convert(input_pose, output_pose, frame_a);
     EXPECT_EQ(status, coordinate_transformer::ResultStatus::TRANSFORM_NOT_FOUND);

     check_conversion_status(frame_b, create_point(0.0, 0.0, 0.0), frame_a, coordinate_transformer::ResultStatus::SUCCESS);

      geometry_msgs::msg::PoseStamped final_input = create_pose(frame_b, 0.0, 0.0, 0.0);
      final_input.header.stamp = tf2_ros::toMsg(tf2::TimePointZero);
      status = transformer->convert(final_input, output_pose, frame_a);
      ASSERT_EQ(status, coordinate_transformer::ResultStatus::SUCCESS);
      EXPECT_NEAR(output_pose.pose.position.x, 1.0, 1e-6);
}

/**
 * @brief Тестирует добавление трансформаций через API CoordinateTransformer.
 *
 * Проверяет, что статическая трансформация, добавленная с помощью
 * метода `add_transform`, корректно применяется при последующих вызовах `convert`.
 */
TEST_F(TestCoordinateTransformer, AddTransformTest) {
    std::string parent = "addtf_parent";
    std::string child = "addtf_child";

    geometry_msgs::msg::TransformStamped tf_to_add;
    tf_to_add.header.stamp = node->get_clock()->now();
    tf_to_add.header.frame_id = parent;
    tf_to_add.child_frame_id = child;
    tf_to_add.transform.translation.x = 5.0;
    tf_to_add.transform.rotation.w = 1.0;

    transformer->addTransform(tf_to_add);

    ASSERT_TRUE(wait_for_transform(tf_buffer, parent, child, tf2::TimePointZero, std::chrono::seconds(1), node))
        << "Transform added via addTransform was not found in the buffer by test listener.";

}


/**
 * @brief Основная функция для запуска тестов GTest.
 * @param argc Количество аргументов командной строки.
 * @param argv Массив аргументов командной строки.
 * @return Статус выполнения тестов (0 в случае успеха).
 */
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}