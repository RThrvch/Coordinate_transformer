# coordinate_transformer

Библиотека C++ и node для преобразования координат с проверкой границ для ROS 2.

## Обзор

Этот пакет предоставляет функциональность для управления и запроса преобразований координат (tf) между различными системами координат (фреймами), в первую очередь предназначенную для использования в среде ROS 2. Он позволяет определять статические преобразования и проверять, попадают ли определенные точки или позы в предопределенные границы для заданных систем координат.

## Зависимости

Этот пакет зависит от следующих пакетов и библиотек ROS 2:
- `rclcpp`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `geometry_msgs`
- `yaml-cpp`
- `std_msgs`

## Установка

1.  Перейдите в директорию `src` вашей рабочей области ROS 2:
    ```bash
    cd ~/ros2_ws/src
    ```
2.  Клонируйте этот репозиторий (если вы еще этого не сделали):
    ```bash
    # git clone <repository_url>
    ```
3.  Соберите пакет:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select coordinate_transformer
    ```
4.  Активируйте рабочую область:
    ```bash
    source install/setup.bash
    ```

## Тестирование

Для запуска тестов, определенных в пакете, используйте следующую команду из корневой директории вашей рабочей области (`~/ros2_ws`):

```bash
colcon test --packages-select coordinate_transformer
```

После завершения выполнения тестов, вы можете просмотреть результаты:

```bash
colcon test-result --all
```

Для более подробного вывода:

```bash
colcon test-result --verbose
```

## Параметры конфигурации (`config.yaml`)

Узел требует файл конфигурации YAML, указанный с помощью аргумента `--params-file`. Файл должен соответствовать формату параметров ROS 2.

Все параметры узла должны находиться внутри структуры `<имя_вашего_узла>:` -> `ros__parameters:`.

```yaml
# Замените 'test_transformer_node' на имя вашего узла, если оно отличается
test_transformer_node:
  ros__parameters:
    # --- Статические трансформации ---

    # Список имен статических преобразований для загрузки и публикации.
    static_transform_names: ["map_to_odom", "odom_to_base"]

    # Словарь, определяющий детали каждого статического преобразования.
    # Ключи должны совпадать с именами в 'static_transform_names'.
    static_transforms:
      map_to_odom: # Имя из static_transform_names
        parent_frame: "map"       # Родительский фрейм
        child_frame: "odom"        # Дочерний фрейм
        translation: { x: 0.1, y: 0.2, z: 0.0 } # Смещение {x, y, z}
        rotation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } # Поворот (кватернион) {x, y, z, w}
      odom_to_base: # Имя из static_transform_names
        parent_frame: "odom"
        child_frame: "base_link"
        translation: { x: 0.5, y: 0.0, z: 0.1 }
        rotation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }

    # --- Границы фреймов ---

    # Список имен фреймов, для которых определены границы.
    boundary_frame_names: ["map", "base_link"]

    # Словарь, определяющий границы для каждого фрейма.
    # Ключи должны совпадать с именами в 'boundary_frame_names'.
    boundaries:
      map: # Имя из boundary_frame_names
        # Минимальные координаты ограничивающего параллелепипеда
        min_x: -50.0
        min_y: -50.0
        min_z: -5.0
        # Максимальные координаты ограничивающего параллелепипеда
        max_x: 50.0
        max_y: 50.0
        max_z: 5.0
      base_link: # Имя из boundary_frame_names
        min_x: -2.0
        min_y: -2.0
        min_z: -0.5
        max_x: 2.0
        max_y: 2.0
        max_z: 2.0

    # --- Другие параметры (пример) ---
    # Вы можете добавлять другие параметры, необходимые вашему узлу.
    target_frame: "map"
```

**Описание ключевых параметров:**

*   `static_transform_names`: (list of string) Список имен статических преобразований.
*   `static_transforms`: (map) Словарь, где каждый ключ - это имя из `static_transform_names`. Значение содержит:
    *   `parent_frame`: (string) Родительский фрейм.
    *   `child_frame`: (string) Дочерний фрейм.
    *   `translation`: (map) Смещение с ключами `x`, `y`, `z`.
    *   `rotation`: (map) Кватернион поворота с ключами `x`, `y`, `z`, `w`.
*   `boundary_frame_names`: (list of string) Список имен фреймов с границами.
*   `boundaries`: (map) Словарь, где каждый ключ - это имя из `boundary_frame_names`. Значение содержит:
    *   `min_x`, `min_y`, `min_z`: (double) Минимальные координаты границы.
    *   `max_x`, `max_y`, `max_z`: (double) Максимальные координаты границы.

Это позволяет библиотеке/узлу `coordinate_transformer` загружать конфигурацию стандартным для ROS 2 способом и проверять, находятся ли точки или позы в пределах определенных пространственных ограничений для конкретных систем координат.

## Пример использования

```cpp
auto transformer = std::make_shared<CoordinateTransformer>(node);

// Установка границ для системы "robot_base"
// Замените min_point и max_point на ваши реальные значения geometry_msgs::msg::Point
geometry_msgs::msg::Point min_point;
min_point.x = -1.0; min_point.y = -1.0; min_point.z = 0.0;
geometry_msgs::msg::Point max_point;
max_point.x = 1.0; max_point.y = 1.0; max_point.z = 1.5;
transformer->setBounds("robot_base", min_point, max_point);

// Преобразование координаты
geometry_msgs::msg::PoseStamped input, output;
// Задайте входные данные input.header.frame_id, input.pose и т.д.
input.header.frame_id = "some_frame"; // Пример
input.pose.position.x = 0.5; // Пример
input.pose.position.y = 0.5; // Пример
input.pose.position.z = 0.5; // Пример
input.pose.orientation.w = 1.0; // Пример

auto status = transformer->convert(input, output, "world");

if (status == ResultStatus::SUCCESS) {
 // Обработка результата
 // output будет содержать преобразованную позу в системе координат "world"
 RCLCPP_INFO(node->get_logger(), "Conversion successful!");
} else {
 // Обработка ошибки
 RCLCPP_ERROR(node->get_logger(), "Conversion failed with status: %d", static_cast<int>(status));
}
```
