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

## Использование

Этот пакет собирает библиотеку и пример исполняемого узла `test_transformer_node`.

Для запуска примера узла обычно требуется предоставить файл конфигурации, определяющий преобразования и границы.

1.  **Подготовьте файл конфигурации (YAML):** Используйте `config/sample_config.yaml` в качестве шаблона или создайте свой собственный. Подробности см. в разделе **Параметры конфигурации** ниже.
2.  **Запустите узел:**
    ```bash
    ros2 run coordinate_transformer test_transformer_node --ros-args --params-file /path/to/your/config.yaml
    ```
    Замените `/path/to/your/config.yaml` на фактический путь к вашему файлу конфигурации.

    *Примечание:* Основное предназначение может заключаться в использовании библиотеки `coordinate_transformer` в других узлах C++. Обратитесь к заголовочным файлам в директории `include/` для получения информации об API.

## Параметры конфигурации (`config.yaml`)

Узел требует файл конфигурации YAML, указанный с помощью аргумента `--params-file`.

Файл конфигурации имеет два основных раздела: `transforms` и `boundaries`.

### `transforms`

Этот раздел определяет список статических преобразований для публикации. Каждая запись преобразования требует:

-   `parent_frame`: (string) Имя родительской системы координат.
-   `child_frame`: (string) Имя дочерней системы координат.
-   `translation`: (object) Компонент смещения преобразования.
    -   `x`: (double) Смещение по оси x.
    -   `y`: (double) Смещение по оси y.
    -   `z`: (double) Смещение по оси z.
-   `rotation`: (object) Компонент вращения преобразования, представленный кватернионом.
    -   `x`: (double) Компонент x кватерниона.
    -   `y`: (double) Компонент y кватерниона.
    -   `z`: (double) Компонент z кватерниона.
    -   `w`: (double) Компонент w кватерниона.

**Пример:**

```yaml
transforms:
  - parent_frame: "map"
    child_frame: "odom"
    translation: { x: 0.0, y: 0.0, z: 0.0 }
    rotation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }

  - parent_frame: "odom"
    child_frame: "base_link"
    translation: { x: 1.0, y: 0.5, z: 0.1 }
    rotation: { x: 0.0, y: 0.0, z: 0.707, w: 0.707 } # Поворот на 90 градусов вокруг Z
```

### `boundaries`

Этот раздел определяет именованные границы, связанные с конкретными системами координат. Каждая граница определяется своими минимальной и максимальной угловыми точками.

-   `<frame_name>`: (string) Имя системы координат, к которой применяется граница (например, `map`, `robot_workspace`). Должно совпадать с системой координат, определенной в вашей системе (либо через раздел `transforms`, либо иным образом).
    -   `min`: (object) Минимальный угол ограничивающего параллелепипеда, выровненного по осям.
        -   `x`: (double) Минимальная координата x.
        -   `y`: (double) Минимальная координата y.
        -   `z`: (double) Минимальная координата z.
    -   `max`: (object) Максимальный угол ограничивающего параллелепипеда, выровненного по осям.
        -   `x`: (double) Максимальная координата x.
        -   `y`: (double) Максимальная координата y.
        -   `z`: (double) Максимальная координата z.

**Пример:**

```yaml
boundaries:
  map: # Граница для системы координат 'map'
    min: { x: -50.0, y: -50.0, z: -1.0 }
    max: { x: 50.0, y: 50.0, z: 5.0 }

  robot_workspace: # Граница для гипотетической системы координат 'robot_workspace'
     min: { x: -1.0, y: -1.0, z: 0.0 }
     max: { x: 1.0, y: 1.0, z: 1.5 }
```

Это позволяет библиотеке/узлу `coordinate_transformer` проверять, находятся ли точки или позы в пределах определенных пространственных ограничений для конкретных систем координат. 

## Пример использования

```cpp
auto transformer = std::make_shared<CoordinateTransformer>(node);

// Загрузка трансформаций из YAML
transformer->loadConfig("config.yaml");

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