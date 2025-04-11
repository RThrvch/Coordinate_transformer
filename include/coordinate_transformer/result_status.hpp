/**
 * @file result_status.hpp
 * @brief Определяет перечисление ResultStatus, используемое для указания результата операций преобразования.
 *
 * @author Ruslan Mukhametsafin
 * @date   2025-04-10
 */

#pragma once

namespace coordinate_transformer
{
/**
 * @brief Перечисление, представляющее статус результата операции преобразования координат.
 */
enum class ResultStatus
{
  SUCCESS,                //!< Операция успешно завершена.
  OUT_OF_BOUNDS,          //!< Результат преобразования вышел за установленные границы.
  TRANSFORM_NOT_FOUND,    //!< Необходимая трансформация между системами координат не найдена.
  INVALID_INPUT,          //!< Входные данные (например, PoseStamped) некорректны.
  CONFIGURATION_ERROR     //!< Ошибка в конфигурации (например, при загрузке YAML).
};
} // namespace coordinate_transformer
