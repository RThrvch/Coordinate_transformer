cmake_minimum_required(VERSION 3.8)
project(coordinate_transformer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

set(THIS_PACKAGE_AMENT_DEPENDENCIES
    rclcpp
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    rcl_interfaces
)
set(THIS_PACKAGE_EXTERNAL_DEPENDENCIES
)
set(THIS_PACKAGE_ALL_DEPENDENCIES
    ${THIS_PACKAGE_AMENT_DEPENDENCIES}
    ${THIS_PACKAGE_EXTERNAL_DEPENDENCIES}
)

add_library(${PROJECT_NAME} SHARED
  src/coordinate_transformer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
)

ament_target_dependencies(${PROJECT_NAME}
    ${THIS_PACKAGE_AMENT_DEPENDENCIES}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME} 
  LIBRARY DESTINATION lib      
  ARCHIVE DESTINATION lib      
  RUNTIME DESTINATION bin      
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}/ 
  FILES_MATCHING PATTERN "*.hpp" 
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config 
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

add_executable(test_transformer_node src/test_transformer_node.cpp)
ament_target_dependencies(test_transformer_node
    rclcpp
    geometry_msgs
    std_msgs
)

target_link_libraries(test_transformer_node ${PROJECT_NAME})

install(TARGETS test_transformer_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_ALL_DEPENDENCIES})

install(EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE ${PROJECT_NAME}::          
  FILE ${PROJECT_NAME}Config.cmake      
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # --- GTest Unit Test ---
  ament_add_gtest(${PROJECT_NAME}_test
    test/unit/test_transformer_logic.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    ${THIS_PACKAGE_ALL_DEPENDENCIES}
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  # ---------------------

  # --- Isolated Launch Integration Test ---
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${CMAKE_CURRENT_SOURCE_DIR}/${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  add_ros_isolated_launch_test(test/integration/test_full_transform.py)

endif()

ament_package()