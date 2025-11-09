cmake_minimum_required(VERSION 3.8)
project(autoware_g29_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(ros_g29_force_feedback REQUIRED)

# Create the adapter executable
add_executable(autoware_g29_adapter_node
  src/autoware_g29_adapter_node.cpp
)

ament_target_dependencies(autoware_g29_adapter_node
  rclcpp
  autoware_adapi_v1_msgs
  ros_g29_force_feedback
)

target_include_directories(autoware_g29_adapter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(autoware_g29_adapter_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS autoware_g29_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()