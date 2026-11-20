cmake_minimum_required(VERSION 3.8)
project(sensor_pkgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(PressureSensor SHARED src/PressureSensor.cc)
target_link_libraries(PressureSensor PRIVATE
  ignition-gazebo6::core
  ignition-plugin1::register
  ignition-transport11::core
  ignition-msgs8::core)
install(TARGETS PressureSensor LIBRARY DESTINATION lib/${PROJECT_NAME})

add_library(DvlSensor SHARED src/DvlSensor.cc)
target_link_libraries(DvlSensor PRIVATE
  ignition-gazebo6::core
  ignition-plugin1::register
  ignition-transport11::core
  ignition-msgs8::core)
install(TARGETS DvlSensor LIBRARY DESTINATION lib/${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Dvl.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/pressure_to_depth.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/dvl_twist_cov.py DESTINATION lib/${PROJECT_NAME})

ament_package()
