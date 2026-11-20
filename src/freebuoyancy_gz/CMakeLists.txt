cmake_minimum_required(VERSION 3.8)
project(freebuoyancy_gz)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

add_library(freebuoyancy-system SHARED
  src/FreeBuoyancy.cc
)

target_link_libraries(freebuoyancy-system
  PRIVATE
    ignition-gazebo6::ignition-gazebo6
    ignition-math6::ignition-math6
    ignition-plugin1::register
)

target_include_directories(freebuoyancy-system PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS freebuoyancy-system
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
