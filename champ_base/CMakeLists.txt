cmake_minimum_required(VERSION 3.5)
project(champ_base)

add_compile_options(-std=c++17)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(champ REQUIRED)
find_package(message_filters REQUIRED)
find_package(unitree_go REQUIRED)


set(dependencies
  rclcpp
  rclpy
  champ_msgs
  visualization_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  nav_msgs
  tf2_ros
  urdf
  message_filters
  champ
  unitree_go
)


include_directories(
  include
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ/
  ${champ_INCLUDE_DIRS}/champ/champ/
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME} 
)

add_library(state_estimation SHARED src/state_estimation.cpp)
ament_target_dependencies(state_estimation ${dependencies})
add_executable(state_estimation_node src/state_estimation_node.cpp)
target_link_libraries(state_estimation_node state_estimation)
ament_target_dependencies(state_estimation_node ${dependencies})


add_library(foot_contacts SHARED src/foot_contacts.cpp)
ament_target_dependencies(foot_contacts ${dependencies})
add_executable(foot_contacts_node src/foot_contacts_node.cpp)
target_link_libraries(foot_contacts_node foot_contacts)
ament_target_dependencies(foot_contacts_node ${dependencies})


install(TARGETS
    foot_contacts
    foot_contacts_node
    state_estimation
    state_estimation_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)


install(TARGETS
    foot_contacts_node
    state_estimation_node
    DESTINATION lib/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
