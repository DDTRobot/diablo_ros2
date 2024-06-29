cmake_minimum_required(VERSION 3.5)
project(diablo_rviz2_control_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
# required for RVIZ

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motion_msgs REQUIRED)


include_directories(include)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
            src/mission_panel.cpp
            src/switch.cpp
            src/teleop_button.cpp
)

set(dependencies
  rclcpp
  rviz_common
  motion_msgs
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Link non ament packages
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME})
install(
DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
DESTINATION "share/${PROJECT_NAME}")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# replaces catkin_package(LIBRARIES ${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})

ament_package()
