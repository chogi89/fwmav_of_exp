cmake_minimum_required(VERSION 2.8.3)
project(fwmav_of_exp)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation geometry_msgs sensor_msgs)
find_package( OpenCV REQUIRED )

add_message_files(FILES MSG_NodeTime.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  LIBRARIES fwmav_of_exp
  CATKIN_DEPENDS roscpp std_msgs cv_bridge image_transport geometry_msgs sensor_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(off_board_camera_test src/off_board_camera_test.cpp)
target_link_libraries(off_board_camera_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(off_board_camera_test fwmav_of_exp_generate_messages_cpp)

add_executable(fwmav_exp_avc src/fwmav_exp_avc.cpp)
target_link_libraries(fwmav_exp_avc ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(fwmav_exp_avc fwmav_of_exp_generate_messages_cpp)
