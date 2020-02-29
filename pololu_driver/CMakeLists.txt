cmake_minimum_required(VERSION 2.8.3)
project(pololu_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_executable(${PROJECT_NAME}_node src/driver.cpp src/controller.cpp src/channel.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})