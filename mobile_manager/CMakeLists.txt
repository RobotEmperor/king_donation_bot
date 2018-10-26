cmake_minimum_required(VERSION 2.8.3)
project(mobile_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)



 add_message_files(
   FILES
   motor_cmd.msg
 )



 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )


catkin_package(
  INCLUDE_DIRS include
 
  CATKIN_DEPENDS roscpp std_msgs 
  #DEPENDS system_lib

)


include_directories(include ${catkin_INCLUDE_DIRS} )

 add_executable(mobile_manager_node src/mobile_manager_node.cpp)
 target_link_libraries(mobile_manager_node ${catkin_LIBRARIES})
 add_dependencies(mobile_manager_node mobile_manager_generate_messages_cpp)
