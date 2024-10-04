cmake_minimum_required(VERSION 3.0.2)
project(joy_control)

 add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES dragon_joy
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/joy_control.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(${PROJECT_NAME}_node src/joy_control_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )

 install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )


 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
  )

 install(DIRECTORY launch/
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
         PATTERN ".svn" EXCLUDE)
