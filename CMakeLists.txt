cmake_minimum_required(VERSION 2.8.3)
project(depthimage_creater)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  amsl_recog_msgs
  cv_bridge
  image_geometry
  image_transport
  jsk_recognition_msgs
  laser_geometry
  message_filters
  message_generation
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3 REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAG "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED COMPONENTS system)
include_directories(${Eigen_INCLUDE_DIRS})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   amsl_recog_msgs#   jsk_recognition_msgs#   nav_msgs#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES depthimage_creater
#  CATKIN_DEPENDS amsl_recog_msgs cv_bridge image_geometry image_transport jsk_recognition_msgs laser_geometry message_filters message_generation nav_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# PCL
add_executable(devide_pointcloud src/pcl/division.cpp)
target_link_libraries(devide_pointcloud
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
)

add_executable(split_pointcloud src/pcl/split_pointcloud.cpp)
target_link_libraries(split_pointcloud
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
)

# Detection
add_executable(euclidean_clustering src/detection/clustering.cpp)
target_link_libraries(euclidean_clustering
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
)

add_executable(rm_object src/detection/rm_object.cpp)
target_link_libraries(rm_object
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(integrate_objectinfo src/detection/integration.cpp)
target_link_libraries(integrate_objectinfo
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(cluster_transform src/detection/cluster_transform.cpp)
target_link_libraries(cluster_transform
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(pickup_cluster src/detection/pickup_cluster.cpp)
target_link_libraries(pickup_cluster
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(plane_segmentation src/detection/plane_segmentation.cpp)
target_link_libraries(plane_segmentation
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(iou src/detection/iou.cpp)
target_link_libraries(iou
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(pub_boundingbox src/detection/pub_boundingbox.cpp)
target_link_libraries(pub_boundingbox
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(cluster_position src/detection/cluster_position.cpp)
target_link_libraries(cluster_position
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(iou_integrater src/detection/iou_integrater.cpp)
target_link_libraries(iou_integrater
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(transform src/detection/transform.cpp)
target_link_libraries(transform
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(showinfo src/detection/showinfo.cpp)
target_link_libraries(showinfo
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(remove_cluster src/detection/remove_cluster.cpp)
target_link_libraries(remove_cluster
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(show_detection_result src/detection/show_detection_result.cpp)
target_link_libraries(show_detection_result
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(normal_estimation_for_map src/detection/normal_estimation_for_map.cpp)
target_link_libraries(normal_estimation_for_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)


# depthimage_creater
add_executable(camera_info src/depthimage_creater/camera_info.cpp)
target_link_libraries(camera_info
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)
add_executable(data_saver src/depthimage_creater/data_saver.cpp)
target_link_libraries(data_saver
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)
add_executable(depthimage_creater_main src/depthimage_creater/depthimage_creater.cpp)
target_link_libraries(depthimage_creater_main
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)
add_executable(tf_broadcaster src/depthimage_creater/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

# projection
add_executable(range_image_projection src/projection/range_image_projection.cpp)
target_link_libraries(range_image_projection
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(quicksort src/projection/quicksort.cpp)
target_link_libraries(quicksort
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

add_executable(munkres src/functions/munkres.cpp)
target_link_libraries(munkres
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)

# pcl
add_executable(outlier_removal src/pcl/outlier_removal.cpp)
target_link_libraries(outlier_removal
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)
add_executable(bilateral_upsampling src/pcl/bilateral_upsampling.cpp)
target_link_libraries(bilateral_upsampling
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)
add_executable(polygon_mesh src/pcl/polygon_mesh.cpp)
target_link_libraries(polygon_mesh
    ${catkin_LIBRARIES}
    ${PCL_LIBRRIES}
    ${OpenCV_LIBRARIES}
)




## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/depthimage_creater.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/depthimage_creater_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_depthimage_creater.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
