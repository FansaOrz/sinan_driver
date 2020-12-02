cmake_minimum_required(VERSION 3.0.2)
project(send_comnav_data)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(

)

add_subdirectory(third_party/GeographicLib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  third_party/GeographicLib/include

)

#add_executable(main src/main.cpp)
#target_link_libraries(main ${catkin_LIBRARIES})


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)
find_package(PkgConfig)

pkg_check_modules(SERIAL libserial)


add_executable(main src/main.cpp)
target_include_directories(main PRIVATE ${SERIAL_INCLUDE_DIRS})
#add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} PRIVATE ${SERIAL_INCLUDE_DIRS})
target_link_libraries(main ${catkin_LIBRARIES} ${SERIAL_LDFLAGS} libGeographiccc ${CMAKE_THREAD_LIBS_INIT} )


#add_executable(ExampleProject main.cpp)

#target_link_libraries(ExampleProject PRIVATE ${SERIAL_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})