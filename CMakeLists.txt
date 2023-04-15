cmake_minimum_required(VERSION 3.5)
project(camera_calibration_azure)

set(Boost_NO_WARN_NEW_VERSIONS 1)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost 1.71 REQUIRED)
find_package(PCL REQUIRED)
find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen)
find_package(OpenMP REQUIRED)
add_definitions(-D_USE_MATH_DEFINES)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories( include
  ${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
  ${OPENMP_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
	${cv_bridge_INCLUD_DIRS}
	)

if (OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  set(OpenMP_LIBS gomp)
endif()

LIST(APPEND HEADERFILES 
include/ImageSubscriberROS2.h
)
LIST(APPEND SOURCEFILES
src/ImageSubscriberROS2.cpp
src/main_ros2.cpp
)
add_executable(run ${HEADERFILES} ${SOURCEFILES})
ament_target_dependencies(run rclcpp std_msgs sensor_msgs OpenCV)
target_compile_options(run PRIVATE ${OpenMP_FLAGS})
target_link_libraries( run	
          ${OpenCV_LIBS}
          ${catkin_LIBRARIES}
          ${PCL_LIBRARIES}
          ${EIGEN_LIBRARIES}
          ${OpenMP_LIBS}
          ${Boost_INCLUDE_DIRS}
          ${cv_bridge_LIBRARIES}
          ) 

install(TARGETS
  run
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
