cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)

add_compile_definitions(M_PI=3.14159265358979323846)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(CV_LIBS 
    $ENV{MY_GAZEBO}/wpilib/libs/cameraserver.lib;
    $ENV{MY_GAZEBO}/wpilib/libs/cscore.lib;
    $ENV{MY_GAZEBO}/wpilib/libs/opencv_core452.lib;
    $ENV{MY_GAZEBO}/wpilib/libs/opencv_imgcodecs452.lib;
)

include_directories(${GAZEBO_INCLUDE_DIRS} $ENV{MY_GAZEBO}/wpilib/includes)

link_directories(${GAZEBO_LIBRARY_DIRS} $ENV{MY_GAZEBO}/wpilib/libs)

add_executable(MjpegTest MjpegTest.cpp)

target_link_libraries(MjpegTest ${GAZEBO_LIBRARIES} ${CV_LIBS})


