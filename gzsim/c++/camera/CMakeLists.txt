cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

# Find gazebo-config

add_compile_definitions(M_PI=3.14159265358979323846)
add_compile_definitions(with_FFMPEG=ON)

# Find Gazebo
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(${GAZEBO_INCLUDE_DIRS} . )
include_directories($ENV{GZ_SIM}/wpilib/includes)

set(CV_LIBS 
    $ENV{GZ_SIM}/wpilib/libs/cameraserver.lib;
    $ENV{GZ_SIM}/wpilib/libs/cscore.lib;
    $ENV{GZ_SIM}/wpilib/libs/opencv_core452.lib;
    $ENV{GZ_SIM}/wpilib/libs/opencv_imgcodecs452.lib;
    $ENV{GZ_SIM}/wpilib/libs/opencv_imgproc452.lib;
    $ENV{GZ_SIM}/wpilib/libs/opencv_videoio452.lib;
    $ENV{GZ_SIM}/wpilib/libs/opencv_video452.lib;
)

link_directories(${GAZEBO_LIBRARY_DIRS} $ENV{GZ_SIM}/wpilib/libs)

# Build plugins
add_library(sim_camera SHARED sim_camera.cpp)
target_link_libraries(sim_camera ${GAZEBO_LIBRARIES} FreeImage.lib ${CV_LIBS})

