cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

# Find gazebo-config

add_compile_definitions(M_PI=3.14159265358979323846)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} . )
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build plugins

add_library(sim_contact SHARED sim_contact.cpp)
target_link_libraries(sim_contact ${GAZEBO_LIBRARIES})

add_library(sim_motor SHARED sim_motor.cpp)
target_link_libraries(sim_motor ${GAZEBO_LIBRARIES})

add_library(sim_encoder SHARED sim_encoder.cpp)
target_link_libraries(sim_encoder ${GAZEBO_LIBRARIES})

add_library(sim_gyro SHARED sim_gyro.cpp)
target_link_libraries(sim_gyro ${GAZEBO_LIBRARIES})

add_library(sim_clock SHARED sim_clock.cpp)
target_link_libraries(sim_clock ${GAZEBO_LIBRARIES})

