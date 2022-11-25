cmake_minimum_required(VERSION 3.1)

set(CMAKE_VERBOSE_MAKEFILE OFF)

set(CMAKE_INSTALL_PREFIX ../)

# add these cmake command line arguments to fix windows dllspec linking problems
option(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS "CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS" TRUE)

option(BUILD_SHARED_LIBS "BUILD_SHARED_LIBS" TRUE)

set(CMAKE_CXX_STANDARD 20)

project(apriltagjni VERSION 3.3.0 LANGUAGES C CXX)

# Library

set(INCLUDES include $ENV{JAVA_HOME}/include $ENV{JAVA_HOME}/include/win32)

include_directories(${INCLUDES})

aux_source_directory(apriltag_src APRILTAG_FILES)
aux_source_directory(common COMMON_SRC)
aux_source_directory(native JNI_FILES)

set(SOURCE ${APRILTAG_FILES} ${COMMON_SRC} ${JNI_FILES})

message(STATUS "Include Dirs: '${INCLUDES}' ")
message(STATUS "Source files: '${SOURCE}' ")

#set(_OpenCV_REQUIRED_COMPONENTS core imgproc videoio highgui)
#find_package(OpenCV COMPONENTS ${_OpenCV_REQUIRED_COMPONENTS} QUIET CONFIG)
#find_package( OpenCV REQUIRED )
#include_directories(${OpenCV_INCLUDE_DIRS} )

set(LIB_SRCS ${SOURCE})

add_library(${PROJECT_NAME} SHARED ${LIB_SRCS})

add_compile_definitions("_CRT_SECURE_NO_WARNINGS")

include(GNUInstallDirs)

# install 
install(TARGETS ${PROJECT_NAME} EXPORT ${targets_export_name}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

