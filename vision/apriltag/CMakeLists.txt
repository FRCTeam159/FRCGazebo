cmake_minimum_required(VERSION 3.1)

set(CMAKE_VERBOSE_MAKEFILE OFF)

set(CMAKE_INSTALL_PREFIX ../)

# add these cmake command line arguments to fix windows dllspec linking problems
option(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS "CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS" TRUE)

option(BUILD_SHARED_LIBS "BUILD_SHARED_LIBS" TRUE)

set(CMAKE_CXX_STANDARD 20)

project(apriltagsim VERSION 3.3.0 LANGUAGES C CXX)

# Library

aux_source_directory(native JNI_FILES)

set(LIB_DIRS $ENV{GZ_SIM}\\vision\\apriltag\\lib)
set(SOURCE ${JNI_FILES} ${APRILTAG_FILES})
set(INCLUDES include $ENV{JAVA_HOME}/include $ENV{JAVA_HOME}/include/win32)

include_directories(${INCLUDES})

message(STATUS "Include Dirs: '${INCLUDES}' ")
message(STATUS "Source files: '${SOURCE}' ")
message(STATUS "lib dirs: '${LIB_DIRS}' ")

set(LIB_SRCS ${SOURCE})

add_library(${PROJECT_NAME} SHARED ${LIB_SRCS})
target_link_directories(${PROJECT_NAME} PUBLIC ${LIB_SRCS})

add_compile_definitions("_CRT_SECURE_NO_WARNINGS")

include(GNUInstallDirs)

# install 
install(TARGETS ${PROJECT_NAME} EXPORT ${targets_export_name}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

