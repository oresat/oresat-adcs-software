if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "Prevented in-tree build, it is bad practice.\nTry 'cd build && cmake ..' instead.")
endif()

cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
project(OreSat-ADCS CXX)
set (CMAKE_CXX_STANDARD 17)


##############################################################################
# Check Linux distribution / verion / kernel version

set(LINUX_DISTRIBUTION "")

if(EXISTS "/etc/os-release")
    execute_process(COMMAND cat /etc/os-release
        COMMAND grep -m1 NAME
        COMMAND awk -F= "{ print $2 }"
        COMMAND tr "\n" " "
        COMMAND sed "s/ //"
        OUTPUT_VARIABLE LINUX_DISTRIBUTION
        RESULT_VARIABLE LINUX_ID_RESULT
    )
    if(${LINUX_DISTRIBUTION} MATCHES "Debian")
        set(LINUX_DISTRIBUTION "Debian")
    endif()
else()
    message(FATAL_ERROR "Missing /etc/os-release. Not a Linux based system?")
endif()


##############################################################################
# Deal with dependencies

# pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# pkg-config
find_package(PkgConfig REQUIRED)

# libsystemd
pkg_check_modules(LIBSYSTEMD REQUIRED libsystemd)

# sdbus-c++
find_package(sdbus-c++ REQUIRED)


##############################################################################
# Deal with git submodule

find_package(Git)
if(NOT GIT_FOUND)
    message(FATAL_ERROR "git check failed")
endif()
if(NOT EXISTS "../src/CANopenNode/.git")
    message(STATUS "Running: git submoduke update --init")
    execute_process(COMMAND git submodule update --init WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()


##############################################################################
# Release vs debug modes (set CFLAGS, LDFLAGS, etc)
# to use: cmake -DCMAKE_BUILD_TYPE=Debug .

# default to release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_C_FLAGS_RELEASE "-O3")


##############################################################################
# Check if make or ninja

if(CMAKE_MAKE_PROGRAM MATCHES "make")
    set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "make")
endif()
if(CMAKE_MAKE_PROGRAM MATCHES "ninja")
    set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "ninja")
endif()
if(NOT CMAKE_MAKE_PROGRAM_PRETTY_NAME)
    set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "${CMAKE_MAKE_PROGRAM}")
endif()


##############################################################################
# Get source files


set(SOURCE_FILES
    src/main.cpp
)


##############################################################################
# Create an executable


add_executable(oresat-adcs ${SOURCE_FILES})
target_link_libraries(oresat-adcs PRIVATE SDBusCpp::sdbus-c++)
#target_link_libraries(oresat-adcs ${LIBSYSTEMD_LIBRARIES} Threads::Threads)
target_include_directories(oresat-adcs PUBLIC ${LIBSYSTEMD_INCLUDE_DIRS})
target_compile_options(oresat-adcs PUBLIC ${LIBSYSTEMD_CFLAGS_OTHER})


