#
# The find_package macro for Orocos-RTT requires
# cmake >= 2.8
#
cmake_minimum_required(VERSION 2.8)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(wiringpi_rtt_icm20948)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
add_subdirectory( src )

add_custom_target(wiringpi_rtt_icm20948_extras ALL SOURCES ${CMAKE_SOURCE_DIR}/icm20948_rtt_imu.ops)

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
