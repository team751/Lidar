# Argument Setup
set(serial_DIR /usr/local/share/serial/cmake/)

# Packages
find_package(serial 1.2.1 REQUIRED)

# Include Directories
include_directories("/usr/local/include/")

# Linking Directories
link_directories("/usr/local/lib/")

message(${serial_INCLUDE_DIRS})

# Library
add_library(LidarSerialDataCollector lidar_serial_data_collector.cpp ../inc/constants.h ../inc/lidar_serial_packet.h)

# Link Library
target_link_libraries(LidarSerialDataCollector ${serial_LIBRARY_DIRS}/libserial.dylib)

# Set flags to use c++11
set(CMAKE_CXX_FLAGS "-std=c++0x" CACHE STRING "compile flags" FORCE)
