# Include Directories
include_directories(${Lidar_SOURCE_DIR}/LidarMain/inc)
include_directories(${Lidar_SOURCE_DIR}/LidarDataCommunication/inc)
include_directories(${Lidar_SOURCE_DIR}/LidarDataProcessor/inc)
include_directories(${Lidar_SOURCE_DIR}/LidarSerialDataCollector/inc)

# Link Directories
link_directories(${Lidar_SOURCE_DIR}/LidarDataCommunication/src)
link_directories(${Lidar_SOURCE_DIR}/LidarDataProcessor/src)
link_directories(${Lidar_SOURCE_DIR}/LidarSerialDataCollector/src)

# link_directories("/usr/local/cuda/lib")

# Set Output Directories
set(EXECUTABLE_OUTPUT_PATH ../../)

# Define Executables
add_executable(lidar lidar.cpp)

# Link Libraries
target_link_libraries(lidar LidarDataCommunication LidarDataProcessor LidarSerialDataCollector)
