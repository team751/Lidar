# Packages
find_package(OpenCV REQUIRED)
find_package(PCL 1.7.1 REQUIRED COMPONENTS common kdtree)

# Include Directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

# Linking Directories
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Library
add_library(LidarDataProcessor lidar_data_processor.cpp ../inc/lidar_data_processor.h point_cloud_conversion.cpp ../inc/point_cloud_conversion.h ../inc/constants.h coordinate_conversion.cpp ../inc/coordinate_conversion.h line_properties.cpp ../inc/line_properties.h)

# Link Library
target_link_libraries(LidarDataProcessor ${OpenCV_LIBS} ${PCL_LIBRARIES})
