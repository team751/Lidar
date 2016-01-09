# Argument Setup
set(serial_DIR /tmp/usr/local/share/serial/cmake/)

# Packages
find_package(serial 1.2.1 REQUIRED)

# Include Directories
include_directories("/tmp/usr/local/include/")

# Linking Directories
link_directories(${serial_LIBRARY_DIRS})


subdirs(src)
