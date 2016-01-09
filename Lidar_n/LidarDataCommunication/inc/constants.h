#ifndef LIDAR_LIDARDATACOMMUNICATION_CONSTANTS_H_
#define LIDAR_LIDARDATACOMMUNICATION_CONSTANTS_H_

#include <stdlib.h>

// Defines constants for use in the LidarDataCommunication package
namespace Constants {
    // The default timeout value for a request to the RoboRIO
    const static struct timeval defaultTimeoutValue = {
            .tv_sec = 0,
            .tv_usec = 100 * 1000,
    };
};

#endif // LIDAR_LIDARDATACOMMUNICATION_CONSTANTS_H_