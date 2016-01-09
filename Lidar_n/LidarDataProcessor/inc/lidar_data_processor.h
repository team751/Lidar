//
// Created by Sam Baumgarten on 10/7/15.
//

#ifndef LIDAR_LIDARDATAPROCESSOR_LIDARDATAPROCESSOR_H
#define LIDAR_LIDARDATAPROCESSOR_LIDARDATAPROCESSOR_H

#include <array>

#include "../../LidarCommonTypes/tote_pose.h"

class LidarDataProcessor {
public:
    TotePose processLidarData(std::array<double, 360> lidarData);
};

#endif // LIDAR_LIDARDATAPROCESSOR_LIDAR_DATA_PROCESSOR_H
