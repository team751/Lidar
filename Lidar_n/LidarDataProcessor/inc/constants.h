//
// Created by Sam Baumgarten on 10/7/15.
//

#ifndef LIDAR_LIDARDATAPROCESSOR_CONSTANTS_H
#define LIDAR_LIDARDATAPROCESSOR_CONSTANTS_H

#include <opencv2/core/types_c.h>
#include <math.h>

namespace Constants {
    const double kRho = .75;
    const double kTheta = CV_PI / 180;
    const int kThreshold = 20;
    const double kMinLineLength = 25;
    const double kMaxLineGap = 15;

    const double kFocalLength = 503.194;
    const double kActualLength = 622.3;

    const double kLengthDeltaThreshold = 120;

    const double kDefaultAngle = -(10.0 * M_PI / 180.0);

}

#endif //LIDAR_CONSTANTS_H
