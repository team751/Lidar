//
// Created by Sam Baumgarten on 10/7/15.
//

#ifndef LIDAR_LIDARDATAPROCESSOR_COORDINATECONVERSION_H
#define LIDAR_LIDARDATAPROCESSOR_COORDINATECONVERSION_H


#include <opencv2/core/core.hpp>

namespace LidarDataCoordinateConversion {
    enum CoordinateSystem {
        LidarCoordinateSystem,
        ImageCoordinateSystem
    };

    cv::Vec4i convertCoordinateSystem(cv::Vec4i coordinateSystem, CoordinateSystem from, CoordinateSystem to);
    cv::Vec2d convertCoordinateSystem(cv::Vec2d coordinateSystem, CoordinateSystem from, CoordinateSystem to);
    double convertCoordinateSystem(double value, CoordinateSystem from, CoordinateSystem to);
};


#endif //LIDAR_LIDARDATAPROCESSOR_COORDINATECONVERSION_H
