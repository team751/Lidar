//
// Created by Sam Baumgarten on 10/7/15.
//

#include "../inc/coordinate_conversion.h"

cv::Vec4i LidarDataCoordinateConversion::convertCoordinateSystem(cv::Vec4i coordinateSystem, CoordinateSystem from, CoordinateSystem to) {
    if (from == CoordinateSystem::LidarCoordinateSystem && to == CoordinateSystem::ImageCoordinateSystem) {
        cv::Vec4i convertedSystem;

        for (int i = 0; i < 4; i++) {
            convertedSystem[i] = (coordinateSystem[i] + 3000) / 10.0;
        }

        return convertedSystem;
    } else if (from == CoordinateSystem::ImageCoordinateSystem && to == CoordinateSystem::LidarCoordinateSystem) {
        cv::Vec4i convertedSystem;

        for (int i = 0; i < 4; i++) {
            convertedSystem[i] = ((double)coordinateSystem[i] * 10.0) - 3000;
        }

        return convertedSystem;
    } else if (from == to) {
        return coordinateSystem;
    }

    return NULL;
}

cv::Vec2d LidarDataCoordinateConversion::convertCoordinateSystem(cv::Vec2d coordinateSystem, CoordinateSystem from, CoordinateSystem to) {
    if (from == CoordinateSystem::LidarCoordinateSystem && to == CoordinateSystem::ImageCoordinateSystem) {
        cv::Vec2d convertedSystem;

        for (int i = 0; i < 2; i++) {
            convertedSystem[i] = (coordinateSystem[i] + 3000) / 10.0;
        }

        return convertedSystem;
    } else if (from == CoordinateSystem::ImageCoordinateSystem && to == CoordinateSystem::LidarCoordinateSystem) {
        cv::Vec2d convertedSystem;

        for (int i = 0; i < 2; i++) {
            convertedSystem[i] = (coordinateSystem[i] * 10.0) - 3000;
        }

        return convertedSystem;
    } else if (from == to) {
        return coordinateSystem;
    }

    return NULL;
}

double LidarDataCoordinateConversion::convertCoordinateSystem(double value, CoordinateSystem from, CoordinateSystem to) {
    if (from == CoordinateSystem::LidarCoordinateSystem && to == CoordinateSystem::ImageCoordinateSystem) {
        return (value + 3000.0) / 10.0;
    } else if (from == CoordinateSystem::ImageCoordinateSystem && to == CoordinateSystem::LidarCoordinateSystem) {
        return (value * 10.0) - 3000;
    } else if (from == to) {
        return value;
    }

    return 0;
}
