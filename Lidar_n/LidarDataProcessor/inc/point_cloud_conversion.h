//
// Created by Sam Baumgarten on 10/7/15.
//

#ifndef LIDAR_LIDARDATAPROCESSOR_POINTCLOUDCONVERSION_H
#define LIDAR_LIDARDATAPROCESSOR_POINTCLOUDCONVERSION_H

#include <array>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

namespace PointCloudConversion {
    cv::Mat createImageWithLidarData(std::array<double, 360> lidarData, int angleStart = 0, int angleEnd = 360, int pointRadius = 1, cv::Scalar color = cv::Scalar(255), int type = CV_8UC3, int width = 1000, int height = 1000);
    pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloudFromLidarData(std::array<double, 360> lidarData, int angleStart = 0, int angleEnd = 360);
}

#endif // LIDAR_LIDARDATAPROCESSOR_POINTCLOUDCONVERSION_H
