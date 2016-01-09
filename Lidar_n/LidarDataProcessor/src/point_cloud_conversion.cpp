//
// Created by Sam Baumgarten on 10/7/15.
//

#include "../inc/point_cloud_conversion.h"

#include "../inc/coordinate_conversion.h"

cv::Mat PointCloudConversion::createImageWithLidarData(std::array<double, 360> lidarData, int angleStart, int angleEnd, int pointRadius, cv::Scalar color, int type, int width, int height) {
    cv::Mat image = cv::Mat::zeros(height, width, type);

    for (int theta = angleStart; theta < angleEnd; theta++) {
        double rho = lidarData[theta % 360];

        double x = LidarDataCoordinateConversion::convertCoordinateSystem(cos(theta * M_PI / 180.0) * rho, LidarDataCoordinateConversion::CoordinateSystem::LidarCoordinateSystem, LidarDataCoordinateConversion::CoordinateSystem::ImageCoordinateSystem);
        double y = LidarDataCoordinateConversion::convertCoordinateSystem(sin(theta * M_PI / 180.0) * rho, LidarDataCoordinateConversion::CoordinateSystem::LidarCoordinateSystem, LidarDataCoordinateConversion::CoordinateSystem::ImageCoordinateSystem);

//        std::cout << theta << std::endl;

        if (x > 0 && x < width && y > 0 && y < height) {
            circle(image, cv::Point(x, y), 1, color, 1);
        }
    }

    return image;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudConversion::createPointCloudFromLidarData(std::array<double, 360> lidarData, int angleStart, int angleEnd) {
    // Create cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Configure cloud
    cloud->width    = 1000;
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    // Fill cloud
    for (int theta = angleStart; theta < angleEnd; theta++) {
        double rho = lidarData[theta % 360];

        double x = cos(theta * M_PI / 180.0) * rho;
        double y = sin(theta * M_PI / 180.0) * rho;

        cloud->points[theta % 360].x = x;
        cloud->points[theta % 360].y = y;
        cloud->points[theta % 360].z = 0.0;
    }

    return cloud;
}