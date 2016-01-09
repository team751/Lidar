#include "../inc/lidar_data_processor.h"

#include <vector>

#include <opencv2/gpu/gpu.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "../inc/constants.h"
#include "../inc/coordinate_conversion.h"
#include "../inc/point_cloud_conversion.h"
#include "../inc/line_properties.h"

TotePose LidarDataProcessor::processLidarData(std::array<double, 360> lidarData) {
    // Create Mats
    cv::Mat lidarImage = PointCloudConversion::createImageWithLidarData(lidarData, 280, 400, 1, cv::Scalar(255), CV_8UC1, 1000, 1000);
    cv::Mat outputImage = PointCloudConversion::createImageWithLidarData(lidarData, 280, 400, 1, cv::Scalar(255, 0, 0), CV_8UC3, 1000, 1000);

//    cv::gpu::GpuMat gpuLidarImage(lidarImage);
//    cv::gpu::GpuMat gpuLines;
//    cv::gpu::HoughLinesBuf gpuLineBuf;
//
//    cv::gpu::HoughLinesP(gpuLidarImage, gpuLines, gpuLineBuf, Constants::kRho, Constants::kTheta, Constants::kThreshold, 10);
//
//    cv::vector<cv::Vec4i> houghLinesFound;
//    if (!gpuLines.empty()) {
//        houghLinesFound.resize(gpuLines.cols);
//        cv::Mat h_lines(1, gpuLines.cols, CV_32SC4, &houghLinesFound[0]);
//        gpuLines.download(h_lines);
//    }

    std::vector<cv::Vec4i> houghLinesFound;
    cv::HoughLinesP(lidarImage, houghLinesFound, Constants::kRho, Constants::kTheta, Constants::kThreshold, Constants::kMinLineLength, Constants::kMaxLineGap);

    bool lineFound;

    cv::Vec4i discoveredLine;
    cv::Vec4i actualLine;
    double minimumLengthDelta = UINT_MAX;
    double minimumLength = UINT_MAX;

    double distance;
    double length;
    double angle;
    double theoreticalLength;

    for (size_t i = 0; i < houghLinesFound.size(); i++) {
        cv::Vec4i line                        = houghLinesFound[i];
        cv::Vec4i transformedLine = LidarDataCoordinateConversion::convertCoordinateSystem(line,
                                                                                                       LidarDataCoordinateConversion::CoordinateSystem::ImageCoordinateSystem,
                                                                                                       LidarDataCoordinateConversion::CoordinateSystem::LidarCoordinateSystem);

        distance          = LineProperties::distanceFromLidar(transformedLine);
        length            = LineProperties::lengthOfLine(transformedLine);
        angle             = LineProperties::angleOfLine(transformedLine);
        theoreticalLength = LineProperties::theoreticalLength(transformedLine);

        double lengthDelta = fabs(theoreticalLength - length);
//        lengthDelta = fmin(lengthDelta, fabs(theoreticalLength - length / 2.0));
//        lengthDelta = fmin(lengthDelta, fabs(theoreticalLength - length / 3.0));

//        std::cout << theoreticalLength << " " << length << " " << distance << " " << ((Constants::kActualLength * distance) / length) << " " << angle << std::endl;

        if (lengthDelta < Constants::kLengthDeltaThreshold && lengthDelta < minimumLengthDelta && length < minimumLength) {
            minimumLengthDelta = lengthDelta;
            minimumLength = length;
            discoveredLine     = line;
        }

        cv::line(outputImage, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 3, CV_AA);
    }

    // If a line meeting out standards was found
    if (minimumLengthDelta != UINT_MAX) {
        // Create point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = PointCloudConversion::createPointCloudFromLidarData(lidarData, 280, 400);

        // Create Kd Tree
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        cv::Vec4i transformedInput = LidarDataCoordinateConversion::convertCoordinateSystem(discoveredLine, LidarDataCoordinateConversion::CoordinateSystem::ImageCoordinateSystem, LidarDataCoordinateConversion::CoordinateSystem::LidarCoordinateSystem);

        // Find actual points
        actualLine = LineProperties::fitLineToPoints(kdtree, cloud, transformedInput);

        lineFound = true;

        cv::Vec4i transformedLine = LidarDataCoordinateConversion::convertCoordinateSystem(actualLine, LidarDataCoordinateConversion::CoordinateSystem::LidarCoordinateSystem, LidarDataCoordinateConversion::CoordinateSystem::ImageCoordinateSystem);

        // Draw line
        cv::line(outputImage, cv::Point(transformedLine[0], transformedLine[1]), cv::Point(transformedLine[2], transformedLine[3]), cv::Scalar(0, 255, 0), 3, CV_AA);
    }

    cv::namedWindow("Output");
    cv::imshow("Output", outputImage);
    cv::waitKey(1);

    if (lineFound) return TotePose(TotePose::ToteEndpoint(actualLine[0], actualLine[1]), TotePose::ToteEndpoint(actualLine[2], actualLine[3]), LineProperties::angleOfLine(actualLine) * 180.0 / M_PI);
    else return TotePose(false);
}