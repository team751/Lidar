//
// Created by Sam Baumgarten on 10/7/15.
//

#ifndef LIDAR_LIDARDATAPROCESSOR_LINEPROPERTIES_H
#define LIDAR_LIDARDATAPROCESSOR_LINEPROPERTIES_H

#include <opencv2/core/core.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace LineProperties {
    double distanceFromLidar(cv::Vec4i line);
    double angleOfLine(cv::Vec4i line);
    double lengthOfLine(cv::Vec4i line);
    double theoreticalLength(cv::Vec4i line);
    cv::Vec4i fitLineToPoints(pcl::KdTreeFLANN<pcl::PointXYZ> kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Vec4i line);

};


#endif //LIDAR_LIDARDATAPROCESSOR_LINEPROPERTIES_H
