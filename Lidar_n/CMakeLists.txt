# Project Definition
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(Lidar)

# Subdirs
subdirs(LidarMain LidarDataCommunication LidarDataProcessor LidarSerialDataCollector)