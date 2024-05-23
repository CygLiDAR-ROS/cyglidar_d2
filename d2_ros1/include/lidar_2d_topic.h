#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "cygbot_constant.h"
#include "distance_processor.h"

using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class Lidar2dTopic
{
    public:
        void initPublisher(ros::Publisher publisher_laserscan, ros::Publisher publisher_point_2d);

        void applyPointCloud2D(uint16_t* distance_buffer_2d);

        void assignPCL2D(const std::string& frame_id);
        void publishPoint2D(ros::Time start_time);

        void assignLaserScan(const std::string& frame_id);
        void publishScanLaser(ros::Time start_time, uint16_t* distance_buffer_2d);

    private:
        ros::Publisher _publisher_laserscan;
        ros::Publisher _publisher_point_2d;

        std::shared_ptr<sensor_msgs::LaserScan>   _message_laserscan;
        std::shared_ptr<sensor_msgs::PointCloud2> _message_point_cloud_2d;
        std::shared_ptr<pcl_XYZRGBA>              _pcl_2d;
};
