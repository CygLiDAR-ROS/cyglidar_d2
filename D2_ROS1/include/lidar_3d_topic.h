#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>

#include "cygbot_constant.h"
#include "distance_processor.h"
#include "mapping_point_cloud.h"
#include "color_encoded_depth_amplitude.h"

using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class Lidar3dTopic
{
    public:
        Lidar3dTopic();
        virtual ~Lidar3dTopic();

        void initPublisher(ros::Publisher publisher_image, ros::Publisher publisher_point_3d);

        void assignPCL3D(const std::string& frame_id);
        void assignImage(const std::string& frame_id);

        void publishDepthFlatImage(ros::Time scan_start_time, uint16_t* distance_buffer_3d);
        void publishAmplitudeFlatImage(ros::Time scan_start_time, uint16_t* distance_buffer_3d);
        void publishDepthPointCloud3D(ros::Time scan_start_time, uint16_t* distance_buffer_3d);
        void publishAmplitudePointCloud3D(ros::Time scan_start_time, uint16_t* distance_buffer_3d);

        void checkAmplitudeStatus(bool enableCLAHE, uint8_t clip_limit, uint8_t tiles_grid_size, uint8_t* amplitude_buffer);
        void updateColorConfig(uint8_t color_mode, std::string& notice);

    private:
        void initColorMap();

        MappingPointCloud*          _map_point_cloud;
        ColorEncodedDepthAmplitude* _color_encode;

        ros::Publisher _publisher_image;
        ros::Publisher _publisher_point_3d;

        std::shared_ptr<sensor_msgs::Image>       _message_image;
        std::shared_ptr<sensor_msgs::PointCloud2> _message_point_cloud_3d;
        std::shared_ptr<pcl_XYZRGBA>              _pcl_3d;

        std::vector<ColorCode_t> _color_map;
        cv::Mat _processed_image;

        uint8_t _color_mode;
        bool _enable_clahe;
};
