#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>

#include "CYG_Constant.h"
#include "CYG_Driver.h"
#include "CYG_PCL.h"
#include "CYG_OpenCV.h"

using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class Topic3D
{
    public:
        Topic3D();
        virtual ~Topic3D();

        void initPublisher(ros::Publisher _publisher_image, ros::Publisher _publisher_point_3d);

        void assignPCL3D(const std::string& _frame_id);
        void assignImage(const std::string& _frame_id);

        void publishDepthFlatImage(ros::Time _scan_start_time, uint16_t* _distance_buffer_3d);
        void publishAmplitudeFlatImage(ros::Time _scan_start_time, uint16_t* _distance_buffer_3d);
        void publishDepthPointCloud3D(ros::Time _scan_start_time, uint16_t* _distance_buffer_3d);
        void publishAmplitudePointCloud3D(ros::Time _scan_start_time, uint16_t* _distance_buffer_3d);

        void checkAmplitudeStatus(bool enableCLAHE, uint8_t clip_limit, uint8_t tiles_grid_size, uint8_t* _amplitude_buffer);
        void updateColorConfig(uint8_t _color_mode, std::string& _notice);

    private:
        void initColorMap();

        ros::Publisher publisher_image;
        ros::Publisher publisher_point_3d;

        std::shared_ptr<sensor_msgs::Image>       message_image;
        std::shared_ptr<sensor_msgs::PointCloud2> message_point_cloud_3d;
        std::shared_ptr<pcl_XYZRGBA>              pcl_3d;
        cv::Mat processed_image;

        CYG_PCL*    cyg_pcl;
        CYG_OpenCV* cyg_opencv;

        std::vector<ColorCode_t> color_map;
        uint8_t r_setup, g_setup, b_setup;
		uint8_t color_array;
        uint8_t color_mode;

        bool enable_clahe;
};
