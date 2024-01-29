#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "CYG_Constant.h"
#include "CYG_Driver.h"
#include "CYG_PCL.h"
#include "CYG_OpenCV.h"

using Image = sensor_msgs::msg::Image;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class Topic3D
{
    public:
        Topic3D();
        virtual ~Topic3D();

        void initPublisher(rclcpp::Publisher<Image>::SharedPtr _publisher_image, rclcpp::Publisher<PointCloud2>::SharedPtr _publisher_point_3d);

        void assignImage(const std::string& _frame_id);
        void assignPCL3D(const std::string& _frame_id);

        void publishDepthFlatImage(rclcpp::Time _scan_start_time, uint16_t* _distance_buffer_3d);
        void publishAmplitudeFlatImage(rclcpp::Time _scan_start_time, uint16_t* _distance_buffer_3d);
        void publishDepthPointCloud3D(rclcpp::Time _scan_start_time, uint16_t* _distance_buffer_3d);
        void publishAmplitudePointCloud3D(rclcpp::Time _scan_start_time, uint16_t* _distance_buffer_3d);

        void checkAmplitudeStatus(bool enableCLAHE, uint8_t clip_limit, uint8_t tiles_grid_size, uint8_t* _amplitude_buffer);
        void updateColorConfig(uint8_t _color_mode, std::string& _notice);

    private:
		void initColorMap();

        rclcpp::Publisher<Image>::SharedPtr       publisher_image;
        rclcpp::Publisher<PointCloud2>::SharedPtr publisher_point_3d;

        std::shared_ptr<Image>       message_image;
        std::shared_ptr<PointCloud2> message_point_cloud_3d;
        std::shared_ptr<pcl_XYZRGBA> pcl_3d;
	    cv::Mat processed_image;        

        CYG_PCL*    cyg_pcl;
        CYG_OpenCV* cyg_opencv;

        std::vector<ColorCode_t> color_map;
        uint8_t r_setup, g_setup, b_setup;
		uint8_t color_array;
        uint8_t color_mode;

        bool enable_clahe;
};
