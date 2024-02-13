#pragma once

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/mat.hpp>

#include "CYG_Constant.h"
#include "CYG_Distortion.h"

using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class CYG_PCL
{
    public:
        CYG_PCL();
        virtual ~CYG_PCL();

        void applyPointCloud3DColors(std::shared_ptr<pcl_XYZRGBA>& _pcl_3d, uint16_t* _distance_buffer_3d);
        void applyAmplitudePointCloud(std::shared_ptr<pcl_XYZRGBA>& _pcl_3d, uint16_t* _distance_buffer_3d, cv::Mat& _amplitude_matrix);

        void getColorMap(std::vector<ColorCode_t>& _color_map);

    private:
        CYG_Distortion* cyg_distortion;

        std::vector<ColorCode_t> color_map;

        uint16_t buffer_index;
        uint16_t raw_distance;
        uint16_t color_level;
        uint16_t total_color_number;
        uint32_t rgb_setup;

        float color_gap;
        float real_world_coordinate_x, real_world_coordinate_y, real_world_coordinate_z;
};
