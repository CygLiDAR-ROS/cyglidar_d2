#include "CYG_PCL.h"

CYG_PCL::CYG_PCL()
{
    cyg_distortion = new CYG_Distortion();

    cyg_distortion->initLensTransform(ROS_Const::PIXEL_REAL_SIZE, D2_Const::IMAGE_WIDTH, D2_Const::IMAGE_HEIGHT,
                                      ROS_Const::OFFSET_CENTER_POINT_X, ROS_Const::OFFSET_CENTER_POINT_Y);
}

CYG_PCL::~CYG_PCL()
{
    delete cyg_distortion;
    cyg_distortion = nullptr;
}

void CYG_PCL::applyPointCloud3DColors(std::shared_ptr<pcl_XYZRGBA> &_pcl_3d, uint16_t* _distance_buffer_3d)
{
    for (buffer_index = 0; buffer_index < D2_Const::IMAGE_HEIGHT * D2_Const::IMAGE_WIDTH; buffer_index++)
    {
        raw_distance = _distance_buffer_3d[buffer_index];

        if(raw_distance < D2_Const::DISTANCE_MAX_VALUE_3D)
        {
            cyg_distortion->transformPixel(buffer_index, raw_distance, real_world_coordinate_x, real_world_coordinate_y, real_world_coordinate_z);

            _pcl_3d->points[buffer_index].x =  real_world_coordinate_z * ROS_Const::MM2M;
            _pcl_3d->points[buffer_index].y = -real_world_coordinate_x * ROS_Const::MM2M;
            _pcl_3d->points[buffer_index].z = -real_world_coordinate_y * ROS_Const::MM2M;

            color_level = (int)((float)raw_distance / color_gap) >= total_color_number ? (total_color_number - 1) : (int)raw_distance / color_gap;

            rgb_setup = ((uint32_t)color_map[color_level].R << 16 | (uint32_t)color_map[color_level].G << 8 | (uint32_t)color_map[color_level].B);
            _pcl_3d->points[buffer_index].rgb = *reinterpret_cast<float *>(&rgb_setup);
            _pcl_3d->points[buffer_index].a = color_map[color_level].A;
        }
        else
        {
            _pcl_3d->points[buffer_index].x = 0;
            _pcl_3d->points[buffer_index].y = 0;
            _pcl_3d->points[buffer_index].z = 0;
            _pcl_3d->points[buffer_index].a = 0;
        }
    }
}

void CYG_PCL::applyAmplitudePointCloud(std::shared_ptr<pcl_XYZRGBA> &_pcl_3d, uint16_t* _distance_buffer_3d, cv::Mat& _amplitude_matrix)
{
    for (buffer_index = 0; buffer_index < D2_Const::IMAGE_HEIGHT * D2_Const::IMAGE_WIDTH; buffer_index++)
    {
        raw_distance = _distance_buffer_3d[buffer_index];

        if(raw_distance < D2_Const::DISTANCE_MAX_VALUE_3D)
        {
            cyg_distortion->transformPixel(buffer_index, raw_distance, real_world_coordinate_x, real_world_coordinate_y, real_world_coordinate_z);

            _pcl_3d->points[buffer_index].x =  real_world_coordinate_z * ROS_Const::MM2M;
            _pcl_3d->points[buffer_index].y = -real_world_coordinate_x * ROS_Const::MM2M;
            _pcl_3d->points[buffer_index].z = -real_world_coordinate_y * ROS_Const::MM2M;

            rgb_setup = ((uint32_t)_amplitude_matrix.at<uint8_t>(0, buffer_index) << 16 | (uint32_t)_amplitude_matrix.at<uint8_t>(0, buffer_index) << 8 | (uint32_t)_amplitude_matrix.at<uint8_t>(0, buffer_index));
            _pcl_3d->points[buffer_index].rgb = *reinterpret_cast<float *>(&rgb_setup);
            _pcl_3d->points[buffer_index].a = 0xFF;
        }
        else
        {
            _pcl_3d->points[buffer_index].x = 0;
            _pcl_3d->points[buffer_index].y = 0;
            _pcl_3d->points[buffer_index].z = 0;
            _pcl_3d->points[buffer_index].a = 0;
        }
    }
}

void CYG_PCL::getColorMap(std::vector<ColorCode_t> &_color_map)
{
    this->color_map = _color_map;

    total_color_number = color_map.size();
    color_gap = D2_Const::DISTANCE_MAX_VALUE_3D / total_color_number;
}
