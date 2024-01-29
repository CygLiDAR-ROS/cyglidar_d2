#include "CYG_OpenCV.h"

void CYG_OpenCV::applyCLAHE(uint8_t _clip_limit, uint8_t _tiles_grid_size)
{
    clahe = cv::createCLAHE();
    clahe->setClipLimit(_clip_limit);
    clahe->setTilesGridSize(cv::Size(_tiles_grid_size, _tiles_grid_size));

    clahe->apply(matrix_raw_amplitude, matrix_clahe_applied_amplitude);
}

cv::Mat CYG_OpenCV::applyDepthFlatImage(uint16_t* _distance_buffer_3d)
{
    for (uint8_t y = 0; y < D2_Const::IMAGE_HEIGHT; y++)
    {
        for (uint8_t x = 0; x < D2_Const::IMAGE_WIDTH; x++)
        {
            image_step = (x * sizeof(uint32_t)) + (y * D2_Const::IMAGE_WIDTH * sizeof(uint32_t));
            buffer_index = x + (y * D2_Const::IMAGE_WIDTH);

            raw_distance = _distance_buffer_3d[buffer_index];
            color_level = (int)((float)raw_distance / color_gap) >= total_color_number ? (total_color_number - 1) : (int)raw_distance / color_gap;

            if (raw_distance < D2_Const::DISTANCE_MAX_VALUE_3D)
            {
                matrix_color.at<uint8_t>(0, image_step)     = color_map[color_level].R;
                matrix_color.at<uint8_t>(0, image_step + 1) = color_map[color_level].G;
                matrix_color.at<uint8_t>(0, image_step + 2) = color_map[color_level].B;
                matrix_color.at<uint8_t>(0, image_step + 3) = color_map[color_level].A;
            }
            else if (raw_distance == D2_Const::ADC_OVERFLOW_3D)
            {
                matrix_color.at<uint8_t>(0, image_step)     = ROS_Const::ADC_OVERFLOW_COLOR.R;
                matrix_color.at<uint8_t>(0, image_step + 1) = ROS_Const::ADC_OVERFLOW_COLOR.G;
                matrix_color.at<uint8_t>(0, image_step + 2) = ROS_Const::ADC_OVERFLOW_COLOR.B;
                matrix_color.at<uint8_t>(0, image_step + 3) = ROS_Const::ADC_OVERFLOW_COLOR.A;
            }
            else if (raw_distance == D2_Const::SATURATION_3D)
            {
                matrix_color.at<uint8_t>(0, image_step)     = ROS_Const::SATURATION_COLOR.R;
                matrix_color.at<uint8_t>(0, image_step + 1) = ROS_Const::SATURATION_COLOR.G;
                matrix_color.at<uint8_t>(0, image_step + 2) = ROS_Const::SATURATION_COLOR.B;
                matrix_color.at<uint8_t>(0, image_step + 3) = ROS_Const::SATURATION_COLOR.A;
            }
            else
            {
                matrix_color.at<uint8_t>(0, image_step)     = ROS_Const::NONE_COLOR.R;
                matrix_color.at<uint8_t>(0, image_step + 1) = ROS_Const::NONE_COLOR.G;
                matrix_color.at<uint8_t>(0, image_step + 2) = ROS_Const::NONE_COLOR.B;
                matrix_color.at<uint8_t>(0, image_step + 3) = ROS_Const::NONE_COLOR.A;
            }
        }
    }

    return matrix_color;
}

cv::Mat CYG_OpenCV::applyAmplitudeFlatImage(uint16_t* _distance_buffer_3d, cv::Mat& _amplitude_matrix)
{
    for (uint8_t y = 0; y < D2_Const::IMAGE_HEIGHT; y++)
    {
        for (uint8_t x = 0; x < D2_Const::IMAGE_WIDTH; x++)
        {
            image_step = (x * sizeof(uint32_t)) + (y * D2_Const::IMAGE_WIDTH * sizeof(uint32_t));
            buffer_index = x + (y * D2_Const::IMAGE_WIDTH);

            raw_distance = _distance_buffer_3d[buffer_index];

            if (raw_distance < D2_Const::DISTANCE_MAX_VALUE_3D)
            {
                matrix_color.at<uint8_t>(0, image_step)     = _amplitude_matrix.at<uint8_t>(0, buffer_index);
                matrix_color.at<uint8_t>(0, image_step + 1) = _amplitude_matrix.at<uint8_t>(0, buffer_index);
                matrix_color.at<uint8_t>(0, image_step + 2) = _amplitude_matrix.at<uint8_t>(0, buffer_index);
                matrix_color.at<uint8_t>(0, image_step + 3) = _amplitude_matrix.at<uint8_t>(0, buffer_index);
            }
            else if (raw_distance == D2_Const::ADC_OVERFLOW_3D)
            {
                matrix_color.at<uint8_t>(0, image_step)     = ROS_Const::ADC_OVERFLOW_COLOR.R;
                matrix_color.at<uint8_t>(0, image_step + 1) = ROS_Const::ADC_OVERFLOW_COLOR.G;
                matrix_color.at<uint8_t>(0, image_step + 2) = ROS_Const::ADC_OVERFLOW_COLOR.B;
                matrix_color.at<uint8_t>(0, image_step + 3) = ROS_Const::ADC_OVERFLOW_COLOR.A;
            }
            else if (raw_distance == D2_Const::SATURATION_3D)
            {
                matrix_color.at<uint8_t>(0, image_step)     = ROS_Const::SATURATION_COLOR.R;
                matrix_color.at<uint8_t>(0, image_step + 1) = ROS_Const::SATURATION_COLOR.G;
                matrix_color.at<uint8_t>(0, image_step + 2) = ROS_Const::SATURATION_COLOR.B;
                matrix_color.at<uint8_t>(0, image_step + 3) = ROS_Const::SATURATION_COLOR.A;
            }
            else
            {
                matrix_color.at<uint8_t>(0, image_step)     = ROS_Const::NONE_COLOR.R;
                matrix_color.at<uint8_t>(0, image_step + 1) = ROS_Const::NONE_COLOR.G;
                matrix_color.at<uint8_t>(0, image_step + 2) = ROS_Const::NONE_COLOR.B;
                matrix_color.at<uint8_t>(0, image_step + 3) = ROS_Const::NONE_COLOR.A;
            }
        }
    }

    return matrix_color;
}

void CYG_OpenCV::getColorMap(std::vector<ColorCode_t> &_color_map)
{
    this->color_map = _color_map;

    total_color_number = color_map.size();
    color_gap = D2_Const::DISTANCE_MAX_VALUE_3D / total_color_number;
}

