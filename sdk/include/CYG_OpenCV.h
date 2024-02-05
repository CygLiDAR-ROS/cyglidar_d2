#pragma once

#include <opencv2/opencv.hpp>

#include "CYG_Constant.h"

class CYG_OpenCV
{
    public:
        void applyCLAHE(uint8_t _clip_limit, uint8_t _tiles_grid_size);

        cv::Mat applyDepthFlatImage(uint16_t* _distance_buffer_3d);
        cv::Mat applyAmplitudeFlatImage(uint16_t* _distance_buffer_3d, cv::Mat& _amplitude_matrix);

        void getColorMap(std::vector<ColorCode_t> &_color_map);

        cv::Mat matrix_raw_amplitude = cv::Mat::zeros(1, D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT, CV_8UC1);
        cv::Mat matrix_clahe_applied_amplitude = cv::Mat::zeros(1, D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT, CV_8UC1);

    private:
        cv::Mat matrix_color = cv::Mat::zeros(D2_Const::IMAGE_HEIGHT, D2_Const::IMAGE_WIDTH, CV_8UC4);
        cv::Ptr<cv::CLAHE> clahe;

        std::vector<ColorCode_t> color_map;

        uint16_t total_color_number;
        uint16_t buffer_index;
        uint16_t raw_distance;
        uint16_t color_level;
        uint16_t image_step;

        float color_gap;
};
