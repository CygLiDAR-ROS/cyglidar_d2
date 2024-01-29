#pragma once

#include <cstdint>
#include <cmath>

#include "CYG_Constant.h"

typedef struct KalmanFilterValues {
    float x;
    float K;
    float P;
} KalmanValues_t;

class CYG_KalmanFilter
{
    public:
        CYG_KalmanFilter();
        ~CYG_KalmanFilter();

        uint16_t applyKalmanFiltering(uint16_t _buffer_index, uint16_t _raw_data);

    private:
        void initKalmanFilter();
        void reinitKalmanFilter(uint16_t _index, uint16_t _raw_data);
        uint16_t runKalmanFiltering(uint16_t _buf_indexer_index, uint16_t _raw_data);
        void clearKalmanBuffer();

        KalmanValues_t* kalman_buffer;
        uint16_t kalman_data;

        float A;
        float H;
        float Q;
        float R; 
        float transpose_of_A;
        float transpose_of_H;

        float z;
        float xp;
        float Pp;
};
