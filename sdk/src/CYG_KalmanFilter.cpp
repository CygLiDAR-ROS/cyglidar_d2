#include "CYG_KalmanFilter.h"

CYG_KalmanFilter::CYG_KalmanFilter()
{
    A = 1.0;
    H = 1.0;
    Q = 50.0;
    R = 200.0; 
    transpose_of_A = 1.0;
    transpose_of_H = 1.0;

    initKalmanFilter();
}

CYG_KalmanFilter::~CYG_KalmanFilter()
{
    delete kalman_buffer;

    kalman_buffer = nullptr;
}

uint16_t CYG_KalmanFilter::applyKalmanFiltering(uint16_t _buffer_index, uint16_t _raw_data)
{
    uint16_t result;

    if (_raw_data < D2_Const::INTERFERENCE_3D)
    {
        kalman_data = runKalmanFiltering(_buffer_index, _raw_data);

        if (abs(kalman_data - _raw_data) < 40)
        {
            result = kalman_data;
        }
        else
        {
            reinitKalmanFilter(_buffer_index, _raw_data);
            result = _raw_data; 
        }
    }
    else
    {
        result = _raw_data;
    }

    return result;
}

void CYG_KalmanFilter::initKalmanFilter()
{
    kalman_buffer = new KalmanValues_t[D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT];

    clearKalmanBuffer();
}

void CYG_KalmanFilter::reinitKalmanFilter(uint16_t _index, uint16_t _raw_data)
{
    kalman_buffer[_index].x = _raw_data;
}

uint16_t CYG_KalmanFilter::runKalmanFiltering(uint16_t _index, uint16_t _raw_data)
{
    z  = static_cast<float>(_raw_data);
    xp =  A * kalman_buffer[_index].x;
    Pp = (A * kalman_buffer[_index].P * transpose_of_A) + Q;

    kalman_buffer[_index].x = xp + (kalman_buffer[_index].K * (z - (H * xp)));
    kalman_buffer[_index].K = (Pp * transpose_of_H) / ((H * Pp * transpose_of_H) + R);
    kalman_buffer[_index].P = Pp - (kalman_buffer[_index].K * H * Pp);

    return static_cast<uint16_t>(kalman_buffer[_index].x);
}

void CYG_KalmanFilter::clearKalmanBuffer()
{
    for (uint16_t i = 0; i < D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT; i++)
    {
        kalman_buffer[i].x = 0.0;
        kalman_buffer[i].K = 0.0;
        kalman_buffer[i].P = 20.0;
    }
}
