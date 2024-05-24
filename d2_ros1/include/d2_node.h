#pragma once

#include <thread>
#include <future>
#include <ros/ros.h>

#include "cygbot_constant.h"
#include "cygbot_parser.h"
#include "distance_processor.h"
#include "serial_uart.h"
#include "distortion_table.h"
#include "lidar_2d_topic.h"
#include "lidar_3d_topic.h"
#include "device_status_topic.h"

class D2Node
{
    public:
        explicit D2Node();
        virtual ~D2Node();

        void connectBoostSerial();
        void disconnectBoostSerial();
        void loopCygParser();

    private:
        struct received_data_buffer
        {
            ros::Time parsing_start_time;
            ros::Time parsing_end_time;
            uint8_t* packet_data;
        }received_buffer[2];

        void initConfiguration();
        void requestPacketData();
        void convertData(received_data_buffer* _received_buffer);
        void processDoubleBuffer();
        void runPublish();
        void doublebufferThread();
        void publishThread();

        Lidar2dTopic*      topic_2d;
        Lidar3dTopic*      topic_3d;
        DeviceStatusTopic* status_topic;
        SerialUart*        serial_uart;
        DistanceProcessor* distance_processor;
        CygbotParser*      cygbot_parser;

        std::string port_number;
        int         baud_rate_mode;
        std::string frame_id;
        int         run_mode;
        int         data_type_3d;
        int         duration_mode;
        int         duration_value;
        int         frequency_channel;
        int         color_mode;
        int         filter_mode;
        int         edge_filter_value;
        bool        enable_kalmanfilter;
        bool        enable_clahe;
        int         clahe_cliplimit;
        int         clahe_tiles_grid_size;

        ros::NodeHandle nh;
        ros::Time start_time_scan_2d;
        ros::Time start_time_scan_3d;

        std::thread double_buffer_thread;
        std::thread publish_thread;

        std::shared_future<void> future;
        std::promise<void> exit_signal;
        std::future_status status;

        std::string mode_notice;

        uint8_t packet_structure[D2_Const::SCAN_MAX_SIZE];
        uint8_t first_total_packet_data[D2_Const::SCAN_MAX_SIZE];
        uint8_t second_total_packet_data[D2_Const::SCAN_MAX_SIZE];
        uint8_t amplitude_buffer_3d[D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT];
        uint16_t distance_buffer_2d[DATA_LENGTH_2D];
        uint16_t distance_buffer_3d[DATA_LENGTH_3D];

        uint8_t  publish_done_flag;
        uint8_t  publish_data_state;
        uint8_t  double_buffer_index;
};
