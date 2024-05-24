#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "cygbot_constant.h"

class DeviceStatusTopic
{
    public:
        void initPublisher(ros::Publisher publisher_device_status);
        void assignDeviceStatus();
        void getTemperature(int16_t temperature);

        void publishDeviceStatus();

    private:
        ros::Publisher _publisher_device_status;
        std::shared_ptr<std_msgs::Float32> _message_temperature;

        float _sensor_temperature;
};
