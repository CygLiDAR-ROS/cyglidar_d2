#include "device_status_topic.h"

void DeviceStatusTopic::initPublisher(ros::Publisher publisher_device_status)
{
    _publisher_device_status = publisher_device_status;
}

void DeviceStatusTopic::assignDeviceStatus()
{
    _message_temperature = std::make_shared<std_msgs::Float32>();

    _sensor_temperature = 0;
}

void DeviceStatusTopic::getTemperature(int16_t temperature)
{
    this->_sensor_temperature = temperature;
}

void DeviceStatusTopic::publishDeviceStatus()
{
    _message_temperature->data = _sensor_temperature / 256; //Celsius basis

    _publisher_device_status.publish(*_message_temperature);
}
