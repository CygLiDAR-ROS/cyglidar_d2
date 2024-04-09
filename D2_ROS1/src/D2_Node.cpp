#include "D2_Node.h"

D2_Node::D2_Node()
{
    topic_2d    = new Topic2D();
    topic_3d    = new Topic3D();
    cyg_driver  = new CYG_Driver();
    serial_port = new CYG_SerialUart();

    topic_2d->initPublisher(nh.advertise<sensor_msgs::LaserScan>  ("scan",       10),
                            nh.advertise<sensor_msgs::PointCloud2>("scan_2D",    10));

    topic_3d->initPublisher(nh.advertise<sensor_msgs::Image>      ("scan_image", 10),
                            nh.advertise<sensor_msgs::PointCloud2>("scan_3D",    10));

    received_buffer[0].packet_data = first_total_packet_data;
    received_buffer[1].packet_data = second_total_packet_data;

    initConfiguration();

    future = exit_signal.get_future();

    double_buffer_thread = std::thread(&D2_Node::doublebufferThread, this);
    publish_thread       = std::thread(&D2_Node::publishThread, this);
}

D2_Node::~D2_Node()
{
    exit_signal.set_value();

    double_buffer_thread.join();
    publish_thread.join();

    delete topic_2d;
    delete topic_3d;
    delete cyg_driver;
    delete serial_port;

    topic_2d    = nullptr;
    topic_3d    = nullptr;
    cyg_driver  = nullptr;
    serial_port = nullptr;
}

void D2_Node::connectBoostSerial()
{
    try
    {
        serial_port->openSerialPort(port_number, baud_rate_mode);

        requestPacketData();
    }
    catch (const boost::system::system_error& ex)
    {
        ROS_ERROR("[BOOST SERIAL ERROR] %s", ex.what());
    }
}

void D2_Node::disconnectBoostSerial()
{
    serial_port->closeSerialPort();
    ROS_ERROR("[PACKET REQUEST] STOP");
}

void D2_Node::loopCygParser()
{
    number_of_data = serial_port->getPacketLength(packet_structure);

    for (uint16_t i = 0; i < number_of_data; i++)
    {
        parser_return = cyg_driver->CygParser(received_buffer[double_buffer_index].packet_data, packet_structure[i]);

        if(parser_return == D2_Const::CHECKSUM_PASSED)
        {
            received_buffer[double_buffer_index].parsing_end_time = ros::Time::now();

            publish_done_flag |= (1 << double_buffer_index);

            double_buffer_index++;
            double_buffer_index &= 1;
        }
        else if (parser_return == D2_Const::PARSING_STARTED)
        {
            received_buffer[double_buffer_index].parsing_start_time = ros::Time::now();
        }
    }
}

void D2_Node::initConfiguration()
{
    ros::NodeHandle priv_nh("~");

    priv_nh.param<std::string>("port_number",           port_number,           "/dev/ttyUSB0");
    priv_nh.param<int>        ("baud_rate",             baud_rate_mode,        0);
    priv_nh.param<std::string>("frame_id",              frame_id,              "laser_frame");
    priv_nh.param<int>        ("run_mode",              run_mode,              ROS_Const::MODE_DUAL);
    priv_nh.param<int>        ("data_type_3d",          data_type_3d,          ROS_Const::MODE_DISTANCE);
    priv_nh.param<int>        ("duration_mode",         duration_mode,         ROS_Const::PULSE_AUTO);
    priv_nh.param<int>        ("duration_value",        duration_value,        10000);
    priv_nh.param<int>        ("frequency_channel",     frequency_channel,     0);
    priv_nh.param<int>        ("color_mode",            color_mode,            ROS_Const::MODE_HUE);
    priv_nh.param<int>        ("filter_mode",           filter_mode,           ROS_Const::NONE_FILTER);
    priv_nh.param<bool>       ("enable_kalmanfilter",   enable_kalmanfilter,   true);
    priv_nh.param<bool>       ("enable_clahe",          enable_clahe,          true);
    priv_nh.param<int>        ("clahe_cliplimit",       clahe_cliplimit,       40);
    priv_nh.param<int>        ("clahe_tiles_grid_size", clahe_tiles_grid_size, 8);

    topic_2d->assignLaserScan(frame_id);
    topic_2d->assignPCL2D(frame_id);
    topic_3d->assignImage(frame_id);
    topic_3d->assignPCL3D(frame_id);

    topic_3d->updateColorConfig(color_mode, mode_notice);
}

void D2_Node::requestPacketData()
{
    serial_port->requestDeviceInfo();
    ros::Duration(3.0).sleep();
    // sleep for 3s, by requsting the info data.

    ROS_INFO("[COLOR MODE] %s", mode_notice.c_str());

    if (enable_kalmanfilter)
    {
        ROS_INFO("[KALMAN FILTER] APPLIED");
    }
    else
    {
        ROS_INFO("[KALMAN FILTER] NONE APPLIED");
    }

    serial_port->requestSwitch3DType(data_type_3d, mode_notice);
    ROS_INFO("[PACKET REQUEST] %s", mode_notice.c_str());

    serial_port->requestNewFiltering(run_mode, filter_mode, mode_notice);
    ROS_INFO("[PACKET REQUEST] %s", mode_notice.c_str());

    serial_port->requestEdgeFiltering(run_mode, edge_filter_value);
    ROS_INFO("[PACKET REQUEST] EDGE FILTERING : %d", edge_filter_value);

    serial_port->requestDurationControl(run_mode, duration_mode, duration_value);
    ros::Duration(1.0).sleep();
    ROS_INFO("[PACKET REQUEST] PULSE DURATION : %d", duration_value);
    // sleep for a sec, by requsting the duration

    serial_port->requestFrequencyChannel(frequency_channel);
    ROS_INFO("[PACKET REQUEST] FREQUENCY CH.%d", frequency_channel);

    serial_port->requestRunMode(run_mode, mode_notice);
    ROS_INFO("[PACKET REQUEST] %s", mode_notice.c_str());
}

void D2_Node::convertData(received_data_buffer* _received_buffer)
{
    ros::Duration time_for_scanning(_received_buffer->parsing_end_time - _received_buffer->parsing_start_time);

    if (_received_buffer->packet_data[D2_Const::PAYLOAD_HEADER] == D2_Const::PACKET_HEADER_2D)
    {
        cyg_driver->getDistanceArray2D(&_received_buffer->packet_data[D2_Const::PAYLOAD_INDEX], distance_buffer_2d);

        ros::Duration timestamp_nanosec_mode_2d(0, cyg_driver->setTimeStamp2D() * 1000);
        ros::Duration timestamp_scan_started(time_for_scanning + timestamp_nanosec_mode_2d);

        start_time_scan_2d = ros::Time::now() - timestamp_scan_started;

        publish_data_state = ROS_Const::PUBLISH_2D;
    }
    else if (_received_buffer->packet_data[D2_Const::PAYLOAD_HEADER] == D2_Const::PACKET_HEADER_3D)
    {
        cyg_driver->getDistanceArray3D(&_received_buffer->packet_data[D2_Const::PAYLOAD_INDEX], distance_buffer_3d, enable_kalmanfilter);

        ros::Duration timestamp_nanosec_mode_3d(0, cyg_driver->setTimeStamp3D() * 1000);
        ros::Duration timestamp_scan_started(time_for_scanning + timestamp_nanosec_mode_3d);

        start_time_scan_3d = ros::Time::now() - timestamp_scan_started;

        publish_data_state = ROS_Const::PUBLISH_3D;
    }
    else if (_received_buffer->packet_data[D2_Const::PAYLOAD_HEADER] == D2_Const::PACKET_HEADER_AMPLITUDE_3D)
    {
        cyg_driver->getDistanceAndAmpliutdeArray3D(&_received_buffer->packet_data[D2_Const::PAYLOAD_INDEX], distance_buffer_3d, amplitude_buffer_3d, enable_kalmanfilter);

        ros::Duration timestamp_nanosec_mode_3d(0, cyg_driver->setTimeStamp3D() * 1000);
        ros::Duration timestamp_scan_started(time_for_scanning + timestamp_nanosec_mode_3d);

        start_time_scan_3d = ros::Time::now() - timestamp_scan_started;

        publish_data_state = ROS_Const::PUBLISH_3D;
    }
    else if (_received_buffer->packet_data[D2_Const::PAYLOAD_HEADER] == D2_Const::PACKET_HEADER_DEVICE_INFO && info_flag == false)
    {
        ROS_INFO("[F/W VERSION] %d.%d.%d", _received_buffer->packet_data[6], _received_buffer->packet_data[7],  _received_buffer->packet_data[8]);
        ROS_INFO("[H/W VERSION] %d.%d.%d", _received_buffer->packet_data[9], _received_buffer->packet_data[10], _received_buffer->packet_data[11]);
        info_flag = true;
    }
}

void D2_Node::processDoubleBuffer()
{
    if(publish_done_flag & 0x1)
    {
        publish_done_flag &= (~0x1);
        convertData(&received_buffer[0]);
    }
    else if(publish_done_flag & 0x2)
    {
        publish_done_flag &= (~0x2);
        convertData(&received_buffer[1]);
    }
}

void D2_Node::runPublish()
{
    if (publish_data_state == ROS_Const::PUBLISH_3D)
    {
        if (data_type_3d == ROS_Const::MODE_DISTANCE)
        {
            topic_3d->publishDepthFlatImage(start_time_scan_3d, distance_buffer_3d);
            topic_3d->publishDepthPointCloud3D(start_time_scan_3d, distance_buffer_3d);
        }
        else if (data_type_3d == ROS_Const::MODE_AMPLITUDE)
        {
            topic_3d->checkAmplitudeStatus(enable_clahe, clahe_cliplimit, clahe_tiles_grid_size, amplitude_buffer_3d);

            topic_3d->publishAmplitudeFlatImage(start_time_scan_3d, distance_buffer_3d);
            topic_3d->publishAmplitudePointCloud3D(start_time_scan_3d, distance_buffer_3d);
        }

        publish_data_state = ROS_Const::PUBLISH_DONE;
    }
    else if (publish_data_state == ROS_Const::PUBLISH_2D)
    {
        topic_2d->applyPointCloud2D(distance_buffer_2d);
        topic_2d->publishPoint2D(start_time_scan_2d);
        topic_2d->publishScanLaser(start_time_scan_2d, distance_buffer_2d);

        publish_data_state = ROS_Const::PUBLISH_DONE;
    }
}

void D2_Node::doublebufferThread()
{
    do
    {
        processDoubleBuffer();
        status = future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
}

void D2_Node::publishThread()
{
    do
    {
        runPublish();
        status = future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
}
