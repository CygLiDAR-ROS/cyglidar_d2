#include "Topic3D.h"

Topic3D::Topic3D()
{
    cyg_pcl    = new CYG_PCL();
    cyg_opencv = new CYG_OpenCV();
    message_point_cloud_3d = std::make_shared<sensor_msgs::PointCloud2>();
    message_image          = std::make_shared<sensor_msgs::Image>();
}

Topic3D::~Topic3D()
{
	delete cyg_pcl;
	delete cyg_opencv;

	cyg_pcl    = nullptr;
	cyg_opencv = nullptr;
}

void Topic3D::initPublisher(ros::Publisher _publisher_image, ros::Publisher _publisher_point_3d)
{
    publisher_image    = _publisher_image;
    publisher_point_3d = _publisher_point_3d;
}

void Topic3D::assignPCL3D(const std::string& _frame_id)
{
    pcl_3d.reset(new pcl_XYZRGBA());

    pcl_3d->header.frame_id = _frame_id;
    pcl_3d->is_dense        = false;
    pcl_3d->width           = D2_Const::IMAGE_WIDTH;
    pcl_3d->height          = D2_Const::IMAGE_HEIGHT;
    pcl_3d->points.resize(DATA_LENGTH_3D);
}

void Topic3D::assignImage(const std::string& _frame_id)
{
    message_image->header.frame_id = _frame_id;
    message_image->width           = D2_Const::IMAGE_WIDTH;
    message_image->height          = D2_Const::IMAGE_HEIGHT;
    message_image->encoding        = sensor_msgs::image_encodings::RGBA8;
    message_image->step            = message_image->width * sizeof(uint32_t);
    message_image->is_bigendian    = false;
    message_image->data.resize(message_image->height * message_image->step);
}

void Topic3D::publishDepthFlatImage(ros::Time _scan_start_time, uint16_t* _distance_buffer_3d)
{
    message_image->header.stamp = _scan_start_time;

	memcpy(message_image->data.data(), cyg_opencv->applyDepthFlatImage(_distance_buffer_3d).data, message_image->height * message_image->width * sizeof(uint32_t));

    publisher_image.publish(*message_image);
}

void Topic3D::publishAmplitudeFlatImage(ros::Time _scan_start_time, uint16_t* _distance_buffer_3d)
{
    message_image->header.stamp = _scan_start_time;

	if (enable_clahe)
	{
		processed_image = cyg_opencv->applyAmplitudeFlatImage(_distance_buffer_3d, cyg_opencv->matrix_clahe_applied_amplitude);
		memcpy(message_image->data.data(), processed_image.data, message_image->height * message_image->width * sizeof(uint32_t));
	}
	else
	{
		processed_image = cyg_opencv->applyAmplitudeFlatImage(_distance_buffer_3d, cyg_opencv->matrix_raw_amplitude);
		memcpy(message_image->data.data(), processed_image.data, message_image->height * message_image->width * sizeof(uint32_t));
	}

    publisher_image.publish(*message_image);
}

void Topic3D::publishDepthPointCloud3D(ros::Time _scan_start_time, uint16_t* _distance_buffer_3d)
{
    pcl_conversions::toPCL(_scan_start_time, pcl_3d->header.stamp);

    cyg_pcl->applyPointCloud3DColors(pcl_3d, _distance_buffer_3d);

    pcl::toROSMsg(*pcl_3d, *message_point_cloud_3d); //change type from pointcloud(pcl) to ROS message(PointCloud2)
    publisher_point_3d.publish(*message_point_cloud_3d);
}

void Topic3D::publishAmplitudePointCloud3D(ros::Time _scan_start_time, uint16_t* _distance_buffer_3d)
{
    pcl_conversions::toPCL(_scan_start_time, pcl_3d->header.stamp);

	if (enable_clahe)
	{
		cyg_pcl->applyAmplitudePointCloud(pcl_3d, _distance_buffer_3d, cyg_opencv->matrix_clahe_applied_amplitude);
	}
	else
	{
		cyg_pcl->applyAmplitudePointCloud(pcl_3d, _distance_buffer_3d, cyg_opencv->matrix_raw_amplitude);
	}

    pcl::toROSMsg(*pcl_3d, *message_point_cloud_3d);
    publisher_point_3d.publish(*message_point_cloud_3d);
}

void Topic3D::updateColorConfig(uint8_t _color_mode, std::string& _notice)
{
    color_mode = _color_mode;

    // Call the following function so as to store colors to draw 3D data
    initColorMap();

    if (color_mode == ROS_Const::MODE_HUE)
    {
        _notice = "HUE MODE";
    }
    else if (_color_mode == ROS_Const::MODE_RGB)
    {
        _notice = "RGB MODE";
    }
    else if (_color_mode == ROS_Const::MODE_GRAY)
    {
        _notice = "GRAY MODE";
    }

    cyg_pcl->getColorMap(color_map);
    cyg_opencv->getColorMap(color_map);
}

void Topic3D::checkAmplitudeStatus(bool _enable_clahe, uint8_t clip_limit, uint8_t tiles_grid_size, uint8_t* _amplitude_data)
{
	enable_clahe = _enable_clahe;

	for (uint16_t i = 0; i < D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT; i++)
    {
        cyg_opencv->matrix_raw_amplitude.at<uint8_t>(0, i) = _amplitude_data[i];
    }

	if (enable_clahe)
	{
		cyg_opencv->applyCLAHE(clip_limit, tiles_grid_size);
	}
}

void Topic3D::initColorMap()
{
	r_setup = 0;
	g_setup = 0;
	b_setup = 0;

	if (color_mode == ROS_Const::MODE_HUE)
	{
		color_array = 5;
		b_setup = 255;
	}
	else if (color_mode == ROS_Const::MODE_RGB)
	{
		color_array = 2;
		b_setup = 255;
	}
	else if (color_mode == ROS_Const::MODE_GRAY)
	{
		color_array = 1;
		b_setup = 0;
	}

    // Iterate for-loop of adding RGB value to an array
	for (uint8_t i = 0; i < color_array; i++)
	{
		for (uint8_t color_count = 0; color_count < 255; color_count++)
		{
			if (color_mode == ROS_Const::MODE_GRAY)
			{
				switch (i)
				{
					case 0:
						r_setup++;
						g_setup++;
						b_setup++;
						break;
				}
			}
			else
			{
				switch (i)
				{
					case 0: // BLUE -> YELLOW
					case 3:
						r_setup++;
						g_setup++;
						b_setup--;
						break;
					case 1: // YELLOW -> RED
					case 4:
						g_setup--;
						break;
					case 2: // RED -> BLUE
						r_setup--;
						b_setup++;
						break;
				}
			}

			color_map.push_back({r_setup, g_setup, b_setup, 0xFF});
		}
	}
}
