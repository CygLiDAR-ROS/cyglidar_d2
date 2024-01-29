#include "D2_Node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "D2_NODE");

    std::shared_ptr<D2_Node> d2_node = std::make_shared<D2_Node>();

    try
    {
        d2_node->connectBoostSerial();

        while(ros::ok())
        {
            d2_node->loopCygParser();
        }

        d2_node->disconnectBoostSerial();
    }
    catch (const ros::Exception &e)
    {
        ROS_ERROR("[D1 NODE ERROR] : %s", e.what());
    }

    ros::shutdown();
}
