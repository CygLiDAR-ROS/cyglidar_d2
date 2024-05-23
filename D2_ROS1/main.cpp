#include "d2_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "D2_NODE");

    std::shared_ptr<D2Node> d2_node = std::make_shared<D2Node>();

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
        ROS_ERROR("[D2 NODE ERROR] : %s", e.what());
    }

    ros::shutdown();
}
