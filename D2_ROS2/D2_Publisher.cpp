#include "D2_Node.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<D2_Node> d2_node = std::make_shared<D2_Node>();

    try
    {
        d2_node->connectBoostSerial();

        while(rclcpp::ok())
        {
            d2_node->loopCygParser();
        }

        d2_node->disconnectBoostSerial();
    }
    catch (const rclcpp::exceptions::RCLError& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[D2 NODE ERROR] : %s", e.what());
    }

    rclcpp::shutdown();
}
