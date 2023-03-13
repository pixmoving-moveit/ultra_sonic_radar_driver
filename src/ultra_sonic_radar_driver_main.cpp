/**
 * @file ultra_sonic_radar_driver_main.cpp
 * @author zumoude (zymouse@pixmoving.net)
 * @brief ultra sonic radar driver 
 * @version 1.0
 * @date 2023-03-08
 * 
 * @copyright Copyright (c) 2022, Pixmoving
 * Rebuild The City With Autonomous Mobility
 * https://www.pixmoving.com
 * 
 */
#include "ultra_sonic_radar_driver/ultra_sonic_radar_driver.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node=std::make_shared<ultra_sonic_radar_driver::UltraSonicRadarDriver>("ultra_sonic_radar_driver_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
