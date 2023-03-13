/**
 * @file ultra_sonic_radar_driver_main.cpp
 * @author Mark Jin (mark@pixmoving.net)
 * @brief 
 * @version 0.9
 * @date 2022-12-01
 * 
 * @copyright Copyright (c) 2022, Pixmoving
 * Rebuild The City With Autonomous Mobility
 * https://www.pixmoving.com
 * 
 */
#include <ultra_sonic_radar_driver/ultra_sonic_radar_driver.hpp>

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "ultra_sonic_radar_driver_node");
    ultra_sonic_radar_driver::UltraSonicRadarDriver ultra_sonic_radar_driver;
    ros::spin();

    return 0;
}
