/**
 * @file ultra_sonic_radar_driver.cpp
 * @author Mark Jin (mark@pixmoving.net)
 * @brief ultra sonic radar ros driver for CM radars
 * @version 0.9
 * @date 2022-11-30
 * 
 * @copyright Copyright (c) 2022, Pixmoving
 * Rebuild The City With Autonomous Mobility
 * https://www.pixmoving.com
 * 
 */


#include "ultra_sonic_radar_driver/ultra_sonic_radar_driver.hpp"

namespace ultra_sonic_radar_driver
{

int dec2hex(uint8_t dec)
{
    int temp;
    int na[2];
    if(dec<=10)
    {
        return dec;
    }
    for(int i=0;i<2;i++)
    {
        if(dec<16)
        {
            na[i] = dec;
            dec = dec / 16;
        }
        else{
            temp = dec / 16;
            dec = dec % 16;
            na[i] = temp;
        }
    }
    return na[0]*10 + na[1]; 
}

void UltraSonicRadarData::setRange(const size_t &id, const float &range, const ros::Time stamp,
    const float &field_of_view_radian, const float &min_range_m, const float &max_range_m)
{
    Range range_msg;
    range_msg.header.frame_id = "ultrasonic_" + std::to_string(id);
    range_msg.header.stamp = stamp;
    range_msg.field_of_view = field_of_view_radian;
    range_msg.max_range = max_range_m;
    range_msg.min_range = min_range_m;
    range_msg.range = range;
    range_msg.radiation_type = Range::ULTRASOUND;
    RangeSharedPtr range_ptr = std::make_shared<Range>(range_msg);
    sensor_data_ptr_.at(id) = range_ptr;
}

UltraSonicRadarDriver::UltraSonicRadarDriver()
{
    // get parameters
    nh_.getParam("ultra_sonic_radar_driver_node/order", param_.order);
    nh_.getParam("ultra_sonic_radar_driver_node/field_of_view_radian", param_.field_of_view_radian);
    nh_.getParam("ultra_sonic_radar_driver_node/min_range_m", param_.min_range_m);
    nh_.getParam("ultra_sonic_radar_driver_node/max_range_m", param_.max_range_m);

    // subscriber
    activate_radar_sub_ = nh_.subscribe("input/activate_radar", 1, &UltraSonicRadarDriver::activateRadarCallback, this);
    can_frame_sub_ = nh_.subscribe("input/can_frame", 1, &UltraSonicRadarDriver::canFrameCallback, this);

    // publisher
    can_frame_pub_ = nh_.advertise<can_msgs::Frame>("output/can_frame", 1);
    for (int i = 0; i < 8;i++)
    {
        ros::Publisher ultra_sonic_range_pub;
        ultra_sonic_range_pub = nh_.advertise<sensor_msgs::Range>("output/ultra_sonic_radar_"+std::to_string(i), 1);
        ultra_sonic_range_pub_vector_.push_back(ultra_sonic_range_pub);
        ultra_sonic_radar_data_.sensor_data_ptr_.push_back(RangeSharedPtr());
    }

    // timer
    timer_ = nh_.createTimer(ros::Rate(10), &UltraSonicRadarDriver::timerCallback, this);

    // activate
    is_radar_activated_ = false;
    is_received_radar_data_ = false;
}

UltraSonicRadarDriver::~UltraSonicRadarDriver()
{

}

void UltraSonicRadarDriver::resetRadarData()
{
    for (int i = 0; i < 8;i++)
    {
        ultra_sonic_radar_data_.setRange(
            param_.order.at(i), 1000.0, ros::Time::now(),
            param_.field_of_view_radian.at(param_.order.at(i)), param_.min_range_m.at(param_.order.at(i)),
            param_.max_range_m.at(param_.order.at(i)));
    }
    publishData();
}

void UltraSonicRadarDriver::activateRadar()
{
    can_msgs::Frame activate_message;
    activate_message.header.stamp = ros::Time::now();
    activate_message.dlc = 3;
    activate_message.id = 0x601;
    activate_message.data[0] = 0xb7;
    activate_message.data[1] = 0x10;
    activate_message.data[2] = 0xff;
    can_frame_pub_.publish(activate_message);
    is_radar_activated_ = true;
    ROS_INFO("RADAR ACTIVATED");
}

void UltraSonicRadarDriver::deactivateRadar()
{
    can_msgs::Frame deactivate_message;
    deactivate_message.header.stamp = ros::Time::now();
    deactivate_message.dlc = 3;
    deactivate_message.id = 0x601;
    deactivate_message.data[0] = 0xb7;
    deactivate_message.data[1] = 0x10;
    deactivate_message.data[2] = 0x00;
    can_frame_pub_.publish(deactivate_message);
    is_radar_activated_ = false;
    resetRadarData();
    ROS_INFO("RADAR DEACTIVATED");
}

void UltraSonicRadarDriver::publishData()
{
    for (int i=0;i<8;i++)
    {
        ultra_sonic_range_pub_vector_.at(i).publish(
            *ultra_sonic_radar_data_.sensor_data_ptr_.at(i));
    }
}

void UltraSonicRadarDriver::activateRadarCallback(const std_msgs::BoolConstPtr &msg)
{
    if(msg->data)
    {
        activateRadar();
        ROS_INFO("ACTIVATING ULTRASONIC RADAR");
    }
    else
    {
        deactivateRadar();
        ROS_INFO("DEACTIVATING ULTRASONIC RADAR");
    }
}

void UltraSonicRadarDriver::canFrameCallback(const can_msgs::FrameConstPtr &msg)
{
    if(msg->id==0x611) {
        current_stamped_ = msg->header.stamp;
        ultra_sonic_radar_data_.setRange(
            param_.order.at(0), (dec2hex(msg->data.at(0))*100+dec2hex(msg->data.at(1)))/1000.0, msg->header.stamp,
            param_.field_of_view_radian.at(param_.order.at(0)), param_.min_range_m.at(param_.order.at(0)),
            param_.max_range_m.at(param_.order.at(0)));
        ultra_sonic_radar_data_.setRange(
            param_.order.at(1), (dec2hex(msg->data.at(2))*100+dec2hex(msg->data.at(3)))/1000.0, msg->header.stamp,
            param_.field_of_view_radian.at(param_.order.at(1)), param_.min_range_m.at(param_.order.at(1)),
            param_.max_range_m.at(param_.order.at(1)));
        ultra_sonic_radar_data_.setRange(
            param_.order.at(2), (dec2hex(msg->data.at(4))*100+dec2hex(msg->data.at(5)))/1000.0, msg->header.stamp,
            param_.field_of_view_radian.at(param_.order.at(2)), param_.min_range_m.at(param_.order.at(2)),
            param_.max_range_m.at(param_.order.at(2)));
        ultra_sonic_radar_data_.setRange(
            param_.order.at(3), (dec2hex(msg->data.at(6))*100+dec2hex(msg->data.at(7)))/1000.0, msg->header.stamp,
            param_.field_of_view_radian.at(param_.order.at(3)), param_.min_range_m.at(param_.order.at(3)),
            param_.max_range_m.at(param_.order.at(3)));
    }else if (msg->id == 0x612) {
        ultra_sonic_radar_data_.setRange(
            param_.order.at(4), (dec2hex(msg->data.at(0))*100+dec2hex(msg->data.at(1)))/1000.0, msg->header.stamp,
            param_.field_of_view_radian.at(param_.order.at(4)), param_.min_range_m.at(param_.order.at(4)),
            param_.max_range_m.at(param_.order.at(4)));
        ultra_sonic_radar_data_.setRange(
            param_.order.at(5), (dec2hex(msg->data.at(2))*100+dec2hex(msg->data.at(3)))/1000.0, msg->header.stamp,
            param_.field_of_view_radian.at(param_.order.at(5)), param_.min_range_m.at(param_.order.at(5)),
            param_.max_range_m.at(param_.order.at(5)));
        ultra_sonic_radar_data_.setRange(
            param_.order.at(6), (dec2hex(msg->data.at(4))*100+dec2hex(msg->data.at(5)))/1000.0, msg->header.stamp,
            param_.field_of_view_radian.at(param_.order.at(6)), param_.min_range_m.at(param_.order.at(6)),
            param_.max_range_m.at(param_.order.at(6)));
        ultra_sonic_radar_data_.setRange(
            param_.order.at(7), (dec2hex(msg->data.at(6))*100+dec2hex(msg->data.at(7)))/1000.0, msg->header.stamp,
            param_.field_of_view_radian.at(param_.order.at(7)), param_.min_range_m.at(param_.order.at(7)),
            param_.max_range_m.at(param_.order.at(7)));
    }
}

void UltraSonicRadarDriver::timerCallback(const ros::TimerEvent &te)
{
    if((ros::Time::now()-current_stamped_).toSec()>TIME_OUT_SECOND||!is_radar_activated_)
    {
        ROS_ERROR_THROTTLE(5, "[ULTRA SONIC RADAR DRIVER] Please activate radar!");
        return;
    }
    publishData();
}

} // namespace ultra_sonic_radar_driver
