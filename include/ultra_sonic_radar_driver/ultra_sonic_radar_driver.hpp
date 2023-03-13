/**
 * @file ultra_sonic_radar_driver.hpp
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
#ifndef __ULTRA_SONIC_RADAR_DRIVER__HPP__
#define __ULTRA_SONIC_RADAR_DRIVER__HPP__

#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "can_msgs/msg/frame.hpp"

#define NUMBER_OF_DETECTORS 8
#define TIME_OUT_SECOND 1

namespace ultra_sonic_radar_driver
{
typedef sensor_msgs::msg::Range Range;
typedef std::shared_ptr<sensor_msgs::msg::Range> RangeSharedPtr;

/**
 * @brief convert hexadecimal data from Chen Mu radar to decimal type
 *
 * @param dec input data like 0x50 means 50 in decimal type
 * @return int data in decimal type
 */
int dec2hex(uint8_t dec);

struct Param
{
    std::vector<int> order;
    std::vector<double> field_of_view_radian;
    std::vector<double> min_range_m;
    std::vector<double> max_range_m;
};

struct UltraSonicRadarData
{
    std::vector<RangeSharedPtr> sensor_data_ptr_;
    UltraSonicRadarData(size_t number_of_detector)
    {
        sensor_data_ptr_.reserve(number_of_detector);
    }
    /**
     * @brief Set the Range object
     * 
     * @param id index of range
     * @param range range needs to be set
     * @param field_of_view_radian field of view(radian) needs to be set
     * @param min_range_m min range(m) needs to be set
     * @param max_range_m max range(m) needs to be set
     */
    void setRange(const size_t &id, const float &range, const rclcpp::Time stamp,
                  const float &field_of_view_radian, const float &min_range_m,
                  const float &max_range_m);
};

class UltraSonicRadarDriver:public rclcpp::Node
{
private:
    Param param_;
    UltraSonicRadarData ultra_sonic_radar_data_ = UltraSonicRadarData(NUMBER_OF_DETECTORS);

    // sign
    bool is_radar_activated_;
    bool is_received_radar_data_;

    rclcpp::Time current_stamped_;
    rclcpp::Time prev_stamped_;

public:
    UltraSonicRadarDriver(std::string name);
    ~UltraSonicRadarDriver();
    /**
     * @brief reset ultrasonic radar data
     * 
     */
    void resetRadarData();
    /**
     * @brief activate ultrasonic radar by sending activate message
     * 
     */
    void activateRadar();
    /**
     * @brief deactivate ultrasonic radar by sending deactivate canbus message
     * 
     */
    void deactivateRadar();
    /**
     * @brief publish radar range msg to topic
     * 
     */
    void publishData();
    /**
     * @brief callback function to process activate/deactivate message
     *
     * @param msg
     */
    void activateRadarCallback(const std_msgs::msg::Bool::ConstSharedPtr & msg);
    /**
     * @brief callback function to process canbus messages from can card driver,
     * in order to convert can frames to range msgs
     * 
     * @param msg 
     */
    void canFrameCallback(const can_msgs::msg::Frame::ConstSharedPtr & msg);

    /**
     * @brief timer callback
     * 
     * @param te 
     */
    void timerCallback();

    protected:
    // publihser
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_pub_;                  // 超声波 配置信息 发布者
    std::vector<rclcpp::Publisher<Range>::SharedPtr> ultra_sonic_range_pub_vector_;     // 单个超声波雷达 数据信息 发布者
    // sublihser
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activate_radar_sub_; // 超声波激活订阅者
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;     // can 信息

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;


};
} // namespace ultra_sonic_radar_driver
#endif