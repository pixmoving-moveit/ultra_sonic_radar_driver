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
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "can_msgs/msg/frame.hpp"

#include "mc_log.hpp"

#define NUMBER_OF_DETECTORS 12

namespace ultra_sonic_radar_driver
{
typedef sensor_msgs::msg::Range Range;

/**
 * @brief convert hexadecimal data from Chen Mu radar to decimal type
 *
 * @param dec input data like 0x50 means 50 in decimal type
 * @return int data in decimal type
 */
int dec2hex(uint8_t dec);

struct Param
{
  uint8_t ultrasonic_number;
  std::vector<int64_t> activate_list;
  std::vector<int64_t> order;
  std::vector<double> field_of_view_radian;
  std::vector<double> min_range_m;
  std::vector<double> max_range_m;
};

struct UltrasonicConfigureParam
{
  uint8_t rate ;    // 超声波发布频率
  uint8_t distance_measurement_mode ;   // 超声波工作模式 - 探测距离设置
  uint8_t work_mode[2] = {0x10, 0x00};       // 超声波工作模式 - 探头是否工作 1f ff

  inline void setMode10() {
    work_mode[0] = 0x10;
    work_mode[1] = 0x00;
    RCLCPP_INFO(rclcpp::get_logger("UltrasonicConfigureParam"), "输入-停用超声波配置");
  }

  inline void setMode1F() {
    work_mode[0] = 0x1f;
    work_mode[1] = 0xff;
    RCLCPP_INFO(rclcpp::get_logger("UltrasonicConfigureParam"), "输入-激活超声波配置");
  }

  inline void show() const {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("UltrasonicConfigureParam"), 
      "超声波配置信息: "
      << "\nrate: " 
      << (int)rate
      << "\ndistance_measurement_mode: "
      << std::hex << (int)distance_measurement_mode
      << "\nultrasonic_mode: {0x"
      << std::hex << (int)work_mode[0]
      << ", 0x" << std::hex << (int)work_mode[1]
      << "}\n"
    );
  }
};

struct UltraSonicRadarData
{
  std::vector<Range> sensor_data_;
  UltraSonicRadarData(size_t number_of_detector)
  {
    sensor_data_.reserve(number_of_detector);
  }
  /**
   * @brief sensor_data_元素的初始化
   * 
   * @param id index of range
   * @param field_of_view_radian field of view(radian) needs to be set
   * @param min_range_m min range(m) needs to be set
   * @param max_range_m max range(m) needs to be set
   */
  void setRangeMsg(
    const size_t &id,
    const float &field_of_view_radian, 
    const float &min_range_m,
    const float &max_range_m);

  /// @brief 设置sensor_data_元素的时间戳和传感器距离
  void setRangeMsg(const size_t &id, const float &range, const rclcpp::Time stamp);

};

class UltraSonicRadarDriver:public rclcpp::Node
{
private:
  Param param_;
  UltraSonicRadarData ultra_sonic_radar_data_ = UltraSonicRadarData(NUMBER_OF_DETECTORS);

public:
  UltraSonicRadarDriver(std::string name);
  ~UltraSonicRadarDriver();

protected:

  // publihser

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_pub_;                  // 超声波 配置信息 发布者
  std::vector<rclcpp::Publisher<Range>::SharedPtr> ultra_sonic_range_pub_vector_;     // 单个超声波雷达 数据信息 发布者

  // sublihser
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;     // can 信息
  /// @brief 从can卡驱动程序处理canbus消息的回调函数，
  void canFrameCallback(const can_msgs::msg::Frame::ConstSharedPtr & msg);

  /**
   *  msg[0]: 超声波的发布频率
   *  msg[1]: 超声波测距模式
   *  msg[2]: 超声波探头是否工作 1.0 所有探头都工作 0.0 所有探头都不工作 
   */ 
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr configure_radar_sub_; // 超声波激活订阅者
  /// @brief 处理激活/停用消息的回调函数
  void configureRadarCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr & msg);
  UltrasonicConfigureParam configure_radar_;
  can_msgs::msg::Frame configure_info_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  /// @brief 定时器回调函数
  void timerCallback();
  uint8_t timer_rate_;  // 定时器频率，单位hz

};
} // namespace ultra_sonic_radar_driver
#endif