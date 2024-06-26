/**
 * @file ultra_sonic_radar_driver.cpp
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
#include <ultra_sonic_radar_driver/ultra_sonic_radar_driver.hpp>

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
  void UltraSonicRadarData::setRange(const size_t &id, const float &range, const rclcpp::Time stamp,
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

  UltraSonicRadarDriver::UltraSonicRadarDriver(std::string name):Node(name)
  {
      // ROS2 get parameters
      param_.ultrasonic_number = this->declare_parameter(
        "ultrasonic_number", 8);

      auto activate_list = declare_parameter(
        "activate_list", std::vector<int>{
            0xb7, 0x1f});
      std::vector<int> activate_list_v(activate_list.begin(), activate_list.end());
      param_.activate_list = activate_list_v;
      
      auto order = declare_parameter(
        "order", std::vector<int>{
            0, 1, 2, 3, 4, 5, 6, 7});
      std::vector<int> order_v(order.begin(), order.end());
      param_.order = order_v;

      auto field_of_view_radian = declare_parameter(
        "field_of_view_radian", std::vector<double>{
            0.52, 0.52, 0.52, 0.52, 0.52, 0.52, 0.52, 0.52});
      std::vector<double> field_of_view_radian_v(field_of_view_radian.begin(), field_of_view_radian.end());
      param_.field_of_view_radian = field_of_view_radian_v;

      auto min_range_m = declare_parameter(
        "min_range_m", std::vector<double>{
            0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
      std::vector<double> min_range_m_v(min_range_m.begin(), min_range_m.end());
      param_.min_range_m  = min_range_m_v;

      auto max_range_m = declare_parameter(
        "max_range_m", std::vector<double>{
            2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
      std::vector<double> max_range_m_v(max_range_m.begin(), max_range_m.end());
      param_.max_range_m = max_range_m_v;
      
      MC_INFO << "ultrasonic_number:" << (int)param_.ultrasonic_number <<  MC_END;
      
      // ros2 publisher
      can_frame_pub_ = this->create_publisher<can_msgs::msg::Frame>(
        "output/can_frame", 1);

      for(int i=0; i<param_.ultrasonic_number; i++){

        rclcpp::Publisher<Range>::SharedPtr ultra_sonic_range_pub;
        ultra_sonic_range_pub = this->create_publisher<Range>("output/ultra_sonic_radar_"+std::to_string(i), rclcpp::SensorDataQoS());
        ultra_sonic_range_pub_vector_.push_back(ultra_sonic_range_pub);
        ultra_sonic_radar_data_.sensor_data_ptr_.push_back(RangeSharedPtr());
      }

      // ros2 sublisher
      activate_radar_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "input/activate_radar", 1, std::bind(&UltraSonicRadarDriver::activateRadarCallback, this, std::placeholders::_1));

      can_frame_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "input/can_frame", 1, std::bind(&UltraSonicRadarDriver::canFrameCallback, this, std::placeholders::_1)
      );

      // int hz = 10; // 10hz
      int hz = 66; // 15hz
      // ros2 Timer 
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(hz), std::bind(&UltraSonicRadarDriver::timerCallback, this));

      // activate
      is_radar_activated_ = false;
      is_received_radar_data_ = false;

      // init time source
      current_stamped_ = this->now();
      prev_stamped_ = this->now();
  }

  UltraSonicRadarDriver::~UltraSonicRadarDriver()
  {
  }

  void UltraSonicRadarDriver::resetRadarData()
  {
      for (int i = 0; i < param_.ultrasonic_number;i++)
      {
          ultra_sonic_radar_data_.setRange(
              param_.order.at(i), 1000.0, this->now(),
              param_.field_of_view_radian.at(param_.order.at(i)), param_.min_range_m.at(param_.order.at(i)),
              param_.max_range_m.at(param_.order.at(i)));
      }
      publishData();

  }

  void UltraSonicRadarDriver::activateRadarCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
      if(msg->data)
      {
          activateRadar();
          RCLCPP_INFO(this->get_logger(), "ACTIVATING ULTRASONIC RADAR");
      }else{
          deactivateRadar();
          RCLCPP_INFO(this->get_logger(), "DEACTIVATING ULTRASONIC RADAR");
      }

  }
  void UltraSonicRadarDriver::activateRadar()
  {
      can_msgs::msg::Frame activate_message;
      activate_message.header.stamp = this->now();
      activate_message.dlc = 3;
      activate_message.id = 0x601;
      activate_message.data[0] = param_.activate_list[0];
      activate_message.data[1] = param_.activate_list[1];
      activate_message.data[2] = 0xff;
      can_frame_pub_->publish(activate_message);
      is_radar_activated_ = true;
      RCLCPP_INFO(this->get_logger(), "RADAR ACTIVATED");
  }
  void UltraSonicRadarDriver::deactivateRadar()
  {
      can_msgs::msg::Frame deactivate_message;
      deactivate_message.header.stamp = this->now();
      deactivate_message.dlc = 3;
      deactivate_message.id = 0x601;
      deactivate_message.data[0] = param_.activate_list[0];
      deactivate_message.data[1] = param_.activate_list[1];
      deactivate_message.data[2] = 0x00;
      can_frame_pub_->publish(deactivate_message);
      is_radar_activated_ = false;
      resetRadarData();
      RCLCPP_INFO(this->get_logger(), "RADAR DEACTIVATED");
  }
  
  void UltraSonicRadarDriver::publishData()
  {
      for (int i=0;i<param_.ultrasonic_number;i++)
      {   
          ultra_sonic_range_pub_vector_.at(i)->publish(*ultra_sonic_radar_data_.sensor_data_ptr_.at(i));
      }
  }

  

  void UltraSonicRadarDriver::canFrameCallback(const can_msgs::msg::Frame::ConstSharedPtr &msg)
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
    }else if (msg->id == 0x613 && param_.ultrasonic_number == 12) {
      ultra_sonic_radar_data_.setRange(
        param_.order.at(8), (dec2hex(msg->data.at(0))*100+dec2hex(msg->data.at(1)))/1000.0, msg->header.stamp,
        param_.field_of_view_radian.at(param_.order.at(8)), param_.min_range_m.at(param_.order.at(4)),
        param_.max_range_m.at(param_.order.at(8)));
      ultra_sonic_radar_data_.setRange(
        param_.order.at(9), (dec2hex(msg->data.at(2))*100+dec2hex(msg->data.at(3)))/1000.0, msg->header.stamp,
        param_.field_of_view_radian.at(param_.order.at(9)), param_.min_range_m.at(param_.order.at(5)),
        param_.max_range_m.at(param_.order.at(9)));
      ultra_sonic_radar_data_.setRange(
        param_.order.at(10), (dec2hex(msg->data.at(4))*100+dec2hex(msg->data.at(5)))/1000.0, msg->header.stamp,
        param_.field_of_view_radian.at(param_.order.at(10)), param_.min_range_m.at(param_.order.at(6)),
        param_.max_range_m.at(param_.order.at(10)));
      ultra_sonic_radar_data_.setRange(
        param_.order.at(11), (dec2hex(msg->data.at(6))*100+dec2hex(msg->data.at(7)))/1000.0, msg->header.stamp,
        param_.field_of_view_radian.at(param_.order.at(11)), param_.min_range_m.at(param_.order.at(7)),
        param_.max_range_m.at(param_.order.at(11)));
    }
   
  }

  void UltraSonicRadarDriver::timerCallback()
  {
      double timeout = (this->now()-current_stamped_).seconds();
      if(timeout > TIME_OUT_SECOND||!is_radar_activated_)
      {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000*5, "[ULTRA SONIC RADAR DRIVER] Please activate radar![%f]",timeout);
        return;
      }
      publishData();
  }
  
}
