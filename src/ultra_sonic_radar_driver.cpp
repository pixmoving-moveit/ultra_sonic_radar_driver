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
void UltraSonicRadarData::setRangeMsg(
  const size_t &id,
  const float &field_of_view_radian, 
  const float &min_range_m, 
  const float &max_range_m)
{
  Range range_msg;
  range_msg.header.frame_id = "ultrasonic_" + std::to_string(id);
  range_msg.field_of_view = field_of_view_radian;
  range_msg.max_range = max_range_m;
  range_msg.min_range = min_range_m;
  range_msg.radiation_type = Range::ULTRASOUND;
  
  sensor_data_.at(id) = range_msg;
}

void UltraSonicRadarData::setRangeMsg(const size_t &id, const float &range, const rclcpp::Time stamp)
{
  sensor_data_.at(id).range = range;
  sensor_data_.at(id).header.stamp = stamp;
}

UltraSonicRadarDriver::UltraSonicRadarDriver(std::string name):Node(name)
{
  // ROS2 get parameters
  param_.ultrasonic_number = this->declare_parameter(
    "ultrasonic_number", 8);
  
  param_.activate_list = declare_parameter(
    "activate_list", std::vector<int>{0xb7, 0x1f});
  configure_radar_.distance_measurement_mode = param_.activate_list[0];
  
  param_.order = declare_parameter(
    "order", std::vector<int>{
    0, 1, 2, 3, 4, 5, 6, 7});

  param_.field_of_view_radian = declare_parameter(
    "field_of_view_radian", std::vector<double>{
    0.52, 0.52, 0.52, 0.52, 0.52, 0.52, 0.52, 0.52});

  param_.min_range_m = declare_parameter(
    "min_range_m", std::vector<double>{
    0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2});

  param_.max_range_m = declare_parameter(
    "max_range_m", std::vector<double>{
    2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
  
  // ros2 publisher
  can_frame_pub_ = this->create_publisher<can_msgs::msg::Frame>(
    "output/can_frame", 1);

  for(int i=0; i<param_.ultrasonic_number; i++){
    rclcpp::Publisher<Range>::SharedPtr ultra_sonic_range_pub;
    ultra_sonic_range_pub = this->create_publisher<Range>("output/ultra_sonic_radar_"+std::to_string(i), rclcpp::SensorDataQoS());
    ultra_sonic_range_pub_vector_.push_back(ultra_sonic_range_pub);
    ultra_sonic_radar_data_.sensor_data_.push_back(Range());
  }

  // ros2 sublisher
  can_frame_sub_ = this->create_subscription<can_msgs::msg::Frame>(
    "input/can_frame", 1, std::bind(&UltraSonicRadarDriver::canFrameCallback, this, std::placeholders::_1)
  );

  configure_radar_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "input/configure_radar", 1, std::bind(&UltraSonicRadarDriver::configureRadarCallback, this, std::placeholders::_1));

  // ros2 Timer 
  timer_rate_ = 10; 
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000/timer_rate_), std::bind(&UltraSonicRadarDriver::timerCallback, this));
  
}

UltraSonicRadarDriver::~UltraSonicRadarDriver()
{
}

void UltraSonicRadarDriver::canFrameCallback(const can_msgs::msg::Frame::ConstSharedPtr &msg)
{
  if(msg->id==0x611  && param_.ultrasonic_number >= 4) {
    // 获取0x611 aa aa aa aa bb bb bb bb 
    for(size_t index = 0; index<4; index++){
       
      ultra_sonic_radar_data_.setRangeMsg(
      param_.order.at(index), 
      (
        dec2hex(msg->data.at(index*2))*100
        +
        dec2hex(msg->data.at(index*2+1))
      )/1000.0, 
      msg->header.stamp);

      ultra_sonic_range_pub_vector_.at(index)->publish(ultra_sonic_radar_data_.sensor_data_.at(index));
    }

 
  }else if (msg->id == 0x612 && param_.ultrasonic_number >= 8) {
    for(size_t index = 0; index<4; index++){
       
      ultra_sonic_radar_data_.setRangeMsg(
      param_.order.at(index+4), 
      (
        dec2hex(msg->data.at(index*2))*100  // 0xaabb 获取高字节aa
        +
        dec2hex(msg->data.at(index*2+1)) // 0xaabb 获取高字节bb
      )/1000.0, 
      msg->header.stamp);
      // 发布话题
      ultra_sonic_range_pub_vector_.at(index+4)->publish(ultra_sonic_radar_data_.sensor_data_.at(index+4));

    }
    
  }else if (msg->id == 0x613 && param_.ultrasonic_number >= 12) {
    for(size_t index = 0; index<4; index++){
       
      ultra_sonic_radar_data_.setRangeMsg(
      param_.order.at(index+8), 
      (
        dec2hex(msg->data.at(index*2))*100
        +
        dec2hex(msg->data.at(index*2+1))
      )/1000.0, 
      msg->header.stamp);

      ultra_sonic_range_pub_vector_.at(index+8)->publish(ultra_sonic_radar_data_.sensor_data_.at(index+8));
    }
  }
  
}

void UltraSonicRadarDriver::timerCallback()
{
  // 奇数指令,且让超声波开始工作，循环发送配置到超声波
  if (configure_radar_.distance_measurement_mode % 2 == 0 
      && configure_radar_.work_mode[0] == 0x1f
      && configure_radar_.work_mode[1] == 0xff){
    // 偶数 发一次返回一次
    can_frame_pub_->publish(configure_info_);
  }
  
}

void UltraSonicRadarDriver::configureRadarCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr & msg)
{
  RCLCPP_INFO(this->get_logger(), "开始配置超声波");

  if(msg->data.size()==3 
    && 10.0<=msg->data[0] 
    && msg->data[0]<20.0

    && 0xb1<=msg->data[1] 
    && msg->data[1]<=0xba

    && 0.0 <= msg->data[2]
    && msg->data[2]<=1.0){
    
    // 存储传入的超声波设置信息
    configure_radar_.rate = static_cast<uint8_t>(msg->data[0]);
    configure_radar_.distance_measurement_mode = static_cast<uint8_t>(msg->data[1]);
    if(static_cast<bool>(msg->data[2])){
      configure_radar_.setMode1F();
    }else{
      configure_radar_.setMode10();
    }

    configure_radar_.show();
    RCLCPP_INFO(this->get_logger(), "超声波配置成功");
    
    // 发送配置到超声波
    configure_info_.header.stamp = this->now();
    configure_info_.dlc = 3;
    configure_info_.id = 0x601;
    configure_info_.data[0] = configure_radar_.distance_measurement_mode;
    configure_info_.data[1] = configure_radar_.work_mode[0];
    configure_info_.data[2] = configure_radar_.work_mode[1];
    can_frame_pub_->publish(configure_info_);

    // 超声波的频率参数发生变化时，重设定时器 
    if(configure_radar_.rate != timer_rate_){
      timer_->cancel();   // 取消定时器，不在执行回调函数
      if(!timer_->is_canceled()){
        RCLCPP_ERROR(this->get_logger(), "超声波配置失败 - 定时器频率重置失败, 定时器没有停止");
        return ;
      }

      timer_rate_ = configure_radar_.rate; 
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000/timer_rate_), 
        std::bind(&UltraSonicRadarDriver::timerCallback, 
        this));
    }
  }else{

    std::stringstream ss;
    for (auto byte : msg->data) {
      ss 
        << std::hex 
        << std::setfill('0') 
        << std::setw(2) 
        << static_cast<int>(byte) 
        << " ";
    }
    RCLCPP_ERROR_STREAM(this->get_logger(), "超声波配置失败 - 配置信息不合格: " << ss.str());
    return ;
  }

}

}