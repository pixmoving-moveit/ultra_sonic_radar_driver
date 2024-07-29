#!/bin/bash
# 发送超声波配置

# msg[0]: 超声波的发布频率
# msg[1]: 超声波测距模式
# msg[2]: 超声波探头是否工作 1 所有探头都工作 0 所有探头都不工作

rate=${2:-20}
distance_measurement_mode=${3:-0xb2}
work_mode=${4:-1}
ros2 topic pub -1 ${1:-/input/configure_radar} std_msgs/msg/Float32MultiArray "
{
layout:{
  dim: [],
  data_offset: 0
  },
  data: [
  $rate, 
  $((distance_measurement_mode)),
  $((work_mode))
  ]
}
"

# hex_number=0x1A3F
# echo $((hex_number))