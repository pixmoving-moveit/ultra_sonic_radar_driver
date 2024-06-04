# 介绍
- 8探头超声波 F40-16TR
- 需要激活CAN驱动（ros2_socket）
# 如何使用

```bash
# 1.0 编译
colcon build --symlink-install --packages-select ultra_sonic_radar_driver
# 2.0 运行
source install/setup.bash 
ros2 launch ultra_sonic_radar_driver mc_radar_driver.launch.xml 
# 3.0 激活
ros2 topic pub -1 /sensing/ultra_sonic_radar/activate_radar std_msgs/msg/Bool "{data: True}"
```
# 配置文件 - config.param.yaml
```yaml
order: [0, 1, 2, 3, 4, 5, 6, 7]                                 # order of detector 
field_of_view_radian: [0.52, 0.52, 0.52, 0.52, 0.52, 0.52, 0.52, 0.52] # radians
min_range_m: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]             # m
max_range_m: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]             # m
```
## order 字段
| Input                  | Data Type        | Explanation                            |
| ----------------------- | ---------------- | -----------                            |
| header | std_msgs/Header | header                   |
| 0号探头的数据 |`/sensing/ultra_sonic_radar/ultra_sonic_radar_0`<br>(`sensor_msgs::msg::Range`)|对应配置文件order字段下标0|
| 1号探头的数据 |`/sensing/ultra_sonic_radar/ultra_sonic_radar_1`<br>(`sensor_msgs::msg::Range`)|对应配置文件order字段下标1|
| 2号探头的数据 |`/sensing/ultra_sonic_radar/ultra_sonic_radar_2`<br>(`sensor_msgs::msg::Range`)|对应配置文件order字段下标2|
| 3号探头的数据 |`/sensing/ultra_sonic_radar/ultra_sonic_radar_3`<br>(`sensor_msgs::msg::Range`)|对应配置文件order字段下标3|
| 4号探头的数据 |`/sensing/ultra_sonic_radar/ultra_sonic_radar_4`<br>(`sensor_msgs::msg::Range`)|对应配置文件order字段下标4|
| 5号探头的数据 |`/sensing/ultra_sonic_radar/ultra_sonic_radar_5`<br>(`sensor_msgs::msg::Range`)|对应配置文件order字段下标5|
| 6号探头的数据 |`/sensing/ultra_sonic_radar/ultra_sonic_radar_6`<br>(`sensor_msgs::msg::Range`)|对应配置文件order字段下标6|
| 7号探头的数据 |`/sensing/ultra_sonic_radar/ultra_sonic_radar_7`<br>(`sensor_msgs::msg::Range`)|对应配置文件order字段下标7|

如果超声波安装位置没有正确，可以通过调整`config.param.yaml`order字段,让对应位置的超声波探头传出的对应的话题上
![](./docs/ultrasonic_installation_position.jpg)
### 例如：
 - 0号探头和1号探头位置安装错误-order: [1, 0, 2, 3, 4, 5, 6, 7] 
 - 这样1号探头的数据还是以`/sensing/ultra_sonic_radar/ultra_sonic_radar_0`发布出来
 - 这样做的目的，主要是保证
![](./docs/ultrasonic_installation_position_error.jpg)
注意点：至于探头的序号，根据超声波银色盒子标记(标记1对应0号探头)
## field_of_view_radian 字段
![](./docs/ultrasonic_FOV.jpg)
