## ROS节点话题信息

[toc]

### 节点名称

```
/YB_Car_Node
```



### 机器人发布话题

| 功能     | 话题      | 信息                      | Frame_id    | 频率  |
| -------- | --------- | ------------------------- | ----------- | ----- |
| 小车速度 | /odom_raw | nav_msgs/msg/Odometry     | odom_frame  | 11 Hz |
| IMU芯片  | /imu      | sensor_msgs/msg/Imu       | imu_frame   | 25 Hz |
| 激光雷达 | /scan     | sensor_msgs/msg/LaserScan | laser_frame | 12 Hz |
| 电池电压 | /battery  | std_msgs/msg/UInt16       |             | 1Hz  | 

备注：

1. 电池电压实际值为/battery话题的data值除以10.0，单位为V。例如data=82，实际电压为8.2V。



### 机器人订阅话题

| 功能       | 话题      | 信息                    |
| ---------- | --------- | ----------------------- |
| 控制小车   | /cmd_vel  | geometry_msgs/msg/Twist |
| 控制蜂鸣器 | /beep     | std_msgs/msg/UInt16     |
| 控制舵机S1 | /servo_s1 | std_msgs/msg/Int32      |
| 控制舵机S2 | /servo_s2 | std_msgs/msg/Int32      |

备注：

1. 控制蜂鸣的data数值大小[0, 10000]，当data=0时蜂鸣器关闭。当data=1时，蜂鸣器一直鸣笛。当data>=10时，蜂鸣器响data毫秒后自动关闭。

2. 控制舵机S1的data数值大小[-90, 90]。

3. 控制舵机S2的data数值大小[-90, 20]。



### 机器人测试控制

#### 控制机器人运动

向/cmd_vel话题发布数据，控制机器人小车以0.5m/s向前行走。

```
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

向/cmd_vel话题发布数据，控制机器人小车以1.5rad/s旋转。

```
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.5}}"
```

向/cmd_vel话题发布数据，控制机器人小车停止。

```
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```



#### 控制蜂鸣器

向/beep话题发布数据，控制蜂鸣器一直鸣笛。

```
ros2 topic pub --once /beep std_msgs/msg/UInt16 "data: 1"
```

向/beep话题发布数据，控制蜂鸣器关闭。

```
ros2 topic pub --once /beep std_msgs/msg/UInt16 "data: 0"
```

向/beep话题发布数据，控制蜂鸣器鸣笛300毫秒后自动关闭。

```
ros2 topic pub --once /beep std_msgs/msg/UInt16 "data: 300"
```



#### 控制PWM舵机

向/servo_s1话题发布数据，控制舵机S1转动到30度。

```
ros2 topic pub --once /servo_s1 std_msgs/msg/Int32 "data: 30"
```

向/servo_s2话题发布数据，控制舵机S2转动到-60度。

```
ros2 topic pub --once /servo_s2 std_msgs/msg/Int32 "data: -60"
```





