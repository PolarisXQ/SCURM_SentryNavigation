# cmd_chassis

convert cmd_vel to chassis cmd

## file descriptions

- `fake_joint.py` since the chassis's angle is unavailable, we use a fake joint to simulate the rotation of the chassis as the navigator required
- `twist_transformer` transform the twist in chassis_link to yaw_link
- `twist2chassis_cmd.py` listen to cmd_vel(twist) and chassis_type(int8) and publish corresponding chassis_cmd. Chassis_type is defined in rm_hardware_driver/rm_interfaces/msg/ChassisCmd.msg: 1 for velocity control in yaw frame, 4 for spin at an angular velocity automatically(maybe a little bit different from the msg file)

## 导航电控通信测试

### 遥控器控制

run teleop node

```bash
# Terminal1
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```bash
# Terminal2
. install/setup.bash
ros2 launch cmd_chassis cmd2chassis.launch.py
```

or 

```bash
# Terminal1
ros2 run mouse_teleop mouse_teleop
```

```bash
# Terminal2
. install/setup.bash
ros2 launch cmd_chassis cmd2chassis.launch.py twist_topic:=/mouse_vel
```

also you can try other teleop nodes, just remember remap the topic to /cmd_vel

```bash
ros2 run joy_teleop joy_teleop
ros2 run key_teleop key_teleop
ros2 run teleop_twist_joy teleop_node
```

```bash
# Terminal3
. install/setup.bash
ros2 launch rm_base rm_base.launch.py
```

### 里程计，速度标定

```bash
# Terminal1
ros2 launch cmd_chassis odom.launch.py
```

```bash
# Terminal2
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py params_file:='/home/sentry_ws/src/sentry_bringup/params/odom_calib.yaml'
```

config the behavior tree (speed, time and distance etc.) at `sentry_bringup/params/odom_calib.yaml`