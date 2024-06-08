# SCURM火锅战队 24赛季哨兵导航

开源的初衷是想把自己的一些好的想法和大家一起分享，所以欢迎各位联系我和我讨论，欢迎PR，提issue🫠~

主要的创新：

1. 实现FAST-LIO2的重定位模式，不需要另外运行重定位算法。算力需求小，使得整个框架在NUC12上的资源占用在30%左右；精度高，定位误差小；内存占用不会随着时间增长。

2. 改进navigation2的故障恢复行为，卡住时机器人会向无碰撞的方向运动。

3. 实现底盘和坡面对齐(comming soon)

算法框架和思路详见技术报告

Docker镜像使用方法参阅[DevcontainterGuide](./DevcontainterGuide.md)

## 包说明

| Package Name | Description |
|--------------|-------------|
| ✅auto_aim_interfaces |  |
| ✅[autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment) | <span style="color:red">**MODIFIED**</span> development env for cmu-series planner |
| ✅[BehaviourTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) | <span style="color:red">**MODIFIED**</span> BehaviourTree lib |
| ✅cmd_chassis | - cmd_vel to chassis_cmd for communication and motion type of the chassis <br> - exexute rotation command in chassis_link since the true value is unavailable |
| ✅control_panel | a simple Qt GUI for simulating referee system |
| ✅FAST_LIO | fastlio mapping |
| ✅livox_ros_driver2 | Driver for livox lidar |
| ✅nav2_plugins <br> - behavior_ext_plugins <br> - costmap_intensity <br> - velocity_smoother_ext | self defined nav2 plugins <br> - an enhenced back_up action that move toward free space <br> - 2 costmap_2d layer that use intensity filed of pointcloud msg rather than height (use with terrain analysis in autonomous_exploration_development_environment) <br> - an enhenced velocity smoother that increase the speed on slope automatically (use with terrain analysis-pathNorm) |
| ✅rm_decision_cpp | sentry desicion module based on BehaviourTree.CPP, implement nav2pose, topics2blackboard, anti_autoaim, attack etc. behaviours. |

## LAUNCH

### MAPPING

- launch mapping node

```bash
ros2 launch sentry_bringup mapping.launch.py
```

- save map

```bash
# occupancy grid map
ros2 run nav2_map_server map_saver_cli -t /projected_map -f test_map --fmt png
# save pcd
ros2 service call /fast_lio_sam/save_map fast_lio_sam/srv/SaveMap
# or set the resolution by adding 
ros2 service call /fast_lio_sam/save_map fast_lio_sam/srv/SaveMap "{resolution: 0.0}"
```

- then terminate all nodes, pcd file will be saved in /PCD/scans.pcd
    
### MAP PROCESSING

process pcd file
- drag the pcd file to CloudCompare(globalmap or scans.pcd), select the pointcloud in the left panel, then tools->clean->SOR filter, set the parameters (25,1 is a baseline) and apply
- select the processed pointcloud from last step, then tools->segmentation->Label Connected Components, set the parameters and apply
- pick out the CC#0(ususally this one), then tools->Other->Remove duplicate points, keep 1 point per 0.01-0.1m to reduce the size of the pointcloud
- select the processed pointcloud, then file->save as, select .pcd format

### LAUNCH ALL

```bash
ros2 launch sentry_bringup bringup_all_in_one.launch.py
```