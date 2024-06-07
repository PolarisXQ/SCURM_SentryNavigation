from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='icp_relocalization',
            executable='transform_publisher',
            name='transform_publisher',
            output='screen'
        ),
        Node(
            package='icp_relocalization',
            executable='icp_node',
            name='icp_node',
            output='screen',
            parameters=[
                {'initial_x':0.0},
                {'initial_y':0.0},
                {'initial_z':0.1},
                {'initial_a':-2.2},
                # {'initial_roll':0.0},
                # {'initial_pitch':0.0},
                # {'initial_yaw':0.0},
                {'map_voxel_leaf_size':0.5},
                {'cloud_voxel_leaf_size':0.3},
                {'map_frame_id':'map'},
                {'solver_max_iter':75},
                {'max_correspondence_distance':0.1},
                {'RANSAC_outlier_rejection_threshold':1.0},
                {'map_path':'/home/sentry_ws/src/FAST_LIO_SAM/PCD/GlobalMap.pcd'},
                {'fitness_score_thre':0.1}, # 是最近点距离的平均值，越小越严格
                {'converged_count_thre':50},
                {'pcl_type':'livox'},
            ],
        )
    ])
