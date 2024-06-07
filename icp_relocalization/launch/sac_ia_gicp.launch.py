import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('sac_ia_gicp')

    # Define node
    sac_ia_gicp_node = Node(
        package='sac_ia_gicp',
        executable='sac_ia_gicp',
        name='sac_ia_gicp_node',
        parameters=[
            {'target_pcd_file': "/home/sentry_ws/src/FAST_LIO_SAM/PCD/GlobalMap.pcd"},
            {'num_threads': 8},
            {'k_serach_source': 100},
            {'k_serach_target': 100},
            # Please tune these param with a clear understanding of the algorithm
            # A good reference would be the official PCL github repo
            {'voxel_grid_leaf_size_source': 0.3},
            {'voxel_grid_leaf_size_target': 0.3}, 
            {'sac_ia_min_sample_distance': 0.1}, # the minimum distance between samples
            {'sac_ia_correspondence_randomness': 50}, # the number of neighbors to use when selecting a random feature correspondence.
            {'sac_ia_num_samples':5}, # nr_samples the number of samples to use during each iteration
            {'icp_max_correspondence_distance': 1.0},
            {'icp_max_iteration': 10000},
            {'icp_transformation_epsilon': 0.01}, # the transformation epsilon (maximum allowable translation squared difference)
            {'icp_euclidean_fitness_epsilon': 0.01}, # useless
            {'fitness_score_thre':0.1},
            {'mode':0}, # 0 - keep running; 1 - run when fast-lio need recovery
            {'max_optimize_times':3}
        ],
        remappings=[
            ('/source_cloud', '/cloud_registered_body'),
        ],
    )

    # Create launch description
    ld = LaunchDescription()

    # Add actions to launch description
    ld.add_action(sac_ia_gicp_node)

    return ld