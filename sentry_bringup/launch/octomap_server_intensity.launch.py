import os.path

from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # send tf odom->map
      # ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom --qx 0.0 --qw 1.0
    odom_map_trans = Node(
        name="odom_map_trans",
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments='--frame-id map --child-frame-id odom --qx 0.0 --qw 1.0'.split(' '),
        output='screen'
        )
    
    # terrain analysis
    terrain_analysis_ext = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
        get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
        ),
    )
    
    # exchange intensity and height filed of terrain analysis
    exchange_filed=Node(
        package='terrain_analysis',
        executable='exchangeField',
        output='screen',
        remappings=[('/input_topic', '/terrain_map_ext'),
                        ('/output_topic', '/terrain_map_ext_exchanged')] 
    )
    
    # transform terrain_map_ext to sensor frame
    sensor_scan_generation = Node(
        package='sensor_scan_generation',
        executable='sensorScanGeneration',
        output='screen',
        remappings=[('/registered_scan', '/terrain_map_ext_exchanged'),
                        ('/sensor_scan', '/terrain_map_at_scan')]
    )
    
    # build octomap use terrain analysis result, that is, intensity field is used as height field
    octomap_server_node = Node(
    package='octomap_server',
        executable='octomap_server_node',
        output='screen',
        parameters=[{
            "frame_id": "map",#! MUST SET TO MAP
            "base_frame_id": "sensor_at_scan",#! THIE MUST BE SPECIFIED IF YOU WANT TO SAVE MAP
            # "use_height_map":False,
            # "colored_map":False,
            # "color_factor":0.8,
            # "point_cloud_min_x":-10.0,
            # "point_cloud_max_x":10.0,
            # "point_cloud_min_y":-10.0,
            # "point_cloud_max_y":10.0,
            "point_cloud_min_z":0.15,
            # "point_cloud_max_z":100.0,
            # "occupancy_min_z":1.0,
            # "occupancy_max_z":100.0,
            # "min_x_size":0.0,
            # "min_y_size":0.0,
            "filter_speckles":True,
            "filter_ground_plane":False,
            # "ground_filter.distance":0.5,
            # "ground_filter.angle":0.15,
            # "ground_filter.plane_distance": 0.07, 
            # "sensor_model.max_range": 5.0,
            # "sensor_model.hit": 0.7,
            # "sensor_model.miss": 0.4,
            # "sensor_model.min": 0.12,
            # "sensor_model.max": 0.97, 
            # "compress_map": True,
            # "incremental_2D_projection": True, 
            "resolution": 0.1,
            "latch": True,#! THIE MUST BE SPECIFIED IF YOU WANT TO SAVE MAP
            # "publish_free_space": False,
            # "octomap_path", ""
        }],
            remappings=[('/cloud_in', '/terrain_map_at_scan')]
    )
            
    ld = LaunchDescription()
    ld.add_action(odom_map_trans)
    ld.add_action(terrain_analysis_ext)
    ld.add_action(exchange_filed)
    ld.add_action(sensor_scan_generation)
    ld.add_action(octomap_server_node)

    return ld