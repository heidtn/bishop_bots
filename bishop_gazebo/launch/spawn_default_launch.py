import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from scripts import GazeboRosPaths



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'models/station.sdf'
    world_name = 'launch/default_world.world'

    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    env = {
        "GAZEBO_MODEL_PATH": model_path,
    }


    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('bishop_gazebo'),
        urdf_file_name)
    
    world = os.path.join(
        get_package_share_directory('bishop_gazebo'),
        world_name 
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world],
            output='screen',
            additional_env=env,
            ),

        #Node(
        #   package='robot_state_publisher',
        #   executable='robot_state_publisher',
        #   name='robot_state_publisher',
        #   output='screen',
        #   parameters=[{'use_sim_time': use_sim_time}],
        #   arguments=[urdf]),

        # Node(
        #    package='joint_state_publisher',
        #    executable='joint_state_publisher',
        #    name='joint_state_publisher',
        #    output='screen',
        #    parameters=[{'use_sim_time': use_sim_time}]
        #    ),

        Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           name='urdf_spawner',
           output='screen',
           arguments=["-entity", "station", "-file", urdf])
    ])
