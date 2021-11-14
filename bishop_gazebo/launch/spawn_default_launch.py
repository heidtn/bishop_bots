import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from scripts import GazeboRosPaths
import xacro
import tempfile
import numpy as np


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    station_urdf_file_name = 'models/station.urdf'
    bot_xacro_file_name = 'models/construction_bot.xacro'
    world_name = 'launch/default_world.world'

    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
    }


    print("urdf_file_name : {}".format(station_urdf_file_name))
    print("Plugin path: ", plugin_path)

    station_urdf = os.path.join(
        get_package_share_directory('bishop_gazebo'),
        station_urdf_file_name)

    bot_xacro = os.path.join(
        get_package_share_directory('bishop_gazebo'),
        bot_xacro_file_name)

    bot_urdf_doc = xacro.parse(open(bot_xacro))

    bot_nodes = []
    bot_start_positions = []
    NUM_BOTS = 10
    WIDTH = 5
    SPACING = 5.0
    for i in range(NUM_BOTS):
        bot_urdf_doc = xacro.process_file(bot_xacro, mappings={"namespace": f"bot_{i}"})
        bot_urdf = bot_urdf_doc.toxml()
        xacro_file_object = tempfile.NamedTemporaryFile(delete=False)
        xacro_file_object.write(bot_urdf.encode("ascii"))
        bot_urdf = xacro_file_object.name

        x = str((i % WIDTH)*SPACING)
        y = str((i // WIDTH)*SPACING)
        
        bot_nodes.append(Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           name='urdf_spawner',
           output='screen',
           arguments=["-entity", f"bot_{i}", "-file", bot_urdf, 
                      '-x', x, '-y', y, '-z', '15.0']))

        bot_nodes.append(Node(
            package='bot_controller',
            executable='basic_controller',
            name='basic_controller',
            output='screen',
            namespace=f"/bot_{i}/"
            #remappings=[
            #   ("odom", f"bot_{i}/odom"),
            #   ("gazebo_ros_force", f"bot_{i}/gazebo_ros_force"),
            #   ("follow_path", f"bot_{i}/follow_path") 
            #]
        ))

    
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


        Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           name='urdf_spawner',
           output='screen',
           arguments=["-entity", "station", '-file', station_urdf]),
        
        *bot_nodes
    ])
