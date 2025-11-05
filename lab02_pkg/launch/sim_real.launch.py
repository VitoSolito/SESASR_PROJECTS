from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction



def generate_launch_description():

    


    controller_node = Node(
        package='lab02_pkg',
        executable='controller',
        name='controller',
        
       parameters= [PathJoinSubstitution([
            FindPackageShare('lab02_pkg'), 'config', 'controller_params.yaml'
        ])]
    )

    

    
    return LaunchDescription([
        
        controller_node
    ])