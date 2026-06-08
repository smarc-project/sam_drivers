from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
import ament_index_python.packages

def generate_launch_description():

    ld = LaunchDescription()    
    namespace = LaunchConfiguration('robot_name')
    # sam_package_dir = FindPackageShare(LaunchConfiguration('sam_description'))
    # sam_path = PathJoinSubstitution([sam_package_dir, LaunchConfiguration('sam_package_path')])    
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('sam_description'),
        'urdf')
    params = os.path.join(config_directory, 'sam_auv.urdf.xacro')
    robot_description_content = ParameterValue(Command(['xacro ', params]), value_type=str) 

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      namespace=namespace,
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }],
                                    #   remappings=[
                                    #         ('robot_description', 'sam/sam_description'),
                                    #     ]
                                      )    
    ld.add_action(robot_state_publisher_node)    
    
    return ld