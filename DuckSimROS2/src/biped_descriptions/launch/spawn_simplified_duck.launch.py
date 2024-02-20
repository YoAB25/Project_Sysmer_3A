#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


# At this stage this import isn't really usefull
# from moveit_configs_utils import MoveItConfigdBuilder

def generate_launch_description():
    # Define the gazebo node & launch the empty gazebo world - Gazebo empty world
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #     get_package_share_directory('gazebo_ros'),
    #     'launch', 'gazebo.launch.py')])      
    # )

    # Define the gazebo node & launch the empty gazebo world - Gazebo factory world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('biped_gazebo_worlds'),
        'launch', 'empty_world.launch.py')])      
    )

    # Define the package path for the descriptions
    biped_gazebo_pkg_path = get_package_share_directory('biped_gazebo_worlds')
    biped_descriptions_pkg_path = get_package_share_directory('biped_descriptions')

    # Load parameters from the cassie.yaml file
    duck_yaml = os.path.join(biped_gazebo_pkg_path, 'config', 'duck.yaml')



    # Define the package path
    package_path = os.path.join(get_package_share_directory('biped_gazebo_worlds'))

    # Define the XACRO file name and path 
    xacro_file = os.path.join(biped_descriptions_pkg_path, 'urdf', 'duck.xacro')

    # IDK maybe to make the hole thingy an xml urdf format I don't know
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description' : doc.toxml()}

    # Declare the robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Declare the robot state publisher node
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Spawn the entity inside the world
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'simplified_duck'],
        output='screen'
    )

    # Node to load controllers and transmissions
    # load_controller_transmission = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     name='ros2_control_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': False, 'yaml_file': duck_yaml}],
    # )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controller'],
        output='screen'
    )


    return LaunchDescription(
        [gazebo,
        node_robot_state_publisher,
        node_joint_state_publisher,
        # load_controller_transmission,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        

        ]
    )

