#!/usr/bin/env python3
"""
Launch file for the coordinate-based grasping solution.

This launch file orchestrates:
1. Gazebo Simulation (Robot + World)
2. MoveIt 2 (Move Group + RViz)
3. MTC Grasp Pose Node (New node that listens to /object_pose)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import OpaqueFunction

def generate_launch_description():
    # Package Names
    pkg_mycobot_gazebo = 'mycobot_gazebo'
    pkg_mycobot_moveit_config = 'mycobot_moveit_config'
    pkg_mtc_pick_place = 'mycobot_mtc_pick_place_demo'
    pkg_mtc_demos = 'mycobot_mtc_demos'

    # Find Package Shares
    pkg_share_mycobot_gazebo = FindPackageShare(pkg_mycobot_gazebo)
    pkg_share_mycobot_moveit_config = FindPackageShare(pkg_mycobot_moveit_config)
    
    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. Gazebo Simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share_mycobot_gazebo,
                'launch',
                'mycobot.gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_file': 'pick_and_place_demo.world',
            'use_camera': 'false', # Camera not strictly needed for coordinate grasping
            'use_rviz': 'false',
            'use_robot_state_pub': 'true',
            'x': '0.0', 'y': '0.0', 'z': '0.0',
            'roll': '0.0', 'pitch': '0.0', 'yaw': '0.0'
        }.items()
    )

    # 2. MoveIt 2 (Move Group + RViz)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share_mycobot_moveit_config,
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'true',
            'rviz_config_file': 'mtc_demos.rviz',
            'rviz_config_package': pkg_mtc_demos
        }.items()
    )

    # 3. MTC Grasp Pose Node
    def configure_mtc_node(context):
        robot_name_str = 'mycobot_280'
        pkg_share_moveit_config_path = FindPackageShare(pkg_mycobot_moveit_config).find(pkg_mycobot_moveit_config)
        config_path = os.path.join(pkg_share_moveit_config_path, 'config', robot_name_str)

        # Paths
        initial_positions_file = os.path.join(config_path, 'initial_positions.yaml')
        joint_limits_file = os.path.join(config_path, 'joint_limits.yaml')
        kinematics_file = os.path.join(config_path, 'kinematics.yaml')
        moveit_controllers_file = os.path.join(config_path, 'moveit_controllers.yaml')
        srdf_model = os.path.join(config_path, f'{robot_name_str}.srdf')
        pilz_limits = os.path.join(config_path, 'pilz_cartesian_limits.yaml')

        # MoveIt Config
        moveit_config = (
            MoveItConfigsBuilder(robot_name_str, package_name=pkg_mycobot_moveit_config)
            .trajectory_execution(file_path=moveit_controllers_file)
            .robot_description_semantic(file_path=srdf_model)
            .joint_limits(file_path=joint_limits_file)
            .robot_description_kinematics(file_path=kinematics_file)
            .planning_pipelines(
                pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
                default_planning_pipeline="ompl"
            )
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .pilz_cartesian_limits(file_path=pilz_limits)
            .to_moveit_configs()
        )

        mtc_params = {
            'execute': True,
            'object_type': "cylinder",
            'object_dimensions': [0.1, 0.0125], # Height, Radius
            
            # Robot configuration
            'arm_group_name': "arm",
            'gripper_group_name': "gripper",
            'gripper_frame': "link6_flange",
            'gripper_open_pose': "open",
            'gripper_close_pose': "half_closed",
            'arm_home_pose': "home",
            'world_frame': "base_link",
            
            # Motion parameters
            'approach_object_min_dist': 0.0015,
            'approach_object_max_dist': 0.3,
            'lift_object_min_dist': 0.005,
            'lift_object_max_dist': 0.3,
            'grasp_frame_transform': [0.0, 0.0, 0.096, 1.5708, 0.0, 0.0],
        }

        mtc_node = Node(
            package=pkg_mtc_pick_place,
            executable="mtc_grasp_pose_node",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time},
                mtc_params
            ],
        )
        return [mtc_node]

    mtc_node_setup = OpaqueFunction(function=configure_mtc_node)

    # Delays
    moveit_delayed = TimerAction(
        period=5.0,
        actions=[moveit_launch]
    )

    mtc_node_delayed = TimerAction(
        period=10.0,
        actions=[mtc_node_setup]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(gazebo_launch)
    ld.add_action(moveit_delayed)
    ld.add_action(mtc_node_delayed)

    return ld
