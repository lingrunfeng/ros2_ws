#!/usr/bin/env python3
"""
Unified launch file for the perception-based grasping solution.

This launch file orchestrates the entire stack:
1. Gazebo Simulation (Robot + World + Camera)
2. MoveIt 2 (Move Group + RViz)
3. Perception Server (Point Cloud Processing)
4. MTC Pick and Place Demo (Logic & Execution)

It replaces the need for multiple terminal commands or scripts.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
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
    pkg_share_mtc_pick_place = FindPackageShare(pkg_mtc_pick_place)

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. Gazebo Simulation
    # ros2 launch mycobot_gazebo mycobot.gazebo.launch.py world_file:=pick_and_place_demo.world use_camera:=true ...
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
            'use_camera': 'true',
            'use_rviz': 'false', # We launch RViz via MoveIt
            'use_robot_state_pub': 'true',
            'x': '0.0', 'y': '0.0', 'z': '0.0',
            'roll': '0.0', 'pitch': '0.0', 'yaw': '0.0'
        }.items()
    )

    # 2. MoveIt 2 (Move Group + RViz)
    # ros2 launch mycobot_moveit_config move_group.launch.py rviz_config_file:=mtc_demos.rviz ...
    # Note: robot.sh uses mtc_demos.rviz from mycobot_mtc_demos
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
            'use_rviz': 'true',  # 明确启动 RViz
            'rviz_config_file': 'mtc_demos.rviz',
            'rviz_config_package': pkg_mtc_demos
        }.items()
    )

    # 3. Perception Server
    # ros2 launch mycobot_mtc_pick_place_demo get_planning_scene_server.launch.py
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share_mtc_pick_place,
                'launch',
                'get_planning_scene_server.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. MTC Pick and Place Node
    # We define this manually to set 'execute': True
    def configure_mtc_node(context):
        robot_name_str = 'mycobot_280'
        
        # Re-resolve paths inside the function
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

        # 直接定义所有参数，不使用 YAML 文件
        mtc_params = {
            # General parameters
            'execute': True,  # 强制执行
            'max_solutions': 25,
            
            # Controller parameters
            'controller_names': ["arm_controller", "grip_action_controller"],
            
            # Robot configuration parameters
            'arm_group_name': "arm",
            'gripper_group_name': "gripper",
            'gripper_frame': "link6_flange",
            'gripper_open_pose': "open",
            'gripper_close_pose': "half_closed",
            'arm_home_pose': "home",
            
            # Scene frame parameters
            'world_frame': "base_link",
            
            # Object parameters
            'object_name': "object",
            'object_type': "cylinder",
            'object_reference_frame': "base_link",
            'object_dimensions': [0.35, 0.0125],
            'object_pose': [0.22, 0.12, 0.0, 0.0, 0.0, 0.0],
            
            # Grasp and place parameters
            'grasp_frame_transform': [0.0, 0.0, 0.096, 1.5708, 0.0, 0.0],
            'place_pose': [-0.183, -0.14, 0.0, 0.0, 0.0, 0.0],
            
            # Motion planning parameters
            'approach_object_min_dist': 0.0015,
            'approach_object_max_dist': 0.3,
            'lift_object_min_dist': 0.005,
            'lift_object_max_dist': 0.3,
            'lower_object_min_dist': 0.005,
            'lower_object_max_dist': 0.4,
            
            # Timeout parameters
            'move_to_pick_timeout': 10.0,
            'move_to_place_timeout': 10.0,
            
            # Grasp generation parameters
            'grasp_pose_angle_delta': 0.1309,
            'grasp_pose_max_ik_solutions': 10,
            'grasp_pose_min_solution_distance': 0.8,
            
            # Place generation parameters
            'place_pose_max_ik_solutions': 10,
            
            # Cartesian planner parameters
            'cartesian_max_velocity_scaling': 1.0,
            'cartesian_max_acceleration_scaling': 1.0,
            'cartesian_step_size': 0.00025,
            
            # Direction vector parameters
            'approach_object_direction_z': 1.0,
            'lift_object_direction_z': 1.0,
            'lower_object_direction_z': -1.0,
            'retreat_direction_z': -1.0,
            
            # Other parameters
            'place_pose_z_offset_factor': 0.5,
            'retreat_min_distance': 0.025,
            'retreat_max_distance': 0.25
        }

        # MTC Node
        mtc_node = Node(
            package=pkg_mtc_pick_place,
            executable="mtc_node",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time},
                {'start_state': {'content': initial_positions_file}},
                mtc_params  # 直接使用字典而不是 YAML 文件
            ],
        )
        return [mtc_node]

    mtc_node_setup = OpaqueFunction(function=configure_mtc_node)

    # Delays to ensure startup order
    # Gazebo takes time to start, so we delay MoveIt slightly
    moveit_delayed = TimerAction(
        period=10.0,
        actions=[moveit_launch]
    )

    # Perception needs MoveIt/Gazebo ready
    perception_delayed = TimerAction(
        period=15.0,
        actions=[perception_launch]
    )

    # MTC Node needs everything ready
    mtc_node_delayed = TimerAction(
        period=20.0,
        actions=[mtc_node_setup]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(gazebo_launch)
    ld.add_action(moveit_delayed)
    ld.add_action(perception_delayed)
    ld.add_action(mtc_node_delayed)

    return ld
