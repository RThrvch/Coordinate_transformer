"""
@file test_full_transform.py
@brief Integration tests для CoordinateTransformer node, используя launch_testing.

@author Ruslan Mukhametsafin
@date   2025-04-10
"""

import os
import sys
import time
import unittest
import shlex

import launch
import launch_ros
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import geometry_msgs.msg
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'coordinate_transformer'

# --- Test Launch Description ---

def generate_test_description():
    # Define boundary parameters directly here for clarity and isolation
    target_map_bounds = {
        'boundaries.target_map.min_x': -50.0,
        'boundaries.target_map.min_y': -50.0,
        'boundaries.target_map.min_z': -5.0,
        'boundaries.target_map.max_x': 50.0,
        'boundaries.target_map.max_y': 50.0,
        'boundaries.target_map.max_z': 5.0
    }

    # --- Define the node under test ---
    test_transformer_node = launch_ros.actions.Node(
        package=PACKAGE_NAME,
        executable='test_transformer_node',
        name='test_transformer_node',
        output='screen',
        parameters=[
            target_map_bounds,
            {'boundary_frame_names': ['target_map']}
        ],
    )
    # ---------------------------------

    # --- Add Static Transform Publishers ---
    static_tf_pub_target = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_target_map_to_a',
        arguments=['0', '0', '0', '0', '0', '0', 'target_map', 'frame_a']
    )
    static_tf_pub_b = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_a_to_b',
        arguments=['10', '0', '0', '0', '0', '0', 'frame_a', 'frame_b']
    )
    # ------------------------------------

    ros2_cmd = 'ros2'

    # 1. Pose  для успешного преобразования (frame_a -> target_map)
    pose_success_yaml = '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: frame_a}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
    cmd_success_list = [
        ros2_cmd, 'topic', 'pub', '--once',
        '/test/input_pose', 'geometry_msgs/msg/PoseStamped',
        pose_success_yaml
    ]
    pub_success_pose = ExecuteProcess(
        cmd=cmd_success_list,
        output='screen',
    )

    # 2. Pose  для преобразования за пределами границ (frame_b -> target_map, x=100.0 -> result x=110.0 > 50.0)
    pose_bounds_yaml = '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: frame_b}, pose: {position: {x: 100.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
    cmd_bounds_list = [
        ros2_cmd, 'topic', 'pub', '--once',
        '/test/input_pose', 'geometry_msgs/msg/PoseStamped',
        pose_bounds_yaml
    ]
    pub_bounds_pose = ExecuteProcess(
        cmd=cmd_bounds_list,
        output='screen',
    )

    # 3. Pose для преобразования не найдено (unknown_frame -> target_map)
    pose_unknown_yaml = '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: unknown_frame}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
    cmd_unknown_list = [
        ros2_cmd, 'topic', 'pub', '--once',
        '/test/input_pose', 'geometry_msgs/msg/PoseStamped',
        pose_unknown_yaml
    ]
    pub_unknown_pose = ExecuteProcess(
        cmd=cmd_unknown_list,
        output='screen',
    )
    # -----------------------------------------------

    # Setup launch description
    ld = launch.LaunchDescription([
        static_tf_pub_target,
        static_tf_pub_b,
        test_transformer_node,

        TimerAction(period=3.0, actions=[
            launch.actions.LogInfo(msg='Publishing success pose...'),
            pub_success_pose
        ]),
        TimerAction(period=5.0, actions=[
             launch.actions.LogInfo(msg='Publishing bounds pose...'),
            pub_bounds_pose
        ]),
        TimerAction(period=7.0, actions=[
            launch.actions.LogInfo(msg='Publishing unknown pose...'),
            pub_unknown_pose
        ]),

        TimerAction(period=9.0, actions=[launch_testing.actions.ReadyToTest()]),
    ])

    test_context = {'test_transformer_node': test_transformer_node}

    return ld, test_context


class TestCoordinateTransformerActive(unittest.TestCase):


    def test_successful_conversion(self, proc_output):
        proc_output.assertWaitFor(
            "Conversion successful. Publishing output pose in frame 'target_map'",
            timeout=10,
            stream='stderr'
        )

    def test_out_of_bounds_conversion(self, proc_output):
        proc_output.assertWaitFor(
            "Conversion result for frame 'target_map' is out of bounds",
            timeout=10,
            stream='stderr'
        )

    def test_transform_not_found(self, proc_output):
        proc_output.assertWaitFor(
            "Transform not found from 'unknown_frame' to 'target_map'",
            timeout=15,
            stream='stderr'
        )


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        print("Checking exit codes...")
        launch_testing.asserts.assertExitCodes(proc_info)
