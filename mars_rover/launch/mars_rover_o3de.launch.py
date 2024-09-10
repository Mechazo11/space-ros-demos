"""
Bring up nodes, hardware interfaces to control a Curiosity Rover in Open 3D Engine.

Author:
Azmyin Md. Kamal,
Ph.D. student in MIE,
Louisiana State University,
Louisiana, USA

Date: August 29th, 2024
Version: 1.0

AI: ChatGPT 4.o

"""

# Imports
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node, SetParameter
from launch.event_handlers import OnProcessExit
import os
from os import environ
import xacro

def generate_launch_description():
    """Experimental modification, only send twist messages."""
    # Node to take teleop
    teleop_rover_node = Node(
        package="mars_rover",
        executable="teleop_rover",
        output='screen'
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        #start_world,
        #robot_state_publisher,
        #spawn,
        # arm_node,
        # mast_node,
        # wheel_node,
        teleop_rover_node, # Renamed from run_node
        # odom_node,
        #ros_gz_bridge,
        #image_bridge,

        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=spawn,
        #         on_exit=[set_hardware_interface_active],
        #     )
        # ),
        # After the hardware interface is activated, 
        # the joint state broadcaster is loaded.
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=set_hardware_interface_active,
        #         on_exit=[load_joint_state_broadcaster],
        #     )
        # ),
        # After the joint state broadcaster is loaded, 
        # all trajectory controllers (arm, mast, wheel, steer, suspension) are loaded and activated.
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[
        #                 # load_arm_joint_traj_controller,
        #                 # load_mast_joint_traj_controller,
        #                 load_wheel_joint_traj_controller,
        #                 load_steer_joint_traj_controller,
        #                 load_suspension_joint_traj_controller],
        #     )
        # ),
    ])
