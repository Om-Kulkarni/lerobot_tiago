#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file tiago.py
@brief Hardware driver class for the Tiago robot using ROS.
@details This class abstracts the ROS communication for controlling the arm, base, and gripper,
         and for gathering sensor observations. It is designed to be used by a network host script.
"""

import rospy # type: ignore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry  
import threading

class Tiago:
    """A class to control the Tiago robot via ROS topics."""

    def __init__(self):
        """Initializes ROS publishers and subscribers."""
        rospy.loginfo("Initializing Tiago Hardware Driver...")

        # Publishers
        self.torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        self.base_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self.gripper_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

        # Subscribers
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self._joint_state_callback)
        # Subscriber for base velocity from odometry data
        self.odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self._odom_callback)

        # Observation data storage and lock
        self._latest_observation = {}
        self._lock = threading.Lock()

        # Gripper configuration
        self.GRIPPER_OPEN_POS = [0.045, 0.045]
        self.GRIPPER_CLOSED_POS = [0.0, 0.0]

        rospy.sleep(1.0)  # Wait for publishers and subscribers to register
        rospy.loginfo("Tiago Hardware Driver Initialized.")

    def _joint_state_callback(self, msg):
        """Callback to update the latest joint states for arm, torso, and gripper."""
        # Process joint state information
        with self._lock:
            for i, name in enumerate(msg.name):
                # Includes gripper joints in the observation
                if 'arm' in name or 'torso' in name or 'gripper' in name:
                    # Storing with a '.pos' suffix for consistency
                    self._latest_observation[f"{name}.pos"] = msg.position[i]

    def _odom_callback(self, msg):
        """Callback to update the latest base velocities."""
        with self._lock:
            # Storing with a '.vel' suffix for consistency
            self._latest_observation['base_linear_x.vel'] = msg.twist.twist.linear.x
            self._latest_observation['base_angular_z.vel'] = msg.twist.twist.angular.z

    def get_observation(self):
        """
        Retrieves the latest observation data from the robot.
        
        Returns:
            dict: A dictionary containing the latest sensor data including joint positions,
                  gripper values, and base velocities.
        """
        with self._lock:
            # Return a copy to avoid thread safety issues
            return self._latest_observation.copy()
        
    def send_action(self, action):
        """
        Receives an action dictionary and commands the robot accordingly.

        Args:
            action (dict): A dictionary containing commands for different parts of the robot.
                           Example:
                           {
                               'torso_lift_joint.pos': 0.3,
                               'arm_joint_positions': [1.6, -0.9, ...],
                               'base_linear_velocity': 0.1,
                               'base_angular_velocity': -0.2,
                               'gripper_command': 1  # 1 for open, 2 for close
                           }
        """
        # Log the received action for debugging
        rospy.logdebug(f"Received action: {action}")

        if 'arm_joint_positions' in action and len(action['arm_joint_positions']) == 7:
            # The client is already sending a 7-DOF command, so no padding is needed.
            self._send_arm_command(action['arm_joint_positions'])

        if 'torso_lift_joint.pos' in action:
            self._send_torso_command(action['torso_lift_joint.pos'])

        # --- BASE ---
        lin_vel = action.get('base_linear_velocity', 0.0)
        ang_vel = action.get('base_angular_velocity', 0.0)
        self._send_base_command(lin_vel, ang_vel)

        # --- GRIPPER ---
        gripper_cmd = action.get('gripper_command')
        if gripper_cmd == 1:
            self._send_gripper_command(self.GRIPPER_OPEN_POS)
        elif gripper_cmd == 2:
            self._send_gripper_command(self.GRIPPER_CLOSED_POS)

        # Return the action that was actually executed (useful for recording)
        return action

    def _send_torso_command(self, z_height):
        """Sends a command to the torso."""
        traj = JointTrajectory()
        traj.joint_names = ['torso_lift_joint']
        point = JointTrajectoryPoint()
        point.positions = [z_height]
        point.time_from_start = rospy.Duration(0.5)
        traj.points = [point]
        self.torso_pub.publish(traj)

    def _send_arm_command(self, arm_positions):
        """Sends a command to the arm."""
        traj = JointTrajectory()
        traj.joint_names = [f'arm_{i}_joint' for i in range(1, 8)]
        point = JointTrajectoryPoint()
        point.positions = arm_positions
        point.time_from_start = rospy.Duration(0.5)
        traj.points = [point]
        self.arm_pub.publish(traj)

    def _send_base_command(self, lin_vel_x, ang_vel_z):
        """Sends a command to the mobile base."""
        twist_msg = Twist()
        twist_msg.linear.x = lin_vel_x
        twist_msg.angular.z = ang_vel_z
        self.base_pub.publish(twist_msg)

    def _send_gripper_command(self, positions):
        """Sends a command to the gripper."""
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(0.5)
        traj.points = [point]
        self.gripper_pub.publish(traj)

    def go_to_home_position(self):
        """
        Commands the robot to move to a predefined home position.
        This is used for calibration/homing.
        """
        rospy.loginfo("Moving to home position...")

        # Default start positions for torso and arm joints
        DEFAULT_TORSO_POS = 0.30  # meters
        DEFAULT_ARM_POSITIONS = [1.61, -0.93, -3.14, 1.83, -1.58, -0.62, -1.58]  # radians

        # Send commands to all parts
        self._send_torso_command(DEFAULT_TORSO_POS)
        self._send_arm_command(DEFAULT_ARM_POSITIONS)
        self._send_gripper_command(self.GRIPPER_OPEN_POS)  # Gripper opened at home pos
        self._send_base_command(0.0, 0.0)  # Ensure base is stationary

        # Wait for the robot to reach the position
        # This duration might need tuning depending on the robot's speed.
        rospy.sleep(5.0)
        rospy.loginfo("Robot is at home position.")

    def stop_robot(self):
        """Stops all robot movement safely."""
        rospy.loginfo("Stopping all robot movement.")
        self._send_base_command(0.0, 0.0)