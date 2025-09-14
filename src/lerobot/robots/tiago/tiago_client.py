#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file tiago_client.py
@brief Implements the client for the Tiago robot, compatible with the LeRobot framework.
@details This class acts as a proxy to the real robot. It sends actions and receives
         observations over a simple TCP socket connection. The cameras are expected to be
         connected to the client machine.
"""

import socket
import struct
import pickle
import logging
from functools import cached_property
from typing import Any

from lerobot.robots.robot import Robot
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.cameras.utils import make_cameras_from_configs
from .config_tiago import TiagoClientConfig

class TiagoClient(Robot):
    """
    A client for the Tiago robot that communicates over TCP sockets.
    This class adheres to the lerobot.robots.Robot interface.
    """

    config_class = TiagoClientConfig
    name = "tiago_client"

    def __init__(self, config: TiagoClientConfig):
        super().__init__(config)
        self.config = config
        self._is_socket_connected = False
        self.socket = None
        # Initialize cameras from the configuration
        self.cameras = make_cameras_from_configs(config.cameras)

    @cached_property
    def _robot_ft(self) -> dict[str, Any]:
        """Defines the structure of the observation dictionary received from the robot."""
        return {
            # Arm
            'arm_1_joint.pos': float,
            'arm_2_joint.pos': float,
            'arm_3_joint.pos': float,
            'arm_4_joint.pos': float,
            'arm_5_joint.pos': float,
            'arm_6_joint.pos': float,
            'arm_7_joint.pos': float,
            # Torso
            'torso_lift_joint.pos': float,
            # Gripper
            'gripper_left_finger_joint.pos': float,
            'gripper_right_finger_joint.pos': float,
            # Base
            'base_linear_x.vel': float,
            'base_angular_z.vel': float,
        }

    @cached_property
    def _cameras_ft(self) -> dict[str, tuple]:
        """Defines the camera observation features."""
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, Any]:
        """Defines the complete structure of the observation dictionary, including cameras."""
        return {**self._robot_ft, **self._cameras_ft}
    
    @cached_property
    def action_features(self) -> dict[str, Any]:
        """Defines the structure of the action dictionary."""
        return {
            'arm_joint_positions': list,
            'torso_lift_joint.pos': float,
            'base_linear_velocity': float,
            'base_angular_velocity': float,
            'gripper_command': int,
        }
    
    @property
    def is_connected(self) -> bool:
        """Indicates if the socket and all cameras are connected."""
        return self._is_socket_connected and all(cam.is_connected for cam in self.cameras.values())
    
    def connect(self):
        """Establishes a TCP socket connection and connects to local cameras."""
        if self._is_socket_connected:
            raise DeviceAlreadyConnectedError("This client is already connected.")
        
        # 1. Connect to the remote robot host
        try:
            logging.info(f"Connecting to Tiago host at {self.config.remote_ip}:{self.config.port}...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.config.connect_timeout_s)
            self.socket.connect((self.config.remote_ip, self.config.port))
            self.socket.settimeout(None)
            self._is_socket_connected = True
            logging.info("Successfully connected to Tiago host.")
        except Exception as e:
            self.socket = None
            raise DeviceNotConnectedError(f"Failed to connect to Tiago host: {e}")

        # 2. Connect to local cameras
        for cam_name, cam in self.cameras.items():
            logging.info(f"Connecting to camera: {cam_name}")
            cam.connect()
        
    def disconnect(self):
        """Closes the socket connection and disconnects from local cameras."""
        # 1. Disconnect cameras
        for cam in self.cameras.values():
            if cam.is_connected:
                cam.disconnect()

        # 2. Disconnect socket
        if not self._is_socket_connected:
            return
        
        if self.socket:
            self.socket.close()
            self.socket = None
        self._is_socket_connected = False
        logging.info("Disconnected from Tiago host.")

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Sends an action dictionary to the robot host."""
        if not self._is_socket_connected:
            raise DeviceNotConnectedError("Cannot send action. Client is not connected to the host.")
        
        try:
            self._send_msg(action)
        except Exception as e:
            self.disconnect()
            raise DeviceNotConnectedError(f"Connection to host lost while sending action: {e}")
        
        return action
    
    def get_observation(self) -> dict[str, Any]:
        """Receives robot state from the host and captures images from local cameras."""
        if not self.is_connected:
            raise DeviceNotConnectedError("Cannot get observation. Client is not fully connected.")

        try:
            # 1. Receive robot state from the remote host
            observation_data = self._recv_msg()
            if observation_data is None:
                raise DeviceNotConnectedError("Host closed the connection.")
            
            obs_dict = pickle.loads(observation_data)

            # 2. Capture images from local cameras and merge them into the observation
            for cam_key, cam in self.cameras.items():
                obs_dict[cam_key] = cam.read()

            return obs_dict
        
        except (socket.error, pickle.UnpicklingError) as e:
            self.disconnect()
            raise DeviceNotConnectedError(f"Connection to host lost while receiving observation: {e}")
        
    def _send_msg(self, msg):
        """Helper to serialize and send a message with a 4-byte length prefix."""
        payload = pickle.dumps(msg, protocol=2)
        header = struct.pack('>I', len(payload))
        self.socket.sendall(header + payload)

    def _recv_msg(self):
        """Helper to receive a message with a 4-byte length prefix."""
        raw_msglen = self.socket.recv(4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        
        data = bytearray()
        while len(data) < msglen:
            packet = self.socket.recv(msglen - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data
    
    def calibrate(self) -> None:
        """Sends a calibration command to the host and waits for confirmation."""
        if not self._is_socket_connected:
            raise DeviceNotConnectedError("Cannot calibrate. Client is not connected to the host.")

        try:
            logging.info("Sending calibration command to host...")
            self._send_msg('calibrate')
            response_data = self._recv_msg()
            if response_data is None:
                raise DeviceNotConnectedError("Host closed the connection during calibration.")
            response = pickle.loads(response_data)
            if response == 'calibration_complete':
                logging.info("Calibration successful. Robot is at home position.")
            else:
                logging.warning(f"Received unexpected response during calibration: {response}")
        except (socket.error, pickle.UnpicklingError) as e:
            self.disconnect()
            raise DeviceNotConnectedError(f"Connection to host lost during calibration: {e}")

    def configure(self) -> None:
        pass
    
    @property
    def is_calibrated(self) -> bool:
        return True
