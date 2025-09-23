# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Simple script to directly control a simulated Tiago robot arm using a
teleoperator (e.g., SO101 leader).
"""

import logging
import time
from dataclasses import asdict, dataclass, field
from pprint import pformat

import draccus
# import pybullet as p
# import pybullet_data

from lerobot.robots import (
    Robot,
    RobotConfig,
    make_robot_from_config,
)

from lerobot.robots.tiago.config_tiago import TiagoClientConfig
from lerobot.robots.tiago.tiago_client import TiagoClient



from lerobot.teleoperators import (
    Teleoperator,
    TeleoperatorConfig,
    make_teleoperator_from_config,
    so101_leader,
)

from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging, move_cursor_up

@dataclass
class TiagoTeleopConfig:
    # Teleoperator device configuration
    teleop: TeleoperatorConfig
    # Configuration for the Tiago real robot client
    tiago_client: TiagoClientConfig = field(default_factory=TiagoClientConfig)
    # Limit the maximum frames per second
    fps: int = 60
    # Run for a fixed duration in seconds (optional)
    teleop_time_s: float | None = None

def get_tiago_joint_limits():
    """Returns a dictionary of hard-coded joint limits for the Tiago robot."""
    # Extracted from the provided image and robot documentation
    return {
        "arm_1_joint": (0.0, 2.7),
        "arm_2_joint": (-1.6, 1.1),
        "arm_3_joint": (-3.5, 1.6),
        "arm_4_joint": (-0.39, 2.4),
        "arm_5_joint": (-2.1, 2.1),
        "arm_6_joint": (-1.4, 1.4),
        "arm_7_joint": (-2.1, 2.1),
        "torso_lift_joint": (0.0, 0.35),
        "gripper_right_finger_joint": (0.0, 0.045),
        "gripper_left_finger_joint": (0.0, 0.045),
    }


def map_leader_to_tiago(action: dict, joint_limits: dict):
    """Maps SO101 leader actions to Tiago joint values (in radians/meters)."""
    mapped_action = {}
    
    # Mapping from teleop device to robot joints
    mappings = {
        "shoulder_pan.pos": "arm_1_joint",
        "shoulder_lift.pos": "arm_2_joint",
        "elbow_flex.pos": "arm_4_joint",
        "wrist_flex.pos": "arm_6_joint",
        "wrist_roll.pos": "arm_7_joint",
        "gripper.pos": "gripper_right_finger_joint",
    }
    
    # Add the string name of any joint that moves in the opposite direction.
    reversed_joints = [
        #"arm_1_joint",  
        #"arm_2_joint",  
        #"arm_4_joint",
    ]
    
    for leader_joint, tiago_joint in mappings.items():
        if leader_joint in action and tiago_joint in joint_limits:
            min_limit, max_limit = joint_limits[tiago_joint]
            
            # Normalizes the [-100, 100] input to a [0, 1] scale
            normalized_value = (action[leader_joint] + 100.0) / 200.0
            
            # If the joint is in list, flip the normalized value (e.g., 0.2 -> 0.8)
            if tiago_joint in reversed_joints:
                normalized_value = 1.0 - normalized_value
            
            # Maps the (potentially inverted) normalized value to the robot's specific joint range
            mapped_action[tiago_joint] = normalized_value * (max_limit - min_limit) + min_limit
            
    # Manually add fixed joint positions that are not controlled by the leader
    mapped_action["arm_3_joint"] = -2.96
    mapped_action["arm_5_joint"] = -1.57
    mapped_action["torso_lift_joint"] = 0.15

    return mapped_action

def format_action_for_tiago(mapped_action: dict):
    """Converts a flat dictionary of joint values to the structured format for the robot."""
    arm_positions = [0.0] * 7
    for i in range(1, 8):
        joint_name = f"arm_{i}_joint"
        if joint_name in mapped_action:
            arm_positions[i-1] = mapped_action[joint_name]

    # Convert gripper position to a binary command (1: open, 2: close)
    gripper_pos = mapped_action.get("gripper_right_finger_joint", 0.0)
    gripper_command = 1 if gripper_pos > 0.0225 else 2  # Threshold is halfway

    return {
        "arm_joint_positions": arm_positions,
        "torso_lift_joint.pos": mapped_action.get("torso_lift_joint", 0.0),
        "base_linear_velocity": 0.0,  # Not controlled in this setup
        "base_angular_velocity": 0.0, # Not controlled in this setup
        "gripper_command": gripper_command,
    }

def real_robot_teleop_loop(teleop: Teleoperator, robot: TiagoClient, fps: int, duration: float | None = None):
    """Main loop for controlling the real robot."""
    joint_limits = get_tiago_joint_limits()
    start = time.perf_counter()
    
    while True:
        loop_start = time.perf_counter()

        # 1. Get action from teleop device and map it
        raw_action = teleop.get_action()
        mapped_action = map_leader_to_tiago(raw_action, joint_limits)
        
        # 2. Format action for the robot
        formatted_action = format_action_for_tiago(mapped_action)
        
        # 3. Send action and get observation
        robot.send_action(formatted_action)
        observation = robot.get_observation()
        
        dt_s = time.perf_counter() - loop_start
        busy_wait(1 / fps - dt_s)
        
        # 4. Print status
        print("\n--- Tiago Real Robot Control ---")
        print(f"Loop time: {1000 * (time.perf_counter() - loop_start):.2f}ms")
        print("\n--- Commanded Action ---")
        print(pformat(formatted_action))
        print("\n--- Actual Observation ---")
        print(pformat(observation))
        
        if duration and time.perf_counter() - start >= duration:
            break
            
        move_cursor_up(20) # Adjust this number based on your terminal height


@draccus.wrap()
def teleoperate(cfg: TiagoTeleopConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))
    
    teleop = make_teleoperator_from_config(cfg.teleop)
    teleop.connect()

    try:           
        robot = TiagoClient(config=cfg.tiago_client)
        try:
            robot.connect()
            robot.calibrate()
            real_robot_teleop_loop(teleop, robot, cfg.fps, duration=cfg.teleop_time_s)
        finally:
            robot.disconnect()

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        teleop.disconnect()
        print("Cleanup complete. Exiting.")

if __name__ == "__main__":
    teleoperate()