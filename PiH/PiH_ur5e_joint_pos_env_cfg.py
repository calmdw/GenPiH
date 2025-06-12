# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from omni.isaac.lab.utils import configclass

import omni.isaac.lab_tasks.manager_based.PiH.mdp as mdp
from omni.isaac.lab_tasks.manager_based.PiH.PiH_env_cfg import PiHEnvCfg
from omni.isaac.lab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
from omni.isaac.lab_assets import UR10_CFG, UR5EFLUENLT_CFG, UR5EISAAC_CFG  # isort: skip

##
# Environment configuration
##

@configclass
class PiHUR5eEnvCfg(PiHEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # self.scene.robot = UR5EFLUENLT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot = UR5EISAAC_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        
        # override events
        self.events.reset_robot_joints.params["position_range"] = (0.98, 1.05)
        # override rewards
        # self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["peg"]
        # self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["peg"]
        self.rewards.ori_error.params["asset_cfg"].body_names = ["peg"]
        # PiH sepecific rewards:
        self.rewards.xyz_error.params["asset_cfg"].body_names = ["peg"]
        self.rewards.xyz_error_tanh.params["asset_cfg"].body_names = ["peg"]
        self.rewards.aligned_insert.params["asset_cfg"].body_names = ["peg"]
        # self.rewards.end_effector_orientation_tracking_fine_grained.params["asset_cfg"].body_names = ["peg"]
        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*"], scale=0.8, use_default_offset=True
        )
        
        # self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
        #     asset_name="robot",
        #     joint_names=["gripper_.*"],
        #     open_command_expr={"gripper_.*": 0.02},
        #     close_command_expr={"gripper_.*": 0.0},
        # )

        # override command generator body
        # end-effector is along x-direction
        self.commands.ee_pose.body_name = "peg"
        self.commands.ee_pose.ranges.roll = (math.pi, math.pi)
        self.commands.ee_pose.ranges.pitch = (0, 0)
        self.commands.ee_pose.ranges.yaw = (math.pi / 2, math.pi / 2 )
        self.commands.ee_pose.ranges.pos_z = (-0.044, 0.045)


@configclass
class PiHUR5eEnvCfg_PLAY(PiHUR5eEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
