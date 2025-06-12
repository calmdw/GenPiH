# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from omni.isaac.lab.utils import configclass

import omni.isaac.lab_tasks.manager_based.manipulation.reach.mdp as mdp
from omni.isaac.lab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg
from omni.isaac.lab_tasks.manager_based.PiH.PiH_env_cfg import PiHEnvCfg
from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg

##
# Pre-defined configs
##
from omni.isaac.lab_assets import UR10_CFG, UR5EFLUENLT_CFG, UR10e_CFG, UR5EISAAC_CFG # isort: skip
from omni.isaac.lab.markers.config import FRAME_MARKER_CFG

##
# Environment configuration
##


@configclass
class PiHUR10eRelEnvCfg(PiHEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        
        # switch robot to ur10
        self.scene.robot = UR10e_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # override events
        self.events.reset_robot_joints.params["position_range"] = (0.98, 1.05)
        # override rewards
        # self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["peg"]
        # self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["peg"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["peg"]
        # PiH sepecific rewards:
        self.rewards.peg_alignment.params["asset_cfg"].body_names = ["peg"]
        self.rewards.peg_alignment_tanh.params["asset_cfg"].body_names = ["peg"]
        self.rewards.peg_aligned_insert.params["asset_cfg"].body_names = ["peg"]
        # self.rewards.success_reward.params["asset_cfg"].body_names = ["peg"]
        # override actions
        self.actions.arm_action = mdp.RelativeJointPositionActionCfg(
            asset_name="robot", joint_names=[".*"], scale=0.05, use_zero_offset=True
        )
        # override command generator body
        # end-effector is along x-direction
        self.commands.ee_pose.body_name = "peg"
        self.commands.ee_pose.ranges.roll = (math.pi, math.pi)
        self.commands.ee_pose.ranges.pitch = (0, 0)
        self.commands.ee_pose.ranges.yaw = (math.pi / 2, math.pi / 2 )
        self.commands.ee_pose.ranges.pos_z = (-0.044, 0.045)


@configclass
class PiHUR10eRelEnvCfg_PLAY(PiHUR10eRelEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
