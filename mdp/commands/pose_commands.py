# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Sub-module containing command generators for pose tracking."""

from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import CommandTerm
from isaaclab.markers import VisualizationMarkers
from isaaclab.utils.math import combine_frame_transforms, compute_pose_error, quat_from_euler_xyz, quat_unique, subtract_frame_transforms

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .commands_cfg import UniformPoseCommandCfg_PiH


class UniformPoseCommand_PiH(CommandTerm):
    """Command generator for generating pose commands uniformly.

    The command generator generates poses by sampling positions uniformly within specified
    regions in cartesian space. For orientation, it samples uniformly the euler angles
    (roll-pitch-yaw) and converts them into quaternion representation (w, x, y, z).

    The position and orientation commands are generated in the base frame of the robot, and not the
    simulation world frame. This means that users need to handle the transformation from the
    base frame to the simulation world frame themselves.

    .. caution::

        Sampling orientations uniformly is not strictly the same as sampling euler angles uniformly.
        This is because rotations are defined by 3D non-Euclidean space, and the mapping
        from euler angles to rotations is not one-to-one.

    """

    cfg: UniformPoseCommandCfg_PiH
    """Configuration for the command generator."""

    def __init__(self, cfg: UniformPoseCommandCfg_PiH, env: ManagerBasedEnv):
        """Initialize the command generator class.

        Args:
            cfg: The configuration parameters for the command generator.
            env: The environment object.
        """
        # initialize the base class
        super().__init__(cfg, env)

        # extract the robot and body index for which the command is generated
        self.robot: Articulation = env.scene[cfg.asset_name]
        self.body_idx = self.robot.find_bodies(cfg.body_name)[0][0]

        # create buffers
        # -- commands: (x, y, z, qw, qx, qy, qz) in root frame
        self.pose_command_b = torch.zeros(self.num_envs, 7, device=self.device)
        self.pose_command_b[:, 0] = -0.054 * 1.0
        self.pose_command_b[:, 1] = -0.055 * 1.0
        self.pose_command_b[:, 2] = -0.015 * 1.0
        self.pose_command_b[:, 3] = 1.0
        self.pose_command_w = torch.zeros_like(self.pose_command_b)
        self.pose_command_rel_robot = torch.zeros_like(self.pose_command_w)
        # -- metrics
        self.metrics["position_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["orientation_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["success_rate"] = torch.zeros(self.num_envs, device=self.device)


    #     asset: Articulation = env.scene[asset_cfg.name]
    # - asset.data.default_joint_vel[:, asset_cfg.joint_ids]

        self.use_pih_holes = self.cfg.PiH_hole_pos_enable
        self.base: RigidObject = env.scene["base"]

    def __str__(self) -> str:
        msg = "UniformPoseCommand:\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape[1:])}\n"
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        return msg

    """
    Properties
    """

    @property
    def command(self) -> torch.Tensor:
        """The desired pose command. Shape is (num_envs, 7).

        The first three elements correspond to the position, followed by the quaternion orientation in (w, x, y, z).

        Considering there are many holes in the base, therefore, the relative pose from sepecific hole to the robot frame needs one more process.
        That is transfer the pose of the sepecific hole in world frame to robot frame.
        """
        
        return self.pose_command_w

    """
    Implementation specific functions.
    """

    def _update_metrics(self):
        # transform command from base frame to simulation world frame
        self.pose_command_w[:, :3], self.pose_command_w[:, 3:] = combine_frame_transforms(
            self.base.data.root_pos_w,
            self.base.data.root_quat_w,
            self.pose_command_b[:, :3],
            self.pose_command_b[:, 3:],
        )
        # compute the error
        pos_error, rot_error = compute_pose_error(
            self.pose_command_w[:, :3],
            self.pose_command_w[:, 3:],
            self.robot.data.body_state_w[:, self.body_idx, :3],
            self.robot.data.body_state_w[:, self.body_idx, 3:7],
        )
        # print("self.pose_command_b", self.pose_command_b)
        # print("self.pose_command_w", self.pose_command_w)
        # print("self.robot.data.body_state_w[:, self.body_idx, :3]", self.robot.data.body_state_w[:, self.body_idx, :7])
        # print("pos_error", pos_error)
        error_xy = torch.sum(torch.abs(pos_error[:, :2]), dim=-1)
        error_xyz = torch.sum(torch.abs(pos_error), dim=-1)
        error_ang = torch.sum(torch.abs(rot_error), dim=-1)
        # print("distance_xyz", distance_xyz)
        # print("distance_ori", distance_ori)
        num_envs = self.pose_command_w.shape[0]
        # print("[NUM_ENVS_command]: ", num_envs, type(num_envs))
        
        is_xy_aligned = torch.where(
        error_xy < 9e-4,
        torch.ones_like(error_xy),
        torch.zeros_like(error_xy),
        )

        success_xyz = torch.where(
        error_xyz < 7e-3,
        torch.ones_like(error_xyz),
        torch.zeros_like(error_xyz),
        )
    
        is_ori_aligned = torch.where(
        error_ang < 8e-2,
        torch.ones_like(error_ang),
        torch.zeros_like(error_ang),
        )

        # print("distance_xyz_ori", distance_xyz_ori)
        is_successed = success_xyz * is_ori_aligned
        # print("is_successed", is_successed)
        # success_rate = torch.sum(is_successed) * 100/ num_envs
        # print("success_rate", success_rate)

        self.metrics["position_error"] = torch.norm(pos_error, dim=-1)
        self.metrics["orientation_error"] = torch.norm(rot_error, dim=-1)
        self.metrics["success_rate"] = is_successed
        self.metrics["xy_aligned"] = is_xy_aligned
        self.metrics["ori_aligned"] = is_ori_aligned

    def _resample_command(self, env_ids: Sequence[int]):
        # sample new pose targets
        # -- position
        # print("[[env_ids]]", env_ids)
        # if not self.use_pih_holes:
        #     print("^^^^^^^^^^^^^^^not using pih holes^^^^^^^^^^^^^^^")
        #     r = torch.empty(len(env_ids), device=self.device)
        #     self.pose_command_b[env_ids, 0] = r.uniform_(*self.cfg.ranges.pos_x)
        #     self.pose_command_b[env_ids, 1] = r.uniform_(*self.cfg.ranges.pos_y)
        #     self.pose_command_b[env_ids, 2] = r.uniform_(*self.cfg.ranges.pos_z)
        #     # -- orientation
        #     euler_angles = torch.zeros_like(self.pose_command_b[env_ids, :3])
        #     euler_angles[:, 0].uniform_(*self.cfg.ranges.roll)
        #     euler_angles[:, 1].uniform_(*self.cfg.ranges.pitch)
        #     euler_angles[:, 2].uniform_(*self.cfg.ranges.yaw)
        #     quat = quat_from_euler_xyz(euler_angles[:, 0], euler_angles[:, 1], euler_angles[:, 2])
        #     # make sure the quaternion has real part as positive
        #     self.pose_command_b[env_ids, 3:] = quat_unique(quat) if self.cfg.make_quat_unique else quat
        
        if self.use_pih_holes:
            # r = torch.empty(len(env_ids), device=self.device)
            # self.pose_command_b[env_ids, 2] = r.uniform_(*self.cfg.ranges.pos_z)
            
            self.pose_command_w[:, :3], self.pose_command_w[:, 3:] = combine_frame_transforms(
                self.base.data.root_pos_w,
                self.base.data.root_quat_w,
                self.pose_command_b[:, :3],
                self.pose_command_b[:, 3:],
            )
            # print("pose_commend_w", pose_commend_w)
            # print("pose_commend_b", pose_commend_b)
            
            # print("[self.pose_command_b]", self.pose_command_b)

            # euler_angles = torch.zeros_like(self.pose_command_b[env_ids, :3])
            # euler_angles[:, 0].uniform_(*self.cfg.ranges.roll)
            # euler_angles[:, 1].uniform_(*self.cfg.ranges.pitch)
            # euler_angles[:, 2].uniform_(*self.cfg.ranges.yaw)
            # quat = quat_from_euler_xyz(euler_angles[:, 0], euler_angles[:, 1], euler_angles[:, 2])
            ## make sure the quaternion has real part as positive
            # self.pose_command_b[env_ids, 3:] = quat_unique(quat) if self.cfg.make_quat_unique else quat

    def _update_command(self):
        pass

    def _set_debug_vis_impl(self, debug_vis: bool):
        # create markers if necessary for the first tome
        if debug_vis:
            if not hasattr(self, "goal_pose_visualizer"):
                # -- goal pose
                self.goal_pose_visualizer = VisualizationMarkers(self.cfg.goal_pose_visualizer_cfg)
                # -- current body pose
                self.current_pose_visualizer = VisualizationMarkers(self.cfg.current_pose_visualizer_cfg)
            # set their visibility to true
            self.goal_pose_visualizer.set_visibility(True)
            self.current_pose_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_pose_visualizer"):
                self.goal_pose_visualizer.set_visibility(False)
                self.current_pose_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        # check if robot is initialized
        # note: this is needed in-case the robot is de-initialized. we can't access the data
        if not self.robot.is_initialized:
            return
        # update the markers
        # -- goal pose
        self.goal_pose_visualizer.visualize(self.pose_command_w[:, :3], self.pose_command_w[:, 3:])
        # -- current body pose
        body_pose_w = self.robot.data.body_state_w[:, self.body_idx]
        self.current_pose_visualizer.visualize(body_pose_w[:, :3], body_pose_w[:, 3:7])
