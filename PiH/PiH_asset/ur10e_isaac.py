# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Universal Robots.

The following configuration parameters are available:

* :obj:`UR10_CFG`: The UR10 arm without a gripper.

Reference: https://github.com/ros-industrial/universal_robot
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
import numpy as np

import os

current_dir = os.path.dirname(os.path.abspath(__file__))

# Construct the full path to the USD file
robot_usd_path = os.path.join(current_dir, "ur10e", "ur10e.usd")
robot_usd_path = os.path.abspath(robot_usd_path)

##
# Configuration
##

UR10e_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=robot_usd_path,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        ## simulation setup
        joint_pos={
            "shoulder_pan_joint": np.pi * 0 / 180,
            "shoulder_lift_joint": np.pi * -90 / 180,
            "elbow_joint": np.pi * 90 / 180,
            "wrist_1_joint": np.pi * -90 / 180,
            "wrist_2_joint": np.pi * -90 / 180,
            "wrist_3_joint": np.pi * 90 / 180,
        },

        ## similar to real setup
        # joint_pos={
        #     "shoulder_pan_joint": np.pi * 0 / 180,
        #     "shoulder_lift_joint": np.pi * -90 / 180,
        #     "elbow_joint": np.pi * -90 / 180,
        #     "wrist_1_joint": np.pi * -90 / 180,
        #     "wrist_2_joint": np.pi * 90 / 180,
        #     "wrist_3_joint": np.pi * -90 / 180,
        # },

        pos=(0.0, 0.0, 0.0),
        rot=(1.0, 0.0, 0.0, 0.0)

        ## real setup
        # joint_pos={
        #     "shoulder_pan_joint": np.pi * -112.5 / 180,
        #     "shoulder_lift_joint": np.pi * -90 / 180,
        #     "elbow_joint": np.pi * -90 / 180,
        #     "wrist_1_joint": np.pi * -90 / 180,
        #     "wrist_2_joint": np.pi * 90 / 180,
        #     "wrist_3_joint": np.pi * -90 / 180,
        # },

        # pos=(0.0, 0.0, 0.0),
        # rot=(0.9808, 0, 0, 0.1951),
        # rot=(0.5556, 0, 0, 0.8315),
        # rot=(1.0, 0.0, 0.0, 0.0)
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            ## original
            # velocity_limit=100,
            # effort_limit=87.0,
            # stiffness=800.0,
            # damping=40.0,
            
            #for testing 
            # velocity_limit=100,
            # effort_limit=87.0,
            # stiffness=800.0,
            # damping=2160.0,

            # ## for training 
            velocity_limit_sim=100,
            effort_limit_sim=87.0,
            stiffness=800.0,
            damping=240.0,
        ),
    },
)
"""Configuration of UR-10 arm using implicit actuator models."""
