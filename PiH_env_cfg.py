# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING

import math
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ActionTermCfg as ActionTerm
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import isaaclab_tasks.PiH.mdp as mdp
import os

current_dir = os.path.dirname(os.path.abspath(__file__))

# Construct the full path to the USD file
base_usd_path = os.path.join(current_dir, "PiH_asset", "cranfield_model", "Cranfield parts - CranfieldBase.usd")
base_usd_path = os.path.abspath(base_usd_path)



##
# Scene definition
##


@configclass
class PiHSceneCfg(InteractiveSceneCfg):
    """Configuration for the scene with a robotic arm."""

    # world
    ground: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )

    # table: AssetBaseCfg = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/Table",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path="/home/xinyu/Downloads/xinyu_file/PiH_fluently/model/scene/Single_Siegmund_table.usd",
    #         scale=(1, 1, 1),
    #     ),
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=(0.45, 0.0, 0.0), rot=(1, 0.0, 0.0, 0.0)),
    # )

    base: RigidObjectCfg = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Base",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.45, 0.0, 0.03], rot=[0, 1, 0, 0]),
            spawn=sim_utils.UsdFileCfg(
                # usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                usd_path=base_usd_path,
                scale=(1.0, 1.0, 1.0),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1e-8,
                    max_linear_velocity=1e-8,
                    max_depenetration_velocity=5.0,
                    disable_gravity=True,
                ),
                mass_props=sim_utils.MassPropertiesCfg(density=1000000000.0),
            ),
        )
    
    # robots
    robot: ArticulationCfg = MISSING

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )
    
    # ee_frame: FrameTransformerCfg = MISSING

##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    ee_pose = mdp.UniformPoseCommandCfg_PiH(
        asset_name="robot",
        body_name=MISSING,
        resampling_time_range=(4.0, 4.0),
        debug_vis=True,
        PiH_hole_pos_enable=True,
        ranges=mdp.UniformPoseCommandCfg_PiH.Ranges(
            # pos_x=(0.696, 0.696),
            # pos_y=(0.235, 0.235),
            pos_x=(0.691, 0.691),
            pos_y=(0.24, 0.24),
            # pos_x=(0.606, 0.606),
            # pos_y=(0.055, 0.055),
            pos_z=(0.045, 0.046),
            roll=(math.pi * -10 / 180, math.pi * 10 / 180),
            pitch=MISSING,  # depends on end-effector axis
            yaw=(math.pi * 180 / 180, math.pi * 240 / 180),
        ),
    )



@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    arm_action: ActionTerm = MISSING
    gripper_action: ActionTerm | None = None


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        base_pos = ObsTerm(func=mdp.base_pose_in_robot_root_frame)
        # peg_pos = ObsTerm(func=mdp.base_position_in_robot_root_frame)
        target_object_pose = ObsTerm(func=mdp.generated_commands, params={"command_name": "ee_pose"})
        # pose_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "ee_pose"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            # "position_range": (0.5, 1.5),
            "position_range": (0.1, 0.3),
            "velocity_range": (0.0, 0.0),
        },
    )

    reset_object_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (0.2, 0.56), "y": (-0.26, 0.26), "z": (0.0, 0.16), "roll": (0.0, 0.66), "pitch":(0.0, 0.66), "yaw": (0.0, 0.66)},
            ## 7ablation exp 
            # "pose_range": {"x": (0.2, 0.56), "y": (-0.26, 0.26), "z": (0.0, 0.16), "roll": (-0.436332, 0.436332), "pitch":(-0.436332, 0.436332), "yaw": (-0.436332, 0.436332)},
            # "pose_range": {"x": (0.2, 0.6), "y": (0.0, 0.0), "z": (0.0, 0.0), "roll": (0.0, 0.0), "pitch":(0.0, 0.0), "yaw": (0.0, 0.0)},
            # "pose_range": {"x": (0.3, 0.3), "y": (-0.25, 0.25), "z": (0.0, 0.0), "roll": (0.0, 0.0), "pitch":(0.0, 0.0), "yaw": (0.0, 0.0)},
            # "pose_range": {"x": (0.3, 0.3), "y": (0.0, 0.0), "z": (0.0, 0.16), "roll": (0.0, 0.0), "pitch":(0.0, 0.0), "yaw": (0.0, 0.0)},
            # "pose_range": {"x": (0.3, 0.3), "y": (0.0, 0.0), "z": (0.12, 0.12), "roll": (-0.436332, 0.436332), "pitch":(0.0, 0.0), "yaw": (0.0, 0.0)},
            # "pose_range": {"x": (0.3, 0.3), "y": (0.0, 0.0), "z": (0.12, 0.12), "roll": (0.0, 0.0), "pitch":(-0.436332, 0.436332), "yaw": (0.0, 0.0)},
            # "pose_range": {"x": (0.3, 0.3), "y": (0.0, 0.0), "z": (0.12, 0.12), "roll": (0.0, 0.0), "pitch":(0.0, 0.0), "yaw": (-0.436332, 0.436332)},
            ## for testing
            # "pose_range": {"x": (0.249+0.054, 0.249+0.054), "y": (0.1465-0.055, 0.1465-0.055), "z": (0.105, 0.105), "roll": (0, 0), "pitch":(0.0, 0.0), "yaw": (0.0, 0.0)},
            # "pose_range": {"x": (0.2308+0.054, 0.2308+0.054), "y": (0.1455-0.055, 0.1455-0.055), "z": (0.096, 0.096), "roll": (0.0, 0.0), "pitch": (0.174533, 0.174533), "yaw": (0.0, 0.0)},
            # "pose_range": {"x": (0.234+0.054, 0.234+0.054), "y": (0.1305-0.055, 0.1305-0.055), "z": (0.099, 0.099), "roll": (0.174533, 0.174533), "pitch": (0.174533, 0.174533), "yaw": (0.0, 0.0)},
            # "pose_range": {"x": (0.249+0.054-0.0347, 0.249+0.054-0.0347), "y": (0.1465-0.055-0.0196, 0.1465-0.055-0.0196), "z": (0.115, 0.115), "roll": (0, 0), "pitch":(0.0, 0.0), "yaw": (0.52333333, 0.52333333)},
            # 0.174533, 0.174533
            # "pose_range": {"x": (0.2, 0.56), "y": (-0.26, 0.26), "z": (0.10, 0.20), "roll": (0.0, 0.0), "pitch":(0.0, 0.0), "yaw": (0.0, 0.86)},
            ## for ur5e
            # "pose_range": {"x": (0.26, 0.56), "y": (-0.26, 0.16), "z": (0.0, 0.08), "roll": (0.0, 0.36), "yaw": (0.0, 0.76)},
            "velocity_range": {}, 
            "asset_cfg": SceneEntityCfg("base", body_names="Base"),
        },
    )

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""
    ####
    #note:
    #1. pre_training, less action penalty around 1e-3 is enough
    #2. pre_training no use parse reward, only the xyz and orientation dense reward in local frame is better and 
    #convient
    #3. have a look of the later fine-tuning and curriculum thing, improve and narrow the reward.
    #####
    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.002) #0.06
    joint_acc = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.002, #0.001
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    ori_error = RewTerm(
        func=mdp.orientation_command_error,
        weight=-0.28,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=MISSING), "command_name": "ee_pose"},
    )

    print("ori_error", ori_error)

    # ori_aligned = RewTerm(
    #     func=mdp.ori_aligned_reward,
    #     weight=1,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="peg"), "command_name": "ee_pose"},
    # )

    # xy_aligned = RewTerm(
    #     func=mdp.xy_aligned_reward,
    #     weight=1,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="peg"), "command_name": "ee_pose"},
    # )

    success = RewTerm(
        func=mdp.success_reward,
        weight=6,
        params={"asset_cfg": SceneEntityCfg("robot", body_names="peg"), "command_name": "ee_pose"},
    )

    ### PiH sepcific rewards
    aligned_insert = RewTerm(
        func=mdp.z_command_error_tanh,
        weight=0.66,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=MISSING), "std": 0.1, "command_name": "ee_pose"},
    )

    xyz_error = RewTerm(
        func=mdp.xyz_command_error,
        weight=-0.16,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=MISSING), "command_name": "ee_pose"},
    )

    xyz_error_tanh = RewTerm(
        func=mdp.xyz_command_error_tanh,
        weight=0.26,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=MISSING), "std": 0.1, "command_name": "ee_pose"},
    )

    # reaching_object = RewTerm(func=mdp.object_ee_distance, params={"std": 0.1}, weight=1.0)


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # success = DoneTerm(
    #     func=mdp.success_pih,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="peg"), "command_name": "ee_pose"},
    #     )

@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    # action_rate = CurrTerm(
    #     func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -0.01, "num_steps": 20000}
    # )

    # joint_vel = CurrTerm(
    #     func=mdp.modify_reward_weight, params={"term_name": "joint_acc", "weight": -0.01, "num_steps": 20000}
    # )

    # action_rate = CurrTerm(
    #     func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -0.1, "num_steps": 600000}
    # )


##
# Environment configuration
##


@configclass
class PiHEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the reach end-effector pose tracking environment."""

    # Scene settings
    scene: PiHSceneCfg = PiHSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.sim.render_interval = self.decimation
        self.episode_length_s = 30.0
        self.viewer.eye = (3.5, 3.5, 3.5)
        # simulation settings
        self.sim.dt = 1.0 / 120
