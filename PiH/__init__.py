# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Franka-Cabinet environment.
"""

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

# gym.register(
#     id="Fluently-UR5e-PiH-v0",
#     entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
#     disable_env_checker=True,
#     kwargs={
#         "env_cfg_entry_point": PiHUR5eEnvCfg,
#         "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg_ur5e.yaml",
#     },
# )

gym.register(
    id="Fluently-UR10e-PiH-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.PiH_ur10e_joint_pos_env_cfg:PiHUR10eEnvCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg_ur10e.yaml",
    },
)

# gym.register(
#     id="Fluently-UR10e-PiH-Rel-v0",
#     entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
#     disable_env_checker=True,
#     kwargs={
#         "env_cfg_entry_point": PiHUR10eRelEnvCfg,
#         "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg_ur10e_rel.yaml",
#     },
# )

# gym.register(
#     id="Fluently-UR10e-PiH-Rel-Play-v0",
#     entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
#     disable_env_checker=True,
#     kwargs={
#         "env_cfg_entry_point": PiHUR10eRelEnvCfg_PLAY,
#         "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg_ur10e.yaml",
#     },
# )

# gym.register(
#     id="Fluently-UR10e-PiH-Play-v0",
#     entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
#     disable_env_checker=True,
#     kwargs={
#         "env_cfg_entry_point": PiHUR10eEnvCfg_PLAY,
#         "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg_ur10e_rel.yaml",
#     },
# )
