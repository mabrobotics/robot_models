"""
Isaac Lab articulation config for hb50. Needs usd/hb50/hb50.usd.

Usage: HB50_CFG.replace(prim_path='/World/envs/env_.*/hb50')
"""

import os

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
import isaaclab.sim as sim_utils

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
HB50_USD_PATH = os.path.join(os.path.dirname(_THIS_DIR), 'hb50.usd')

_JOINT_NAMES = [
    'fr_j0', 'fr_j1', 'fr_j2',
    'fl_j0', 'fl_j1', 'fl_j2',
    'rl_j0', 'rl_j1', 'rl_j2',
    'rr_j0', 'rr_j1', 'rr_j2',
]

# TODO: real stand pose
_DEFAULT_JOINT_POS = {
    'fr_j0': 0.0, 'fr_j1': 0.5, 'fr_j2': -1.2,
    'fl_j0': 0.0, 'fl_j1': -0.5, 'fl_j2': 1.2,
    'rl_j0': 0.0, 'rl_j1': -0.5, 'rl_j2': 1.2,
    'rr_j0': 0.0, 'rr_j1': 0.5, 'rr_j2': -1.2,
}

HB50_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=HB50_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.35),
        joint_pos=_DEFAULT_JOINT_POS,
    ),
    actuators={
        'legs': ImplicitActuatorCfg(
            joint_names_expr=['.*_j[0-2]'],
            effort_limit=16.0,
            velocity_limit=25.0,
            armature=0.013122,
            stiffness=40.0,  # TODO: tune
            damping=2.0,  # TODO: tune
        ),
    },
)
