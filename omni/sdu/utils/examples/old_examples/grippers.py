from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})
simulation_app.set_setting('persistent/app/viewport/displayOptions', 31983)

import omni.isaac.core.utils.extensions as extensions_utils
from pxr import Gf, UsdPhysics, Usd, UsdGeom
import omni.usd
from omni.physx import get_physx_interface
from omni.isaac.core import World
from novo_sim.tasks.follow_target import FollowTarget 
from novo_sim.ik_solver import KinematicsSolver
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils import transformations as tf
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.utils import rotations as r
from novo_sim.robots.ur5e import UR5E

from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.franka import Franka

from omni.isaac.core.utils.types import ArticulationAction
import novo_sim.ur_rtde_ISSAC_interface as ur_rtde

import numpy as np
import carb
import sys
import os
import signal
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    # Use carb to log warnings, errors and infos in your application (shown on terminal)
    carb.log_error("Could not find nucleus server with /Isaac folder")
novo_omniverse_path = os.getcwd() + "/novo_sim/omniverse/"



def add_robot(my_world):
    add_reference_to_stage(usd_path= novo_omniverse_path + "robotiq_gripper/Novo-Nordisk/ur5e_crp2.usd",prim_path="/World/ur5e")
    gripper = ParallelGripper(
        end_effector_prim_path="/World/ur5e/crp_200_gripper2/crg200_base",
        joint_prim_names=["joint_left", "joint_right"],
        joint_opened_positions=np.array([0, 0]),
        joint_closed_positions=np.array([0.04, 0.04]),
        action_deltas=np.array([-0.04, -0.04]))
    manipulator = SingleManipulator(prim_path="/World/ur5e", name="ur5e_robot", end_effector_prim_name="wrist_3_link", gripper=gripper, position=[0, 0.5, 0])
    joints_default_positions = np.array([0.0,-1.5707,1.5707,-1.5707,-1.5707,0.0, 0.0, 0.0])
    manipulator.set_joints_default_state(positions=joints_default_positions)
    my_world.scene.add(manipulator)
    return gripper



my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
gripper = add_robot(my_world)
franka = Franka(prim_path="/World/franka",name="franka")
my_world.scene.add(franka)
my_world.reset()

## Move Cube to end_effector of robot ##

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if i == 200:
            gripper.close()
            franka.gripper.close()
            pass
        if i == 400:
            gripper.open()
            franka.gripper.open()
            pass
            i = 0
        if my_world.current_time_step_index == 0:
            my_world.reset()
        i += 1
simulation_app.close()