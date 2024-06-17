from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})
simulation_app.set_setting('persistent/app/viewport/displayOptions', 31983)

import omni.isaac.core.utils.extensions as extensions_utils
from pxr import Gf, UsdPhysics, Usd, UsdGeom
import omni.usd
from omni.physx import get_physx_interface
from omni.isaac.core import World
from novo_sim.tasks.follow_target import FollowTarget 
from novo_sim.tasks.pick_place import PickPlace
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
from novo_sim.controllers.pick_place_controller import PickPlaceController

from omni.isaac.motion_generation.lula import RmpFlow

import argparse
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


NOVO_DIRECTORY =  os.getcwd() + "/novo_sim/"
rmp_config_dir = os.path.join(NOVO_DIRECTORY,"robots/ur5e_assets")

parser = argparse.ArgumentParser()
parser.add_argument("--urdf_path",type=str,default="ur5e.urdf")
parser.add_argument("--rmpflow_config_path",type=str,default="ur5e_rmpflow_config.yaml")
parser.add_argument("--end_effector_frame_name",type=str,default="tcp")
args = parser.parse_args()

my_world = World(stage_units_in_meters=1.0,physics_dt=1/60)
my_world.scene.add_default_ground_plane()


def ur5e_cpr200_robot():
    add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/robotiq_gripper/Novo-Nordisk/ur5e_crp200.usd",prim_path="/World/ur5e")
    gripper = ParallelGripper(
        end_effector_prim_path="/World/ur5e/crp_200_gripper2/crg200_base",
        joint_prim_names=["joint_left", "joint_right"],
        joint_opened_positions=np.array([0, 0]),
        joint_closed_positions=np.array([0.04, 0.04]),
        action_deltas=np.array([-0.04, -0.04]))
    manipulator = SingleManipulator(prim_path="/World/ur5e", name="ur5e_robot", end_effector_prim_name="wrist_3_link", gripper=gripper)
    joints_default_positions = np.array([0.0,-1.5707,1.5707,-1.5707,-1.5707,0.0, 0.0, 0.0])
    manipulator.set_joints_default_state(positions=joints_default_positions)
    return manipulator


my_task = PickPlace(robot=ur5e_cpr200_robot())
my_world.add_task(my_task)

#Initialize an RmpFlow object
rmpflow = RmpFlow(
    robot_description_path = os.path.join(rmp_config_dir,"ur5e_robot_description.yaml"),
    urdf_path = os.path.join(rmp_config_dir,args.urdf_path),
    rmpflow_config_path = os.path.join(rmp_config_dir,args.rmpflow_config_path),
    end_effector_frame_name = args.end_effector_frame_name, #This frame name must be present in the URDF
    maximum_substep_size = .0034
)

my_world.reset()


task_params = my_task.get_params()
my_ur5e = my_world.scene.get_object(task_params["robot_name"]["value"])
my_controller = PickPlaceController(
    name="pick_place_controller", gripper=my_ur5e.gripper, robot_articulation=my_ur5e, rmp_flow=rmpflow
)
articulation_controller = my_ur5e.get_articulation_controller()


## Move Cube to end_effector of robot ##

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            end_effector_offset=np.array([0, 0, 0.12]),
        )
        if my_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)
simulation_app.close()