from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

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
from omni.isaac.core.utils import rotations as r

# import ur_rtde_ISSAC_interface as ur_rtde
import numpy as np
import carb
import sys
import signal




assets_root_path = get_assets_root_path()
if assets_root_path is None:
    # Use carb to log warnings, errors and infos in your application (shown on terminal)
    carb.log_error("Could not find nucleus server with /Isaac folder")



my_world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path="/home/tkristiansen/omniverse/sigmund_table2.usd", prim_path="/World/tables")
add_reference_to_stage(usd_path="/home/tkristiansen/omniverse/robot_flange.usd", prim_path="/World/flange")

my_world.scene.add(XFormPrim(prim_path="/World/tables", name="tables"))
my_world.scene.add(XFormPrim(prim_path="/World/world_frame", name="world_frame"))
my_world.scene.add(XFormPrim(prim_path="/World/flange", name="flange"))

my_task = FollowTarget(name="ur5e_follow_target")
my_world.add_task(my_task)
my_world.scene.add_default_ground_plane(z_position=-0.74)
extensions_utils.disable_extension(extension_name="omni.physx.flatcache")
my_world.reset()


task_params = my_world.get_task("ur5e_follow_target").get_params()
target_name = task_params["target_name"]["value"]
ur5e_name = task_params["robot_name"]["value"]
my_ur5e = my_world.scene.get_object(ur5e_name)
articulation_controller = my_ur5e.get_articulation_controller()


## Setup IK solver and get articulation controller
my_controller = KinematicsSolver(my_ur5e)
articulation_controller = my_ur5e.get_articulation_controller()


## Move Cube to end_effector of robot ##
robot_base_translation,robot_base_orientation = my_ur5e.get_world_pose()
my_controller._kinematics.set_robot_base_pose(robot_base_translation,robot_base_orientation)
joints_default_positions = np.array([0.0,-1.5707,1.5707,-1.5707,-1.5707,0.0])
ee_frame = my_controller._kinematics.compute_forward_kinematics(frame_name="wrist_3_link", joint_positions=joints_default_positions)
cube_orien = r.euler_angles_to_quat(r.matrix_to_euler_angles(ee_frame[1]))
cube = my_world.scene.get_object(target_name)
cube.set_world_pose(position=ee_frame[0], orientation=cube_orien)
flange = my_world.scene.get_object("flange")
flange.set_world_pose([robot_base_translation[0],robot_base_translation[1],0])




# my_ur_rtde = ur_rtde.Ur5e_interface()
# signal.signal(signal.SIGINT,my_ur_rtde.signal_signit)

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        
        # robot_base_translation,robot_base_orientation = my_ur5e.get_world_pose()
        # my_controller._kinematics.set_robot_base_pose(robot_base_translation,robot_base_orientation)
        observations = my_world.get_observations()
        joint_positions = observations[ur5e_name]['joint_positions'][:6]
        # my_ur_rtde.slave_robot_joint(joint_positions)

        # print(robot_base_translation)
        # print(test.get_world_pose())
        print(observations[target_name]["orientation"])
        actions, succ = my_controller.compute_inverse_kinematics(
            target_position=observations[target_name]["position"],
            target_orientation=observations[target_name]["orientation"],
        )
        if succ:
            articulation_controller.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")
simulation_app.close()