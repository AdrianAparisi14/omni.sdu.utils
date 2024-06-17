from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.isaac.core.utils.extensions as extensions_utils
from pxr import Gf, UsdPhysics, Usd, UsdGeom
import omni.usd
from omni.physx import get_physx_interface
from omni.isaac.core import World
from novo_sim.tasks.follow_target import FollowTarget 
from omni.isaac.motion_generation.lula import RRT
from omni.isaac.motion_generation import PathPlannerVisualizer
from novo_sim.ik_solver import KinematicsSolver
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils import transformations as tf
from omni.isaac.core.utils import rotations as r
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core import objects

from novo_sim.robots.ur5e import UR5E
# import ur_rtde_ISSAC_interface as ur_rtde
import numpy as np
import carb
import sys
import signal
import os




assets_root_path = get_assets_root_path()
if assets_root_path is None:
    # Use carb to log warnings, errors and infos in your application (shown on terminal)
    carb.log_error("Could not find nucleus server with /Isaac folder")

NOVO_DIRECTORY =  os.getcwd() + "/novo_sim/"
ur5e_assets = os.path.join(NOVO_DIRECTORY,"robots/ur5e_assets/")


my_world = World(stage_units_in_meters=1.0)
ground_plane= my_world.scene.add_default_ground_plane()

add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/robotiq_gripper/Novo-Nordisk/ur5e_crp200.usd",prim_path="/World/ur5e")
ur5e = UR5E()
my_world.scene.add(ur5e)



my_world.reset()

wall_obstacle = objects.cuboid.VisualCuboid("/World/Wall", position = np.array([0.39254,-0.08834,0.31144]), size = 1.0, scale = np.array([.4,.1,.4]))
ur5e._RRTplanner.add_obstacle(wall_obstacle)
ur5e._RRTplanner.add_ground_plane(ground_plane=ground_plane)
ur5e._RRTplanner.update_world()



add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", f"/target")

## Move Cube to end_effector of robot ##
robot_base_translation,robot_base_orientation = ur5e.get_world_pose()
ee_frame = ur5e.compute_forward_kinematics()
target_orientation = r.euler_angles_to_quat(r.matrix_to_euler_angles(ee_frame[1]))
target = XFormPrim(f"/target",scale=[.04,.04,.04], position=ee_frame[0], orientation=target_orientation)
path_planner_visualizer = PathPlannerVisualizer(ur5e,ur5e._RRTplanner)

observations = my_world.get_observations()
target_pose = target.get_world_pose()[0]

plan = path_planner_visualizer.compute_plan_as_articulation_actions(max_cspace_dist = .01)
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        
        # Check every 60 frames whether the end effector moved
        if my_world.current_time_step_index % 120 == 0:
            curr_target_pose = target.get_world_pose()[0]

            # If the end effector moved: replan
            if np.linalg.norm(target_pose-curr_target_pose) > .01:
                target_pose = curr_target_pose
                ur5e._RRTplanner.set_end_effector_target(target_pose,target_orientation)
                plan = path_planner_visualizer.compute_plan_as_articulation_actions(max_cspace_dist = .01)

        if plan:
            actions = plan.pop(0)
            ur5e._articulation_controller.apply_action(actions)
simulation_app.close()