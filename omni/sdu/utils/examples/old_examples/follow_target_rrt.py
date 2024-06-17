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

my_task = FollowTarget(name="ur5e_follow_target")
my_world.add_task(my_task)
# my_world.scene.add_default_ground_plane(z_position=-0.74)
extensions_utils.disable_extension(extension_name="omni.physx.flatcache")
my_world.reset()


task_params = my_world.get_task("ur5e_follow_target").get_params()
target_name = task_params["target_name"]["value"]
ur5e_name = task_params["robot_name"]["value"]
my_ur5e = my_world.scene.get_object(ur5e_name)

## Setup IK solver and get articulation controller
my_controller = KinematicsSolver(my_ur5e)
articulation_controller = my_ur5e.get_articulation_controller()




mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
rrt_config_dir = os.path.join(mg_extension_path, "path_planner_configs")
rmp_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")

novo_omniverse_path = os.getcwd() + "/novo_sim/omniverse/"
robot_description_path=novo_omniverse_path + "rmpflow/ur5e_robot_description.yaml"
urdf_path=novo_omniverse_path + "ur5e.urdf"
rrt_config_path=ur5e_assets + "ur5e_planner_config.yaml"
# Initialize an RRT object
rrt = RRT(
    robot_description_path =robot_description_path,
    urdf_path = urdf_path,
    rrt_config_path = rrt_config_path,
    end_effector_frame_name = "tool0"
)

wall_obstacle = objects.cuboid.VisualCuboid("/World/Wall", position = np.array([0.39254,-0.08834,0.31144]), size = 1.0, scale = np.array([.4,.1,.4]))
rrt.add_obstacle(wall_obstacle)
rrt.update_world()

## Move Cube to end_effector of robot ##
robot_base_translation,robot_base_orientation = my_ur5e.get_world_pose()
my_controller._kinematics.set_robot_base_pose(robot_base_translation,robot_base_orientation)
joints_default_positions = np.array([0.0,-1.5707,1.5707,-1.5707,-1.5707,0.0])
ee_frame = my_controller._kinematics.compute_forward_kinematics(frame_name="wrist_3_link", joint_positions=joints_default_positions)
cube_orien = r.euler_angles_to_quat(r.matrix_to_euler_angles(ee_frame[1]))
cube = my_world.scene.get_object(target_name)
cube.set_world_pose(position=ee_frame[0], orientation=cube_orien)

path_planner_visualizer = PathPlannerVisualizer(my_ur5e,rrt)

observations = my_world.get_observations()
target_pose = observations[target_name]["position"]

plan = path_planner_visualizer.compute_plan_as_articulation_actions(max_cspace_dist = .01)

# my_ur_rtde = ur_rtde.Ur5e_interface()
# signal.signal(signal.SIGINT,my_ur_rtde.signal_signit)

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        
        observations = my_world.get_observations()
        # Check every 60 frames whether the end effector moved
        if my_world.current_time_step_index % 60 == 0:
            curr_target_pose = observations[target_name]["position"]

            # If the end effector moved: replan
            if np.linalg.norm(target_pose-curr_target_pose) > .01:
                target_pose = curr_target_pose
                rrt.set_end_effector_target(target_pose)
                plan = path_planner_visualizer.compute_plan_as_articulation_actions(max_cspace_dist = .01)

        if plan:
            actions = plan.pop(0)
            articulation_controller.apply_action(actions)
simulation_app.close()