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
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils import transformations as tf
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.utils import rotations as r
from novo_sim.robots.ur5e import UR5E

from omni.isaac.manipulators.grippers import ParallelGripper

from omni.isaac.core.utils.types import ArticulationAction
import novo_sim.ur_rtde_ISSAC_interface as ur_rtde
import numpy as np
import carb
import sys
import signal

class shutdown_handler():
    def __init__(
        self,
        robots: None
    )-> None:
        self.robots = robots



    def signal_signit(self, sig, frame):
        for i in  self.robots:
            i.rtde_c.teachMode()
            i.rtde_c.servoStop()
            i.rtde_c.stopScript()




def scene_setup(world):
    add_reference_to_stage(usd_path="/home/tkristiansen/omniverse/robot_flange.usd", prim_path="/World/flange2")
    add_reference_to_stage(usd_path="/home/tkristiansen/omniverse/robot_flange.usd", prim_path="/World/flange3")
    add_reference_to_stage(usd_path="/home/tkristiansen/omniverse/robot_flange.usd", prim_path="/World/flange4")
    world.scene.add(XFormPrim(prim_path="/World/flange2", name="flange2"))
    world.scene.add(XFormPrim(prim_path="/World/flange3", name="flange3"))
    world.scene.add(XFormPrim(prim_path="/World/flange4", name="flange4"))
    joints_default_positions = np.array([0.0,-1.5707,1.5707,-1.5707,-1.5707,0.0])
    asset_path = "/home/tkristiansen/omniverse/robotiq_gripper/Novo-Nordisk/ur5e_2.usd"
    robot_base_translation =[-0.22565849,0.4204293,0.035];



    cam = Camera(prim_path="/World/camera", name="camera", position=[-1.0, 0, 0.4],frequency=1)
    cam.set_focal_length(1.5)
    cam.set_clipping_range(0.00001)
    world.scene.add(cam)


    flange2 = world.scene.get_object("flange2")
    flange2.set_default_state(position=[-robot_base_translation[0],-robot_base_translation[1]+0.1,0])
    joints_default_positions = np.array([np.pi,-1.5707,-1.5707,-1.5707,1.5707,0.0, 0.,0.])
    world.scene.add(UR5E(prim_path="/World/ur5e2", name="ur5e2",usd_path="/home/tkristiansen/omniverse/robotiq_gripper/Novo-Nordisk/ur5e_crp200.usd" , end_effector_prim_name="wrist_3_link",position= [-robot_base_translation[0],-robot_base_translation[1]+0.1,0.035]))
    ur5e2 = world.scene.get_object("ur5e2")
    ur5e2.set_joints_default_state(positions=joints_default_positions)


    flange3 = world.scene.get_object("flange3")
    flange3.set_default_state([-robot_base_translation[0],robot_base_translation[1]-0.1,0])
    joints_default_positions = np.array([0.0,-1.5707,1.5707,-1.5707,-1.5707,-1.5707])
    asset_path = "/home/tkristiansen/omniverse/robotiq_gripper/Novo-Nordisk/ur5e_rob.usd"
    ur5e3 = UR5E(prim_path="/World/ur5e3", name="ur5e3",usd_path=asset_path, end_effector_prim_name="wrist_3_link",position= [-robot_base_translation[0],robot_base_translation[1]-0.1,0.035])
    ur5e3.set_joints_default_state(positions=joints_default_positions)
    world.scene.add(ur5e3)

    flange4 = world.scene.get_object("flange4")
    flange4.set_default_state([robot_base_translation[0],-robot_base_translation[1],0])
    # joints_default_positions = np.array([np.pi,-1.5707,-1.5707,-1.5707,1.5707,0.0          ,0,0])
    joints_default_positions = np.array([0,-1.5707,-1.5707,-1.5707,1.5707,-np.pi,0,0])
    # joints_default_positions = np.array([0,-1.5707,1.5707,-1.5707,-1.5707,0.0,0,0])
    ur5e4 = UR5E(prim_path="/World/ur5e4",usd_path="/home/tkristiansen/omniverse/robotiq_gripper/Novo-Nordisk/ur5e_crp200.usd", name="ur5e4", end_effector_prim_name="wrist_3_link",position= [robot_base_translation[0],-robot_base_translation[1],0.035], orientation=r.euler_angles_to_quat([0,0,60],True))
    ur5e4.set_joints_default_state(positions=joints_default_positions)
    world.scene.add(ur5e4)







assets_root_path = get_assets_root_path()
if assets_root_path is None:
    # Use carb to log warnings, errors and infos in your application (shown on terminal)
    carb.log_error("Could not find nucleus server with /Isaac folder")



my_world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path="/home/tkristiansen/omniverse/sigmund_table2.usd", prim_path="/World/tables")
add_reference_to_stage(usd_path="/home/tkristiansen/omniverse/robot_flange.usd", prim_path="/World/flange1")
my_world.scene.add(XFormPrim(prim_path="/World/tables", name="tables"))
my_world.scene.add(XFormPrim(prim_path="/World/world_frame", name="world_frame"))
my_world.scene.add(XFormPrim(prim_path="/World/flange1", name="flange1"))


scene_setup(my_world)


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
flange = my_world.scene.get_object("flange1")
flange.set_world_pose([robot_base_translation[0],robot_base_translation[1],0])
cube_default_position=cube.get_world_pose()

ur5e2 = my_world.scene.get_object("ur5e2")
ur5e2_controller = ur5e2.get_articulation_controller()


target_joint_positions = ur5e2.get_joint_positions() 

ur5e4 = my_world.scene.get_object("ur5e4")
ur5e4_controller = ur5e4.get_articulation_controller()

cam  = my_world.scene.get_object("camera")
print(np.shape(cam.get_current_frame()['rgba']))
print(cam.get_intrinsics_matrix())
print(cam.get_world_pose())



# my_ur_rtde = ur_rtde.Ur5e_interface(robot_ip="192.168.1.11")
my_ur_rtde = None 
if my_ur_rtde != None:
    signal.signal(signal.SIGINT,my_ur_rtde.signal_signit)


# robot_4_rtde = ur_rtde.Ur5e_interface(robot_ip="192.168.1.14")
robot_4_rtde = None 
if robot_4_rtde != None:
    robot_4_rtde.set_teachmode()
    ur5e4_joint_pos = ur5e4.get_joint_positions()
    ur5e4_joint_pos[:6] = robot_4_rtde.get_joint_positions()
    if my_ur_rtde is None:
        signal.signal(signal.SIGINT,robot_4_rtde.signal_signit)

# robots_rtde = [my_ur_rtde, robot_4_rtde]
# shutdown_rtde = shutdown_handler(robots=robots_rtde)
shutdown_rtde = None


if my_ur_rtde != None and robot_4_rtde != None:
    signal.signal(signal.SIGINT,shutdown_rtde.signal_signit)

i = 0
cube_prev_observation = cube_default_position

while simulation_app.is_running():
    my_world.step(render=True)
    # my_world.step(render=False)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        

        ###################### ROBOT_1 ########################
        if robot_4_rtde != None:
            ur5e4_joint_pos[:6] = robot_4_rtde.get_joint_positions()
            ur5e4_controller.apply_action(ArticulationAction(joint_positions=ur5e4_joint_pos))
        ###################### ROBOT_1 ########################



        # target_joint_positions[2] = np.sin(i/100)*1.57/2.0 - 1.5707
        # print(target_joint_positions)


        ###################### ROBOT_2 ########################
        gripper_positions = ur5e2.get_joint_positions()[6:]
        if i%1000 < 500:
            target_joint_positions[6:]=[gripper_positions[0] + 0.0001, gripper_positions[1] + 0.0001]
        if i%1000 > 500:
            target_joint_positions[6:]=[gripper_positions[0] - 0.0001, gripper_positions[1] - 0.0001]

        ur5e2_controller.apply_action(ArticulationAction(joint_positions=target_joint_positions))
        i += 1
        #######################################################


        ###################### ROBOT_1 ########################
        robot_base_translation,robot_base_orientation = my_ur5e.get_world_pose()
        my_controller._kinematics.set_robot_base_pose(robot_base_translation,robot_base_orientation)
        observations = my_world.get_observations()
        joint_positions = observations[ur5e_name]['joint_positions'][:6]
        if abs(cube_default_position[0][0]- observations[target_name]["position"][0]) < 0.2 and abs(cube_default_position[0][1]- observations[target_name]["position"][1]) < 0.2 and abs(cube_default_position[0][2]- observations[target_name]["position"][2]) < 0.15:
            actions, succ = my_controller.compute_inverse_kinematics(
                target_position=observations[target_name]["position"],
                target_orientation=observations[target_name]["orientation"],
            )
            cube_prev_observation = observations[target_name]["position"]
            if my_ur_rtde != None:
                my_ur_rtde.slave_robot_joint(joint_positions)
        else:
            cube.set_world_pose(position=cube_prev_observation)
        if succ:
            articulation_controller.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")
            if robot_4_rtde != None:
                robot_4_rtde.signal_signit()
            if my_ur_rtde != None:
                my_ur_rtde.signal_signit()
        ###################### ROBOT_1 ########################



simulation_app.close()