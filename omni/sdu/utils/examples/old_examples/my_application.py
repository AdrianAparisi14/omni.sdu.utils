from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot 
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.prims import XFormPrim, RigidPrim
from omni.isaac.core.articulations import Articulation
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
import omni.isaac.examples.novo_sim.ur_rtde_ISSAC_interface as ur_rtde
import numpy as np
import carb

my_world = World(stage_units_in_meters=1.0)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find nucleus server with /Isaac folder")
asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
path_to_gripper = "/home/tkristiansen/omniverse/robotiq_gripper/Novo-Nordisk/ur5e_2.usd"
add_reference_to_stage(usd_path="/home/tkristiansen/omniverse/sigmund_table.usd", prim_path="/World/sigmund_table")
# add_reference_to_stage(usd_path=asset_path, prim_path="/World/ur5e")
add_reference_to_stage(usd_path=path_to_gripper, prim_path="/World/ur5e")
gripper = ParallelGripper(
    #We chose the following values while inspecting the articulation
    end_effector_prim_path="/World/ur5e/robotiq_base",
    joint_prim_names=["robotiq_finger_joint1", "robotiq_finger_joint2"],
    joint_opened_positions=np.array([0, 0]),
    joint_closed_positions=np.array([0.0185, 0.0185]),
    action_deltas=np.array([-0.0185, -0.0185]),
)

#define the manipulator
# my_ur5 = my_world.scene.add(Robot(prim_path="/World/sigmund_table/ur5e", name="ur5e_robot", position=[0.,0.,0.21234]))
# my_table = my_world.scene.add(RigidPrim(prim_path="/World/sigmund_table",position=[0.,0.,0.74]))
# my_ur5 = my_world.scene.add(SingleManipulator(prim_path="/World/ur5e", name="ur5e_robot", end_effector_prim_name="tool0", position=[0.,0.,0.74]))
my_ur5 = my_world.scene.add(SingleManipulator(prim_path="/World/ur5e", name="ur5e_robot", end_effector_prim_name="robotiq_base", gripper=gripper, position=[0.,0.,0.74]))
my_world.scene.add_default_ground_plane(z_position=-0.5)


# my_ur_rtde = ur_rtde.Ur5e_interface()
joint_default_positions = np.array([0.0,-1.5707,1.5707,-1.5707,-1.5707,0.0, 0.0185, 0.0185])
# joint_default_positions = my_ur_rtde.get_joint_positions() 


my_ur5.set_joints_default_state(positions=joint_default_positions)
my_articulation_controller = my_ur5.get_articulation_controller()

my_world.reset()
is_playing = 0
i = 0
target_joint_positions = joint_default_positions
# my_ur_rtde.set_teachmode()

while simulation_app.is_running():
    my_world.step(render=True)
    # target_joint_positions[2] = np.sin(i/100)*1.57/2.0 + 1.5707
    # print(target_joint_positions)
    # target_joint_positions = my_ur_rtde.get_joint_positions()
    # my_articulation_controller.apply_action(ArticulationAction(joint_positions=target_joint_positions))
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        i += 1
        gripper_positions = my_ur5.gripper.get_joint_positions()
        if i < 500:
            #close the gripper slowly
            my_ur5.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] + 0.0001, gripper_positions[1] + 0.0001]))
        if i > 500:
            #open the gripper slowly
            my_ur5.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] - 0.0001, gripper_positions[1] - 0.0001]))
        if i == 1000:
            i = 0



    if my_world.is_stopped():
        i = 0
        # my_ur_rtde.end_teachmode()
        # my_ur_rtde.shutdown_interface()
        my_world.reset()
        print("World has stopped")

simulation_app.close()

