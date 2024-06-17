from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.motion_generation.lula import RmpFlow
from omni.isaac.motion_generation import ArticulationMotionPolicy
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import cuboid
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
import numpy as np
import os
import argparse
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper



NOVO_DIRECTORY =  os.getcwd() + "/novo_sim/"
rmp_config_dir = os.path.join(NOVO_DIRECTORY,"robots/ur5e_assets")



def add_robot(my_world):
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
    my_world.scene.add(manipulator)
    return gripper, manipulator


parser = argparse.ArgumentParser()
parser.add_argument("--urdf_path",type=str,default="ur5e.urdf")
parser.add_argument("--rmpflow_config_path",type=str,default="ur5e_rmpflow_config.yaml")
parser.add_argument("--end_effector_frame_name",type=str,default="tcp")
args = parser.parse_args()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
gripper, robot = add_robot(my_world)

#Initialize an RmpFlow object
rmpflow = RmpFlow(
    robot_description_path = os.path.join(rmp_config_dir,"ur5e_robot_description.yaml"),
    urdf_path = os.path.join(rmp_config_dir,args.urdf_path),
    rmpflow_config_path = os.path.join(rmp_config_dir,args.rmpflow_config_path),
    end_effector_frame_name = args.end_effector_frame_name, #This frame name must be present in the URDF
    maximum_substep_size = .0034
)

#Uncomment this line to visualize the collision spheres in the robot_description YAML file
#rmpflow.visualize_collision_spheres()
physics_dt = 1/60.
articulation_rmpflow = ArticulationMotionPolicy(robot,rmpflow,physics_dt)

articulation_controller = robot.get_articulation_controller()

#Make a target to follow
target_cube = cuboid.VisualCuboid("/World/target",position = np.array([0.492,0.133,0.4862-0.12]) , orientation=[0,1,0,0],color=np.array([1.,0,0]),size = .04)

#Make an obstacle to avoid
obstacle = cuboid.VisualCuboid("/World/obstacle",position = np.array([.8,0.15,.5]),color = np.array([0,1.,0]), size = .1)
rmpflow.add_obstacle(obstacle)


my_world.reset()
i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    i += 1
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        if i == 200:
            # pos = target_cube.get_world_pose()[0]
            # target_cube.set_world_pose(position=[pos[0] -1, pos[1], pos[2]])
            gripper.close()
        if i == 400:
            # pos = target_cube.get_world_pose()[0]
            # target_cube.set_world_pose(position=[pos[0] +1, pos[1], pos[2]])
            gripper.open()
            i=0

        #Set rmpflow target to be the current position of the target cube.
        rmpflow.set_end_effector_target(
            target_position=target_cube.get_world_pose()[0],
            # target_orientation=target_cube.get_world_pose()[1]
        )
        rmpflow.update_world()
        rmpflow.visualize_collision_spheres()

        actions = articulation_rmpflow.get_next_articulation_action()
        articulation_controller.apply_action(actions)
        # print(actions)

simulation_app.close()