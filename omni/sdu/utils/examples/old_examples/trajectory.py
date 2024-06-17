from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})
simulation_app.set_setting('persistent/app/viewport/displayOptions', 31983)

from omni.isaac.core.objects import cuboid
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
import numpy as np
import os
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from novo_sim.robots.ur5e import UR5E

from omni.isaac.core.utils import rotations as r
from omni.isaac.motion_generation import ArticulationTrajectory
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.prims import XFormPrim

NOVO_DIRECTORY =  os.getcwd() + "/novo_sim/"
rmp_config_dir = os.path.join(NOVO_DIRECTORY,"robots/ur5e_assets")

my_world = World(stage_units_in_meters=1.0)

ur5e = UR5E()
my_world.scene.add(ur5e)


my_world.reset()

task_space_position_targets = np.array([
    [0.4, -0.3, 0.2],
    [0.4, 0.3, 0.2],
    [0.4, 0.3, 0.6],
    [0.4, -0.3, 0.6],
    [0.4, -0.3, 0.2]
    ])
task_space_orientation_targets = np.tile(np.array([0,0,1,0]),(5,1))

ur5e._cart_space_trajectory_generator.set_c_space_velocity_limits(np.array([0.1,0.1,0.1,0.1,0.1,0.1]))
ur5e._cart_space_trajectory_generator.set_c_space_jerk_limits(np.ones(6)*0.8)
ur5e._cart_space_trajectory_generator.set_c_space_trajectory_generator_solver_param(param_name="min_time_span",param_val=1.0)
trajectory = ur5e._cart_space_trajectory_generator.compute_task_space_trajectory_from_points(
    positions= task_space_position_targets,orientations= task_space_orientation_targets, frame_name= "wrist_3_link")
physics_dt = 1/60.
articulation_trajectory = ArticulationTrajectory(ur5e,trajectory,physics_dt)
action_sequence = articulation_trajectory.get_action_sequence()
for i in range(len(task_space_position_targets)):
    add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", f"/target_{i}")
    frame = XFormPrim(f"/target_{i}",scale=[.04,.04,.04])
    position = task_space_position_targets[i]
    orientation = task_space_orientation_targets[i]
    frame.set_world_pose(position,orientation)


add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", f"/target_6")
frame = XFormPrim(f"/target_{6}",scale=[.04,.04,.04])

articulation_controller = ur5e.get_articulation_controller()
action_index = 0

print(action_sequence[0].joint_positions)
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()

        if action_index == 0:
            initial_state = action_sequence[0]
            joint_init=initial_state.joint_positions
            ur5e.set_joint_positions(joint_init)
            vel_init=initial_state.joint_positions
            # ur5e.set_joint_velocities(vel_init)
        pos = ur5e.compute_forward_kinematics()[0]
        frame.set_world_pose(position= pos, orientation=task_space_orientation_targets[0])
        # action_sequence[action_index].joint_positions[6:]=0
        # action_sequence[action_index].joint_velocities[6:]=0
        # articulation_controller.apply_action(action_sequence[action_index])
        ur5e.moveJ(action_sequence[action_index].joint_positions)

        action_index += 1
        action_index %= len(action_sequence)
            


simulation_app.close()