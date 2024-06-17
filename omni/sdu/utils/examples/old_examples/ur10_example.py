from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.motion_generation.lula import LulaTaskSpaceTrajectoryGenerator
from omni.isaac.motion_generation import ArticulationTrajectory
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.prims import XFormPrim
import os
import numpy as np

my_world = World(stage_units_in_meters=1.0)

robot_prim_path = "/ur10"
usd_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
end_effector_name = "ee_link"

add_reference_to_stage(usd_path, robot_prim_path)
my_robot = Articulation(robot_prim_path)

my_world.reset()
my_robot.initialize()

# Lula config files for supported robots are stored in the motion_generation extension under "/motion_policy_configs"
mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
rmp_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")

#Initialize a LulaCSpaceTrajectoryGenerator object
task_space_trajectory_generator = LulaTaskSpaceTrajectoryGenerator(
    robot_description_path = rmp_config_dir + "/universal_robots/ur10/rmpflow/ur10_robot_description.yaml",
    urdf_path = rmp_config_dir + "/universal_robots/ur10/ur10_robot.urdf"
)

# Choose reachable position and orientation targets
task_space_position_targets = np.array([
    [0.3, -0.3, 0.1],
    [0.3, 0.3, 0.1],
    [0.3, 0.3, 0.5],
    [0.3, -0.3, 0.5],
    [0.3, -0.3, 0.1]
    ])
task_space_orientation_targets = np.tile(np.array([0,1,0,0]),(5,1))
trajectory = task_space_trajectory_generator.compute_task_space_trajectory_from_points(
    task_space_position_targets, task_space_orientation_targets, end_effector_name
)
if trajectory is None:
    print("No trajectory could be generated!")
    exit(0)

# Use the ArticulationTrajectory wrapper to run Trajectory on UR10 robot Articulation
# physics_dt = 1/(240.*4.)
physics_dt = 1/(60.)
articulation_trajectory = ArticulationTrajectory(my_robot,trajectory,physics_dt)

# Returns a list of ArticulationAction meant to be set on subsequent physics steps
action_sequence = articulation_trajectory.get_action_sequence()


#Create frames to visualize the task_space targets of the UR10
for i in range(len(task_space_position_targets)):
    add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", f"/target_{i}")
    frame = XFormPrim(f"/target_{i}",scale=[.04,.04,.04])
    position = task_space_position_targets[i]
    orientation = task_space_orientation_targets[i]
    frame.set_world_pose(position,orientation)
# https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/advanced_tutorials/tutorial_motion_generation_lula_trajectory_generator.html 

#Run the action sequence on a loop
articulation_controller = my_robot.get_articulation_controller()
action_index = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if action_index == 0:
            #Teleport robot to the initial state of the trajectory
            initial_state = action_sequence[0]
            my_robot.set_joint_positions(initial_state.joint_positions)
            my_robot.set_joint_velocities(initial_state.joint_velocities)

        articulation_controller.apply_action(action_sequence[action_index])

        action_index += 1
        action_index %= len(action_sequence)

simulation_app.close()