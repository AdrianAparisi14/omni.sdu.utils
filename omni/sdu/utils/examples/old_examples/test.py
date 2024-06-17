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
import lula

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


# The following code demonstrates how to specify a complicated task-space path
# using the lula.TaskSpacePathSpec object

#Lula has its own classes for Rotations and 6 DOF poses: Rotation3 and Pose3
r0 = lula.Rotation3(np.pi, np.array([1.0, 0.0, 0.0]))
t0 = np.array([.3,-.3,.3])
spec = lula.create_task_space_path_spec(lula.Pose3(r0,t0))

# Add path linearly interpolating between r0,r1 and t0,t1
t1 = np.array([.3,-.3,.5])
r1 = lula.Rotation3(np.pi/2,np.array([1,0,0]))
spec.add_linear_path(lula.Pose3(r1, t1))

# Add pure translation.  Constant rotation is assumed
spec.add_translation(t0)

# Add pure rotation.
spec.add_rotation(r0)

# Add three-point arc with constant orientation.
t2 = np.array([.3,.3,.3,])
midpoint = np.array([.3,0,.5])
spec.add_three_point_arc(t2, midpoint, constant_orientation = True)

# Add three-point arc with tangent orientation.
spec.add_three_point_arc(t0, midpoint, constant_orientation = False)

# Add three-point arc with orientation target.
spec.add_three_point_arc_with_orientation_target(lula.Pose3(r1, t2), midpoint)

# Add tangent arc with constant orientation. Tangent arcs are circles that connect two points
spec.add_tangent_arc(t0, constant_orientation = True)

# Add tangent arc with tangent orientation.
spec.add_tangent_arc(t2, constant_orientation=False)

# Add tangent arc with orientation target.
spec.add_tangent_arc_with_orientation_target(
    lula.Pose3(r0, t0))

trajectory = task_space_trajectory_generator.compute_task_space_trajectory_from_path_spec(
    spec, end_effector_name
)
if trajectory is None:
    print("No trajectory could be generated!")
    exit(0)

# Use the ArticulationTrajectory wrapper to run Trajectory on UR10 robot Articulation
physics_dt = 1/60.
articulation_trajectory = ArticulationTrajectory(my_robot,trajectory,physics_dt)

# Returns a list of ArticulationAction meant to be set on subsequent physics steps
action_sequence = articulation_trajectory.get_action_sequence()


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