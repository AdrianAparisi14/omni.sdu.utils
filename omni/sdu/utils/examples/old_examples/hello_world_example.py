#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

my_world = World()

my_world.scene.add_default_ground_plane()
fancy_cube =  my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube",
        name="fancy_cube",
        position=np.array([0, 0, 1.0]),
        scale=np.array([0.5015, 0.5015, 0.5015]),
        color=np.array([0, 0, 1.0]),
    ))
# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly

my_world.reset()

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        position, orientation = fancy_cube.get_world_pose()
        linear_velocity = fancy_cube.get_linear_velocity()
        # will be shown on terminal
        print("Cube position is : " + str(position))
        # print("Cube's orientation is : " + str(orientation))
        # print("Cube's linear velocity is : " + str(linear_velocity))
simulation_app.close() # Close Isaac sim
