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
from omni.isaac.core.objects import DynamicCuboid
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




assets_root_path = get_assets_root_path()
if assets_root_path is None:
    # Use carb to log warnings, errors and infos in your application (shown on terminal)
    carb.log_error("Could not find nucleus server with /Isaac folder")


my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

cam = Camera(prim_path="/World/camera", name="camera", position=[-1.0, 0, 0.4],frequency=1)
cam.set_focal_length(1.5)
cam.set_clipping_range(0.00001)
my_world.scene.add(cam)
wall_light = DynamicCuboid(prim_path="/World/wall_light",name="wall_light",position=[0.,0.,0.43604] ,scale=[0.03969,1.88056,1.4187])
my_world.scene.add(wall_light)

my_world.reset()




cam  = my_world.scene.get_object("camera")
print(np.shape(cam.get_current_frame()['rgba']))
print(cam.get_intrinsics_matrix())
print(cam.get_world_pose())


while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()

simulation_app.close()