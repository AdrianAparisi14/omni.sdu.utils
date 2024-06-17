from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})
# simulation_app.set_setting('persistent/app/viewport/displayOptions', 31983)


from omni.isaac.core.objects import cuboid
from omni.isaac.core import World

from omni.isaac.core.utils.stage import open_stage
import numpy as np

from omni.isaac.core.objects import sphere 
import os
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from novo_sim.robots.ur5e import UR5E
from novo_sim.grippers.grippers import CRG200, ROBOTIQ


from omni.isaac.core.utils import rotations as r
from omni.isaac.core.utils import transformations as tf
from omni.isaac.motion_generation import ArticulationTrajectory
from omni.isaac.core.utils.nucleus import get_assets_root_path 
from omni.isaac.core.prims import XFormPrim, GeometryPrim
from roboticstoolbox.tools.trajectory import jtraj

import roboticstoolbox as rtb
import lula
# from omni.isaac.motion_generation.lula import utils 
from novo_sim.utilities import utils as ut
from novo_sim.tasks.tasks import Tasks
from spatialmath.base import trinterp
from spatialmath.spatialvector import SE3
from omni.isaac.core.utils.prims import get_prim_at_path
import omni.isaac.core.utils.extensions as extensions_utils
from omni.isaac.core.objects import DynamicCuboid

    
extensions_utils.disable_extension(extension_name="omni.physx.flatcache")

NOVO_DIRECTORY =  os.getcwd() + "/novo_sim/"
NOVO_NUCLEUS_ASSETS = "omniverse://i40-control-i.sandbox.tek.sdu.dk/Projects/novo/Assets/"
rmp_config_dir = os.path.join(NOVO_DIRECTORY,"robots/ur5e_assets")
calibration_path = NOVO_DIRECTORY+"calibration_data/QuadArmB.csv"



my_world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene")
my_world.scene.add_default_ground_plane(z_position=-0.74)
physx_scene = get_prim_at_path("/physicsScene")
physx_scene.GetAttribute("physxScene:enableGPUDynamics").Set(True) #for some reason does not work
# physx_scene.GetAttribute("physics:gravityMagnitude").Set(0.098) #for some reason does not work

# ur5e2 = UR5E(prim_path="/World/ur5e2",
#              name="ur5e2", 
#              usd_path= NOVO_NUCLEUS_ASSETS + "Robots/ur5e/ur5e.usd")

add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/robotiq_gripper/Novo-Nordisk/tray4.usd",prim_path="/World/tray")
add_reference_to_stage(usd_path= NOVO_NUCLEUS_ASSETS + "Fingertips/test/LeftFinger.usd",prim_path="/World/fingertip_left")
add_reference_to_stage(usd_path= NOVO_NUCLEUS_ASSETS + "Fingertips/test/LeftFinger.usd",prim_path="/World/fingertip_right")
add_reference_to_stage(usd_path= NOVO_NUCLEUS_ASSETS + "Siegmund table/Combined_Siegmund_tables.usd",prim_path="/World/sigmund_table")
add_reference_to_stage(usd_path= NOVO_NUCLEUS_ASSETS + "Fingertip holder/Technicon_fixture.usd",prim_path="/World/FingerTipHolder")

tray = GeometryPrim(prim_path="/World/tray",position=np.array([-0.5,0.8,0]))
fingertipholder = XFormPrim(prim_path="/World/FingerTipHolder",position=np.array([0.8,0.8,-0.02334]),orientation=r.euler_angles_to_quat(euler_angles=[0,0,0],degrees=True))

ur5e = UR5E(attact_flange=True,
            position=[-0.3,0.3,0], 
            orientation=r.euler_angles_to_quat([0,0,126],degrees=True),
            gripper=ROBOTIQ)


ut.move_prim("/World/fingertip_left",ur5e._gripper.left_finger_prim_path + "/fingertip_left")
ut.move_prim("/World/fingertip_right",ur5e._gripper.right_finger_prim_path + "/fingertip_right")


my_world.scene.add(ur5e)
my_world.reset()

cube = DynamicCuboid(
    prim_path="/World/random_cube",
    name="fancy_cube",
    position=np.array([-0.3, 0.8, 0.05]),
    scale=np.array([0.0515, 0.0515, 0.0515]),
    orientation=r.euler_angles_to_quat([180,0,0],degrees=True),
    color=np.array([0, 0, 1.0]),
    mass=0.2,
)

scale_drum = XFormPrim("/World/tray/scale_drum")
housing_pos = XFormPrim("/World/tray/housing").get_world_pose()[0]
scale_drum_pos = scale_drum.get_world_pose()[0]
rot = r.euler_angles_to_quat([180,0,0],degrees=True)
rot2 = r.euler_angles_to_quat([180,0,-10],degrees=True)
rot3 = r.euler_angles_to_quat([180,0, 10],degrees=True)

tasks = Tasks(ur5e)
tasks.add_jtraj(desired_pose = ut.get_pose3(np.add(scale_drum_pos, [0,0,0.1]),rot_quat=rot), time_step=20)
tasks.add_ctraj(desired_pose = ut.get_pose3(np.add(scale_drum_pos, [0,0,0.055]),rot_quat=rot),time_step=30)
tasks.close_gripper()
tasks.add_ctraj(desired_pose = ut.get_pose3(np.add(scale_drum_pos, [0,0,0.15]),rot_quat=rot),time_step=200)
tasks.add_jtraj(desired_pose = ut.get_pose3(np.add(housing_pos, [0,0,0.12]),rot_quat=rot),time_step=40)
tasks.add_ctraj(desired_pose = ut.get_pose3(np.add(housing_pos, [0,0.002,0.101]),rot_quat=rot))
tasks.open_gripper()

tcp_sphere = sphere.VisualSphere("/World/path/tcp",color=np.array([0.,1,0]),radius=0.008)

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            tasks.reset()
            my_world.reset()

        tcp_sphere.set_world_pose(position=ur5e.get_tcp().translation)
        tasks.next_step()

simulation_app.close()