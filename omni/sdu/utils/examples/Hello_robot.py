from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
from omni.isaac.core import World
import omni.isaac.core.utils.extensions as extensions_utils
import numpy as np
from omni.isaac.core.utils.stage import add_reference_to_stage
from novo_sim.robots.ur5e import UR5E
from omni.isaac.core.utils.nucleus import get_assets_root_path
from novo_sim.grippers.grippers import CRG200, ROBOTIQ, attach_gripper_to_robot

from novo_sim.robots.ur10 import UR10

from omni.isaac.manipulators import SingleManipulator
import os
from omni.isaac.core.utils import rotations as r
from novo_sim.utilities import utils as ut
from novo_sim.tasks.tasks import Tasks
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.objects import DynamicCuboid

extensions_utils.disable_extension(extension_name="omni.physx.flatcache")
NOVO_DIRECTORY =  os.getcwd() + "/novo_sim/"
NOVO_NUCLEUS_ASSETS = "omniverse://i40-control-i.sandbox.tek.sdu.dk/Projects/novo/Assets/"

my_world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene")
my_world.scene.add_default_ground_plane()
physx_scene = get_prim_at_path("/physicsScene")
physx_scene.GetAttribute("physxScene:enableGPUDynamics").Set(True)

add_reference_to_stage(usd_path= NOVO_NUCLEUS_ASSETS + "Fingertips/test/LeftFinger.usd",prim_path="/World/fingertip_left")
add_reference_to_stage(usd_path= NOVO_NUCLEUS_ASSETS + "Fingertips/test/LeftFinger.usd",prim_path="/World/fingertip_right")

ur5e = UR5E(prim_path="/World/ur5e",
            name="ur5e",
            attact_flange=True,
            position=[-0.3,0.3,0], 
            orientation=r.euler_angles_to_quat([0,0,126],degrees=True),
            gripper=ROBOTIQ)

ur5e2 = UR5E(prim_path="/World/ur5e2", 
             name="ur5e2", 
             gripper=CRG200)

ur10_usd_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
add_reference_to_stage(usd_path=ur10_usd_path, prim_path="/World/ur10")
ur10 = UR10(prim_path="/World/ur10",name="ur10",end_effector_prim_name="/tool0",position=[1,0,0])
attach_gripper_to_robot(ur10, "/World/ur10",ROBOTIQ)
ur10.set_joints_default_state(np.array([0.0,-1.5707,1.5707,-1.5707,-1.5707,0.0,0,0]))


ur10_usd_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
add_reference_to_stage(usd_path=ur10_usd_path, prim_path="/World/ur102")
ur102 = UR10(prim_path="/World/ur102",name="ur102",end_effector_prim_name="/tool0",position=[1,1,0])
attach_gripper_to_robot(ur102, "/World/ur102",CRG200)
ur102.set_joints_default_state(np.array([0.0,-1.5707,1.5707,-1.5707,-1.5707,0.0,0,0]))

cube = DynamicCuboid(
    prim_path="/World/random_cube",
    name="fancy_cube",
    position=np.array([-0.3, 0.8, 0.05]),
    # scale=np.array([0.0515, 0.0515, 0.0515]),
    scale=np.array([0.0215, 0.0215, 0.0215]),
    orientation=r.euler_angles_to_quat([180,0,0],degrees=True),
    color=np.array([0, 0, 1.0]),
    mass=0.2,
)
cube2 = DynamicCuboid(
    prim_path="/World/random_cube2",
    name="fancy_cube2",
    position=np.array([0.3, 0.8, 0.05]),
    scale=np.array([0.0515, 0.0515, 0.0515]),
    orientation=r.euler_angles_to_quat([180,0,0],degrees=True),
    color=np.array([1.0, 0, 0]),
    mass=0.2,
)

ut.move_prim("/World/fingertip_left",ur5e._gripper.left_finger_prim_path + "/fingertip_left")
ut.move_prim("/World/fingertip_right",ur5e._gripper.right_finger_prim_path + "/fingertip_right")

my_world.scene.add(ur5e)
my_world.scene.add(ur5e2)
my_world.scene.add(ur10)
my_world.scene.add(ur102)
my_world.reset()

rot = r.euler_angles_to_quat([180,0,0],degrees=True)
rot2 = r.euler_angles_to_quat([180,0,-90],degrees=True)

cube_p = cube.get_world_pose()[0]
cube_p2 = cube2.get_world_pose()[0]

tasks = Tasks(ur5e)
tasks.add_jtraj(desired_pose = ut.get_pose3(np.add(cube_p, [0,0,0.1]),rot_quat=rot), time_step=60)
tasks.add_ctraj(desired_pose = ut.get_pose3(np.add(cube_p, [0,0,-0.01]),rot_quat=rot),time_step=60)
tasks.close_gripper()
tasks.add_ctraj(desired_pose = ut.get_pose3(np.add(cube_p, [0,0,0.15]),rot_quat=rot),time_step=200)
tasks.add_jtraj(desired_pose = ut.get_pose3(np.add(cube_p2, [0,0,0.15]),rot_quat=rot2),time_step=100)
tasks.add_ctraj(desired_pose = ut.get_pose3(np.add(cube_p2, [0,0,0.054]),rot_quat=rot2),time_step=100)
tasks.open_gripper()
tasks.add_ctraj(desired_pose = ut.get_pose3(np.add(cube_p2, [0,0,0.2]),rot_quat=rot2),time_step=100)

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            tasks.reset()
            my_world.reset()
        tasks.next_step()

simulation_app.close()