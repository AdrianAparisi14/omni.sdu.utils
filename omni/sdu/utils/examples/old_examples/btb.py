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
from novo_sim.grippers.grippers import CRG200

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
from spatialmath.base import trinterp
from spatialmath.spatialvector import SE3
from omni.isaac.core.utils.prims import get_prim_at_path
import omni.isaac.core.utils.extensions as extensions_utils
from omni.isaac.core.objects import DynamicCuboid



class Job():
    def __init__(self, steps, time_step, robot) -> None:
        self.steps = []
        self.t = time_step
        self.t_end 
        self.is_finised = 0

    def add_ctracj_job(self,start_pose, goal_pose, time_steps):
        self.steps.append(move_ctraj_job())
        

    def execute_step(self):
        step = self.steps[self.t]
        self.t += 1
        return     

    def reset(self):
        self.t = 0
        self.is_finised = 0


class move_ctraj_job():
    def __init__(self, start_pose, goal_pose, time_steps) -> None:
        self.steps = rtb.ctraj(SE3(start_pose), SE3(goal_pose), time_steps)
        self.t = 0
        self.final_step = time_steps

    def execute_step(self):
        step = self.steps[self.t]
        self.t += 1
        return step

    def is_done(self):
        if self.t == self.final_step:
            return True
        else:
            return False

    def reset(self):
        self.t = 0

class Action():
    def __init__(self):
        self._t = 0
        self.action_buffer = np.asarray([])
        self.default_joint_position = np.asarray([])

    def add_ctracj():
        pass

    def add_jtracj():
        pass

    def add_attact_fingertip():
        pass

    def add_detact_fingertip():
        pass

    def add_replace_fingertip():
        pass

    def add_pick_and_place():
        pass
    


extensions_utils.disable_extension(extension_name="omni.physx.flatcache")

NOVO_DIRECTORY =  os.getcwd() + "/novo_sim/"
rmp_config_dir = os.path.join(NOVO_DIRECTORY,"robots/ur5e_assets")
calibration_path = NOVO_DIRECTORY+"calibration_data/QuadArmB.csv"




my_world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene")
my_world.scene.add_default_ground_plane(z_position=-0.74)
physx_scene = get_prim_at_path("/physicsScene")
physx_scene.GetAttribute("physxScene:enableGPUDynamics").Set(True) #for some reason does not work
# physx_scene.GetAttribute("physics:gravityMagnitude").Set(0.098) #for some reason does not work


# add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/robotiq_gripper/Novo-Nordisk/physicsScene.usd",prim_path="/physicsScene")
add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/robotiq_gripper/Novo-Nordisk/ur5e_crp2.usd",prim_path="/World/ur5e")
add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/robotiq_gripper/Novo-Nordisk/tray4.usd",prim_path="/World/tray")
# add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/robotiq_gripper/Novo-Nordisk/fingertip.usd",prim_path="/World/fingertip_left")
# add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/robotiq_gripper/Novo-Nordisk/fingertip.usd",prim_path="/World/fingertip_right")
add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/LeftFinger.usd",prim_path="/World/fingertip_left")
add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/LeftFinger.usd",prim_path="/World/fingertip_right")
add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/sigmund_table2.usd",prim_path="/World/sigmund_table")
add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/FingerTipHolder.usd",prim_path="/World/FingerTipHolder")


# tray = GeometryPrim(prim_path="/World/tray",scale=[0.01,0.01,0.01],position=np.array([-0.5,0.8,0]))
tray = GeometryPrim(prim_path="/World/tray",position=np.array([-0.5,0.8,0]))
fingertipholder = XFormPrim(prim_path="/World/FingerTipHolder",position=np.array([-0.8,0.8,0]),orientation=r.euler_angles_to_quat(euler_angles=[0,0,-90],degrees=True))
# tray = XFormPrim(prim_path="/World/tray",position=np.array([0.5,0,0]))


cube = DynamicCuboid(
    prim_path="/World/random_cube",
    name="fancy_cube",
    position=np.array([-0.3, 0.8, 0.1]),
    scale=np.array([0.0515, 0.0515, 0.0515]),
    color=np.array([0, 0, 1.0]),
)

ur5e = UR5E(attact_flange=True,position=[-0.3,0.3,0], orientation=r.euler_angles_to_quat([0,0,126],degrees=True),end_effector_offset=[0,0,0.15],gripper=CRG200())

ut.move_prim("/World/fingertip_left","/World/ur5e/crp_200_gripper2/crg200_finger_left/fingertip_left")
ut.move_prim("/World/fingertip_right","/World/ur5e/crp_200_gripper2/crg200_finger_right/fingertip_right")


my_world.scene.add(ur5e)
my_world.reset()

tcp = ur5e.get_tcp()
r3 = tcp.rotation

add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", f"/target_begin")
# frame = XFormPrim(f"/target_begin",scale=[.07,.07,.07], position=tcp.translation + np.array([0,-0.2,0]),orientation=np.array([r3.w(),r3.x(),r3.y(),r3.z()]))
frame = XFormPrim(f"/target_begin",scale=[.07,.07,.07], position=tcp.translation,orientation=np.array([r3.w(),r3.x(),r3.y(),r3.z()]))


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
housing = XFormPrim("/World/tray/housing")
scale_drum_pos =scale_drum.get_world_pose()[0]
# scale_drum_pos =cube.get_world_pose()[0]
add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", f"/target_end")
# frame_end = XFormPrim(f"/target_end",scale=[.07,.07,.07], position=np.array([scale_drum_pos[0],scale_drum_pos[1],tcp.translation[2]]),orientation=np.array([r3.w(),r3.x(),r3.y(),r3.z()]))
frame_end = XFormPrim(f"/target_end",scale=[.07,.07,.07], position=np.array([scale_drum_pos[0],scale_drum_pos[1],tcp.translation[2]]),orientation=r.euler_angles_to_quat([180,0,0],degrees=True))
rot = r.euler_angles_to_quat([180,0,0],degrees=True)

joint_pos =ur5e.get_inverse_kinematics(ut.get_prim_pose3(frame))
ur5e.set_joint_positions(np.concatenate((joint_pos, np.array([0, 0])), axis=0))

robot = rtb.DHRobot([
    rtb.RevoluteDH(d=0.1625, alpha=np.pi / 2),
    rtb.RevoluteDH(a=-0.425),
    rtb.RevoluteDH(a=-0.3922),
    rtb.RevoluteDH(d=0.1333, alpha=np.pi / 2),
    rtb.RevoluteDH(d=0.0997, alpha=-np.pi / 2),
    rtb.RevoluteDH(d=0.0996)
],
name="UR5e")




# spec = lula.create_task_space_path_spec(tcp)
# spec.add_linear_path(ut.get_prim_pose3(frame_end))
# trajectory = ur5e._cart_space_trajectory_generator.compute_task_space_trajectory_from_path_spec(spec, "tcp")
# articulation_Trajectoy = ArticulationTrajectory(ur5e, trajectory, 1/6000.)
# action_sequence = articulation_Trajectoy.get_action_sequence()
# print(len(action_sequence))


i = 0
# action_index = 0
# steps = rtb.quintic(0,1, 1200).q
steps = rtb.ctraj(SE3(ut.get_prim_pose3(frame).matrix()), SE3(ut.get_prim_pose3(frame_end).matrix()), 600)
# steps = rtb.ctraj(SE3(ut.get_prim_pose3(frame).matrix()), SE3(ut.get_prim_pose3(frame_end).matrix()), 600)
steps2 = rtb.ctraj(SE3(ut.get_prim_pose3(frame_end).matrix()), SE3(ut.get_prim_pose3(cube).matrix()), 600)

job1 = move_ctraj_job(ut.get_prim_pose3(frame).matrix(), ut.get_prim_pose3(frame_end).matrix(), 400)
job2 = move_ctraj_job(ut.get_prim_pose3(frame_end).matrix(), ut.get_pose3(np.add(scale_drum.get_world_pose()[0], [0,0,0.055]),rot_quat=rot).matrix(), 400)
job3 = move_ctraj_job(ut.get_pose3(np.add(scale_drum.get_world_pose()[0], [0,0,0.055]),rot_quat=rot).matrix(), ut.get_pose3(np.add(scale_drum.get_world_pose()[0], [0,0,0.16]),rot_quat=rot).matrix(), 200)
job4 = move_ctraj_job(ut.get_pose3(np.add(scale_drum.get_world_pose()[0], [0,0,0.16]),rot_quat=rot).matrix(), ut.get_pose3(trans=np.asarray([housing.get_world_pose()[0][0],housing.get_world_pose()[0][1],scale_drum.get_world_pose()[0][2]+ 0.14]),rot_quat=rot).matrix(), 200)
job5 = move_ctraj_job(ut.get_pose3(trans=np.asarray([housing.get_world_pose()[0][0],housing.get_world_pose()[0][1],scale_drum.get_world_pose()[0][2]+ 0.14]),rot_quat=rot).matrix(),ut.get_pose3(trans=np.asarray([housing.get_world_pose()[0][0],housing.get_world_pose()[0][1],scale_drum.get_world_pose()[0][2]+0.1165]),rot_quat=rot).matrix() , 200)

job = [job1, job2, job3,job4,job5]
# ur5e.gripper.close()

test = [steps, steps2]
print(len(test))

tcp_sphere = sphere.VisualSphere("/World/path/tcp",position=tcp.translation,color=np.array([0.,1,0]),radius=0.008)

for i in range(60, len(steps) - 60, 60) : 
    sphere.VisualSphere("/World/path/step_" + str(i),position=steps[i].t,color=np.array([0.,1,0]),radius=0.008)
job_list = 0
last_pose = []
once = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            i = 0
            job_list = 0
            once = 0
            for j in job:
                j.reset()
            my_world.reset()
        # target_pose = ut.get_prim_pose3(frame)
        # ur5e.movePose3(target_pose)
            # pose = trinterp(ut.get_prim_pose3(frame).matrix(), ut.get_prim_pose3(frame_end).matrix(),steps[i])
            # rot = pose[:3,:3]
            # t = pose[:3,3]
        tcp_sphere.set_world_pose(position=ur5e.get_tcp().translation)

        if job[job_list].is_done():
            job_list += 1
            if job_list >= len(job):
                job_list -=1

        if job[job_list].is_done():
            ur5e.movePose3(ut.get_pose3(trans=last_pose.t,rot_mat=last_pose.R))
        else:
            current_pose = job[job_list].execute_step()
            last_pose = current_pose
            ur5e.movePose3(ut.get_pose3(trans=current_pose.t,rot_mat=current_pose.R))
        if job_list == 2 or job_list == 3:
            ur5e.close_gripper()
            # if once == 0:
            #     # ur5e.gripper.close()
            #     ur5e.close_gripper()
            #     once += 1
         

        if job_list == 4:
            if job[job_list].is_done():
                ur5e.gripper.open()
            else:
                ur5e.close_gripper()
    i += 1
simulation_app.close()