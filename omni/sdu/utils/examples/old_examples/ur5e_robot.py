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
from omni.isaac.core.utils import transformations as tf
from omni.isaac.motion_generation import ArticulationTrajectory
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.prims import XFormPrim
from roboticstoolbox.tools.trajectory import jtraj

import roboticstoolbox as rtb
import Lula


NOVO_DIRECTORY =  os.getcwd() + "/novo_sim/"
rmp_config_dir = os.path.join(NOVO_DIRECTORY,"robots/ur5e_assets")


my_world = World(stage_units_in_meters=1.0)
# my_world.scene.add_default_ground_plane()

add_reference_to_stage(usd_path= NOVO_DIRECTORY + "omniverse/robotiq_gripper/Novo-Nordisk/ur5e_crp200.usd",prim_path="/World/ur5e")
calibration_path = NOVO_DIRECTORY+"calibration_data/QuadArmB.csv"
ur5e = UR5E(attact_flange=True, end_effector_offset=[0,0,0.12])
my_world.scene.add(ur5e)


my_world.reset()

init_joints = ur5e.get_joint_positions()
tcp = ur5e.compute_forward_kinematics()
# tcp_pose = tf.tf_matrix_from_pose(tcp[0], tcp[1])
tcp_t = tcp[0]
# tcp_pose_end = tf.tf_matrix_from_pose([tcp_t[0] + 0.3,tcp_t[1],tcp_t[2]], tcp[1])
tcp_end = [tcp_t[0] ,tcp_t[1] + 0.2,tcp_t[2]]

add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", f"/target_end")
frame = XFormPrim(f"/target_end",scale=[.04,.04,.04], position=tcp_end ,orientation=r.euler_angles_to_quat(r.matrix_to_euler_angles(tcp[1])))

end_joints, succ = ur5e._kinematics.compute_inverse_kinematics(frame_name= "tcp", target_position=np.array(tcp_end), target_orientation=r.euler_angles_to_quat(r.matrix_to_euler_angles(tcp[1])))

end_joints = np.array(end_joints)
end_joints=  np.append(end_joints, 0)
end_joints=  np.append(end_joints, 0)


tg = jtraj(init_joints, end_joints, 1000)


add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", f"/target_6")
frame = XFormPrim(f"/target_{6}",scale=[.04,.04,.04], position=tcp[0] ,orientation=r.euler_angles_to_quat(r.matrix_to_euler_angles(tcp[1])))

task_space_position_targets = np.array([
    [0.3, -0.3, 0.1],
    [0.3, 0.3, 0.1],
    [0.3, 0.3, 0.5],
    [0.3, -0.3, 0.5],
    [0.3, -0.3, 0.1]
    ])
task_space_orientation_targets = np.tile(np.array([0,1,0,0]),(5,1))






for i in range(len(task_space_position_targets)):
    add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", f"/target_{i}")
    frame = XFormPrim(f"/target_{i}",scale=[.04,.04,.04])
    position = task_space_position_targets[i]
    orientation = task_space_orientation_targets[i]
    frame.set_world_pose(position,orientation)

add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", f"/target_6")
frame = XFormPrim(f"/target_{6}",scale=[.04,.04,.04], position=tcp[0] ,orientation=r.euler_angles_to_quat(r.matrix_to_euler_angles(tcp[1])))


i = 0

action = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
    if i > 100:
        tcp = ur5e.compute_forward_kinematics()
        frame.set_world_pose(tcp[0], r.euler_angles_to_quat(r.matrix_to_euler_angles(tcp[1])))
        ur5e.apply_action(joint_positions=tg.q[action])
        print(tg.q[action])
        # ur5e.apply_action(joint_positions=tg.q(action), joint_velocities=tg.qd(action), joint_efforts=tg.qdd(action))
        action += 1
        action %= 1000
    i += 1
        
        

            


simulation_app.close()