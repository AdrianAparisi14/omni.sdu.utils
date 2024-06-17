# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Sequence
import numpy as np
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.manipulators.grippers.surface_gripper import SurfaceGripper
from omni.isaac.core.prims.xform_prim import XFormPrim 
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
import carb

from omni.isaac.core.utils import prims
from omni.isaac.core.utils.physics import set_rigid_body_enabled 
from omni.isaac.manipulators import SingleManipulator
import os
from omni.isaac.core.utils import transformations as tf
from omni.isaac.core.utils import rotations as r

from omni.isaac.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from omni.isaac.motion_generation.lula import LulaCSpaceTrajectoryGenerator
from omni.isaac.motion_generation import ArticulationTrajectory

from omni.isaac.motion_generation.lula import RRT
import lula

from omni.isaac.motion_generation.lula.trajectory_generator import (
    LulaCSpaceTrajectoryGenerator,
    LulaTaskSpaceTrajectoryGenerator,
)
from omni.isaac.core.utils.types import ArticulationAction
import argparse
import omni.kit.app
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.manipulators.grippers.gripper import Gripper
from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper
from omni.isaac.motion_generation.lula import RmpFlow

from omni.sdu.utils.controllers.rmpflow_controller import RMPFlowController
from omni.sdu.utils.utilities import utils as ut
from scipy.spatial.transform import Rotation as R

NOVO_NUCLEUS_ASSETS = "omniverse://i40-control-i.sandbox.tek.sdu.dk/Projects/novo/Assets/"
HEIGHT_OF_FLANGE = 0.035
INITIAL_JOINT_Q = np.array([0.0,-1.5707,1.5707,-1.5707,-1.5707,0.0])

class UR10(Robot):
    """[summary]

        Args:
            prim_path (str): [description]
            name (str, optional): [description]. Defaults to "ur10_robot".
            usd_path (Optional[str], optional): [description]. Defaults to None.
            position (Optional[np.ndarray], optional): [description]. Defaults to None.
            orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
            end_effector_prim_name (Optional[str], optional): [description]. Defaults to None.
            attach_gripper (bool, optional): [description]. Defaults to False.
            gripper_usd (Optional[str], optional): [description]. Defaults to "default".

        Raises:
            NotImplementedError: [description]
        """

    def __init__(
        self,
        prim_path: str = "/World/ur10e",
        end_effector_prim_name: str = "tool0",
        name: str = "ur10_robot",
        position: Optional[Sequence[float]] = None,
        usd_path: Optional[str] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = [1.0,1.0,1.0],
        gripper: Gripper = None,
        attact_flange: bool = False,
        calibration_path: str = None,
        initial_joint_q: Optional[Sequence[float]] = INITIAL_JOINT_Q,
        end_effector_offset: Optional[Sequence[float]] = [0,0,0]
    ) -> None:
        self._end_effector_prim_name = end_effector_prim_name
        self._end_effector = None
        self._initial_joint_q = initial_joint_q 
        self._end_effector_offset = np.array(end_effector_offset)
        prim = get_prim_at_path(prim_path)
        self._name = name
        if self._name is None:
            self._name = prim_path.split("/")[-1]
        if calibration_path is not None:
            self._calibrated = self.set_pose_calibrated(calibration_path)
        else:
            self._calibrated = False 
            self._world_position = position
            self._world_orientaion = orientation
        if attact_flange: 
            self.attact_flange("flange_"+self._name)
        if not prim.IsValid():
            if usd_path:
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            else:
                assets_root_path = get_assets_root_path()
                if assets_root_path is None:
                    carb.log_error("Could not find Isaac Sim assets folder")
                    return
                usd_path = assets_root_path + "/Isaac/Robots/UR10/ur10.usd"
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            if self._end_effector_prim_name is None:
                self._end_effector_prim_path = prim_path + "/tool0"
            else:
                self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
        
        if gripper is not None: 
            self._gripper = gripper(robot_prim_path = prim_path)
            self._end_effector_prim_path = self._gripper._end_effector_prim_path
            if np.all(self._end_effector_offset == 0):
                self._end_effector_offset = self._gripper._end_effector_offset
        else:
            self._gripper = gripper
        
        Articulation.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=self._world_position,
            translation=None,
            orientation=self._world_orientaion,
            scale=scale,
            visible=None,
            articulation_controller=None,
        )
        return

    def attact_flange(self,name):
        """Attaces a flange to the base of the robot

        Args:
            name (str): The name of the desired robot 
        """
        prim_path = "/World/"+name
        add_reference_to_stage(usd_path=NOVO_NUCLEUS_ASSETS +"Flange/flange.usd", prim_path=prim_path)
        if self._world_position is None:
            self._world_position = np.zeros(3) 
        if self._calibrated:
            pose = self._world_position
            XFormPrim(prim_path=prim_path, name=name, position=[pose[0],pose[1],pose[2] - HEIGHT_OF_FLANGE] )
        else:
            XFormPrim(prim_path=prim_path, name=name, position=self._world_position)
            self._world_position[2] += HEIGHT_OF_FLANGE

    @property
    def end_effector(self) -> RigidPrim:
        """
        Returns:
            RigidPrim: end effector of the manipulator (can be used to get its world pose, angular velocity..etc).
        """
        return self._end_effector

    def set_pose_calibrated(self, calibration_path):
        """Set the position of robot to the calibrated pose in world frame.

        Args:
            calibration_path (str): path to calibration file. File must contain a pose 

        Returns:
            bool: False if path does not exist
        """
        if os.path.exists(calibration_path):
            pose = np.genfromtxt(calibration_path,delimiter=",")
            T_r_z = [[-1, 0 ,0 ,0], [0, -1, 0, 0],[0,0,1,0],[0,0,0,1]]
            T_result =np.matmul(pose,T_r_z)
            pose = tf.pose_from_tf_matrix(T_result)
            self._world_position = pose[0]
            self._world_orientaion = pose[1]
            return True
        return False

    def init_motion_control(self):
        rmp_config_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"ur10e_assets")
        robot_description_path = os.path.join(rmp_config_dir,"ur10e_robot_description.yaml")
        urdf_path = os.path.join(rmp_config_dir,"ur10e.urdf")

        self._kinematics = LulaKinematicsSolver(robot_description_path=robot_description_path,
                                                urdf_path=urdf_path)
        world_pose = self.get_world_pose()
        self._kinematics.set_robot_base_pose(robot_position=world_pose[0],robot_orientation=world_pose[1])

        # self._joint_space_trajectory_generator = LulaCSpaceTrajectoryGenerator(robot_description_path=robot_description_path,urdf_path=urdf_path)
        # self._cart_space_trajectory_generator = LulaTaskSpaceTrajectoryGenerator(robot_description_path=robot_description_path,urdf_path=urdf_path)
        # self._cart_space_trajectory_generator._kinematics_solver.set_robot_base_pose(robot_position=world_pose[0],robot_orientation=world_pose[1])

        # self._rmpflow = RmpFlow(
        #     robot_description_path = os.path.join(rmp_config_dir,"ur5e_robot_description.yaml"),
        #     urdf_path = os.path.join(rmp_config_dir,"ur5e.urdf"),
        #     rmpflow_config_path = os.path.join(rmp_config_dir,"ur5e_rmpflow_config.yaml"),
        #     end_effector_frame_name = "tcp", #This frame name must be present in the URDF
        #     maximum_substep_size = .0034)

        # self._cspace_controller=RMPFlowController(
        #         name=self._name + "_cspace_controller", robot_articulation=self, rmp_flow=self._rmpflow)
        
        # self._RRTplanner = RRT(
        #     robot_description_path =robot_description_path,
        #     urdf_path = urdf_path,
        #     rrt_config_path = os.path.join(rmp_config_dir,"ur5e_planner_config.yaml"),
        #     end_effector_frame_name = "tcp"
        # )

    def get_tcp(self) -> lula.Pose3:
        """Compute tcp pose with end_effector_offset

        Returns:
            lula.Pose3: 
        """
        #TODO: Make it generic for any joint robot with or without gripper 
            #Assumes 6 joint robot
        tcp = self._kinematics.compute_forward_kinematics(frame_name="tcp", joint_positions=self.get_joint_positions()[:6])
        offset = lula.Pose3.from_translation(self._end_effector_offset)
        flange = lula.Pose3(lula.Rotation3(np.array(tcp[1])) , np.array(tcp[0]))
        tcp = flange * offset 
        return tcp 

    def get_forward_kinematics(self, joint_positions: Optional[Sequence[float]] = None) -> lula.Pose3:
        """Compute tcp pose with end_effector_offset

        Returns:
            lula.Pose3: 
        """
        #TODO: Make it generic for any joint robot with or without gripper 
            #Assumes 6 joint robot
        tcp = self._kinematics.compute_forward_kinematics(frame_name="tcp", joint_positions=joint_positions)
        offset = lula.Pose3.from_translation(self._end_effector_offset)
        flange = lula.Pose3(lula.Rotation3(np.array(tcp[1])) , np.array(tcp[0]))
        tcp = flange * offset 
        return tcp 

    def get_inverse_kinematics(self, target_pose = lula.Pose3, seed: Optional[Sequence[float]] = None):
        """Compute the joint positions to a target pose

        Args:
            target_pose (lula.Pose3):  

        Returns:
            np.array(): Target joint position 
        """
        #TODO: Make it generic for any joint robot with or without gripper 
            #Assumes 6 joint robot
        offset_pose = lula.Pose3.from_translation(np.array(self._end_effector_offset))
        goal = target_pose * offset_pose.inverse()
        if seed is None:
            seed_joint_pos = self.get_joint_positions()[:6]
        else:
            seed_joint_pos = seed 
        target_joint_position, success = self._kinematics.compute_inverse_kinematics(frame_name ="tcp",warm_start=seed_joint_pos, 
                                                                                     target_position=goal.translation, 
                                                                                     target_orientation=ut.pose3_to_quat(goal))
        if success == False:
            carb.log_error("Inverse Kinematics solver could not find a solution to tagert pose")
            return seed_joint_pos 
        else:
            return np.array(target_joint_position)


    def moveL(self, target_position: Optional[Sequence[float]] = None, target_orientation: Optional[Sequence[float]] = None):
        #TODO: Should be removed
        action = self._cspace_controller.forward(target_end_effector_position=target_position, target_end_effector_orientation=target_orientation)
        self._articulation_controller.apply_action(action)
        return

    def QtoQplanner(self, start, end):
        pass

    def close_gripper(self):
        """Simple close gripper function
            Tries to keep the object centered
        """
        #TODO: Make it generic for any joint robot 
            #Assumes 6 joint robot
        current_positions = self._gripper.get_joint_positions()
        closed_positions = self._gripper._joint_closed_positions
        error = closed_positions - current_positions 
        position_error = (error[0] - error[1])*0.2
        target_acceleration = closed_positions - self._gripper._gain*(error) + np.array([position_error, -position_error])
        target = np.concatenate((np.array([None, None, None, None, None, None]), target_acceleration), axis=0)
        action = ArticulationAction(joint_positions=target)
        self._gripper._articulation_apply_action_func(action)
        
    @property
    def gripper(self) -> Gripper:
        """
        Returns:
            Gripper: gripper of the manipulator (can be used to open or close the gripper, get its world pose or angular velocity..etc).
        """
        return self._gripper
    
    def print_state(self):
        print("test")

    def get_initial_joint_position(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        if self.gripper is None:
            initial_q = np.append(self._initial_joint_q, np.zeros(self.num_dof-6)) 
        else:
            initial_q = self._initial_joint_q
        return initial_q 
    
    def movePose3(self, target_pose = lula.Pose3):
        """Solving IK problem for target pose. Applies joints articulation

        Args:
            target_pose (lula.Pose3): _description_
        Return:
        """
        joint_pos = self.get_inverse_kinematics(target_pose)
        if self.gripper is not None:
            #TODO: Assumes parallel gripper. Must be more generic
            joint_pos = np.concatenate((joint_pos, np.array([None,None])),axis=0)
        self.apply_articulationAction(joint_positions=joint_pos)
        return


    def moveJ(self, joint_q):
        """Move robot to desired joint position

        Args:
            joint_q (array): desired joint position 
        Return:
        """
        if self.gripper is not None:
            #TODO: Assumes parallel gripper. Must be more generic
            joint_q = np.concatenate((joint_q, np.array([None,None])),axis=0)
        self.apply_articulationAction(joint_positions=joint_q)
        return

    def apply_articulationAction(self, joint_positions:Optional[Sequence[float]] = None, joint_velocities:Optional[Sequence[float]] = None,joint_efforts:Optional[Sequence[float]] = None,joint_indices: Optional[Sequence[float]] = None ):
        action = ArticulationAction(
            joint_positions=joint_positions,
            joint_velocities=joint_velocities,
            joint_efforts=joint_efforts,
            joint_indices=joint_indices)
        self._articulation_controller.apply_action(action)

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates an articulation view using physX tensor api.
            This needs to be called after each hard reset (i.e stop + play on the timeline) before interacting with any
            of the functions of this class.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.
        """
        Articulation.initialize(self, physics_sim_view=physics_sim_view)
        # end_effector_prim_path = self.prim_path + "/" + self._end_effector_prim_name
        self._end_effector = RigidPrim(prim_path=self._end_effector_prim_path, name=self.name + "_end_effector")
        self._end_effector.initialize(physics_sim_view)
        if isinstance(self._gripper, ParallelGripper):
            self._gripper.initialize(
                physics_sim_view=physics_sim_view,
                articulation_apply_action_func=self.apply_action,
                get_joint_positions_func=self.get_joint_positions,
                set_joint_positions_func=self.set_joint_positions,
                dof_names=self.dof_names,
            )
        if isinstance(self._gripper, SurfaceGripper):
            self._gripper.initialize(physics_sim_view=physics_sim_view, articulation_num_dofs=self.num_dof)
        initial_state = np.append(self._initial_joint_q , np.zeros(self.num_dof-6))
        self.set_joints_default_state(positions=initial_state)
        self.init_motion_control()
        return

    def post_reset(self) -> None:
        """Resets the manipulator, the end effector and the gripper to its default state.
        """
        Articulation.post_reset(self)
        self._end_effector.post_reset()
        if self._gripper is not None:
            self._gripper.post_reset()
        return