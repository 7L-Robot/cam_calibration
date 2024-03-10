import os
import numpy as np
from scipy.spatial.transform import Rotation

import time

import pybullet
import pybullet_data
import pybullet_utils
# from pymp import Planner, toSE3
# from pinocchio.visualize import MeshcatVisualizer

from simulator import Camera, RigidObject
import utils

import cv2
from cv2 import aruco

from extrinsic import Extrinsic_Calibrator

class Robot():
    def __init__(self, robot_id, cam, eye_on_hand) -> None:
        joint_num = pybullet.getNumJoints(robot_id)

        joints = []
        names = []

        self.cam = cam

        self.eye_on_hand = eye_on_hand

        for j in range(joint_num):
            info = pybullet.getJointInfo(robot_id, j)
            joint_name = info[1]
            joint_type = info[2]

            if joint_type in [0, 1]:
                joints.append(j)
                names.append(joint_name)

        self.joints = joints
        self.names = names
        self.robot_id = robot_id
        self.joint_num = len(joints)

        self.init_joints = np.array([0, 0.2, 0.0, -2.62, 0.0, 2.94, 0.785, 0.04, 0.04])

        self.set_joints(self.init_joints)
    
    def get_joints(self):
        all_joints = pybullet.getJointStates(self.robot_id, self.joints)

        joint_poses = []
        for joint in all_joints:
            joint_poses.append(joint[0])

        return np.array(joint_poses)

    def close(self):
        self.move_to_joint([0,0], [9, 10], 30, 20)

    def open(self):
        self.move_to_joint([1,1], [9, 10], 30, 20)

    def follow_path(self, path):

        for p in path:
            self.move_to_joint(p)

    def move_to_joint(self, joints_values, joints_ids=None, max_force=60, loop_num=10):
        if joints_ids is None:
            joints_ids = self.joints
        
        for j in range( len(joints_ids) ):
            # pybullet.setJointMotorControl2(
            #     self.robot_id,
            #     joints_ids[j],
            #     pybullet.POSITION_CONTROL,
            #     joints_values[j],
            #     force = max_force)
            pass
        self.set_joints(joints_values, joints_ids)
        
        # for _ in range(loop_num):
        #     pybullet.stepSimulation()
            # time.sleep(1./240.)

        # while True:
            # s = np.sum( np.abs(self.get_joints() - np.array(joints)) ) 
            # if s < 1e-3:
            #     break

    def set_joints(self, joints_values, joints_ids=None):
        if joints_ids is None:
            joints_ids = self.joints

        for j in range( len(joints_ids) ):
            pybullet.resetJointState(
                self.robot_id,
                joints_ids[j],
                joints_values[j] )

    def move_to(self, pos, quat, start_joints=None, rrt_range=0.1):
        if start_joints is None:
            start_joints = self.get_joints()

        # plan_result = planner.plan_birrt([pos, quat], start_joints, rrt_range=rrt_range, seed=1024)
        ee_link = 8
        w, x,y,z = quat
        plan = pybullet.calculateInverseKinematics(self.robot_id, ee_link, pos, [x,y,z,w])
        
        # if 'position' not in plan_result:
        #     plan = []
        # else:
        if len(plan) > 0:
            # plan = plan_result["position"]  # [nq, N]
            # rest_num = bot.joint_num - plan.shape[1]
            # if rest_num > 0:
            #     rest_joints = start_joints[ -rest_num: ][None, :].repeat( len(plan) , axis=0)
            #     plan = np.concatenate( [plan, rest_joints], axis=1 )
            plan = [plan]

            plan = list(plan)

            # bot.follow_path(plan)
            bot.move_to_joint(plan[-1])

        # for _ in range(100):
        #     pybullet.stepSimulation()

        if self.eye_on_hand:
            link_state = pybullet.getLinkState(self.robot_id, 11)
            pos, quat = link_state[:2]
            rot = Rotation.from_quat(quat).as_matrix()
            self.cam.set_pose(pos, pos + rot[:3,2] * 0.1, rot[:3,0])

        self.cam.get_camera_image()
        

if __name__ == '__main__':

    eye_on_hand = True
    
    if eye_on_hand:
        urdf_path = "./franka_description/robots/panda_arm_cam.urdf"
    else:
        urdf_path = "./franka_description/robots/panda_arm_hand.urdf"

    srdf_path = "./franka_description/robots/panda_arm_hand.srdf"

    
    board_name = 'ArUco_DICT_7X7_50_5_7_50_10_0'
    # board_name = 'ArUco_DICT_7X7_50_4_4_40_10_0'

    # board_name = 'ChArUco_DICT_7X7_50_5_7_50_10_0'
    # board_name = "chessboard_8_10_30_15"
    

    physcisClient = pybullet.connect(pybullet.GUI)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 1)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeID = pybullet.loadURDF("plane.urdf")
    robot = pybullet.loadURDF(urdf_path, useFixedBase=True)

    box = pybullet.loadURDF("./franka_description/robots/box.urdf")
    
    vis_params, col_params, body_params = utils.get_param_template()
    col_params['fileName'] = vis_params['fileName'] = os.path.join('franka_description/board', board_name + '.obj')
    
    # board_obj.set_pose_from_pos_and_quat([0.47, 0, 0.01], [0, 0, 0, 1])
    if eye_on_hand:
        board_obj = RigidObject(board_name, vis_params, col_params, body_params)
        board_obj.set_pose_from_pos_and_quat([0.47, -0.1, 0.01], [0, 0, 0, 1])

    cam = Camera(640, 480, 57, 3, [1.5, -0.1, 0.4], [0,0,0.37], [-1, 0, 0], 0.055, 10)

    print(cam.pose[:3,3])


    bot = Robot(robot, cam, eye_on_hand)

    board_type = '_'.join(board_name.split('_')[1:4])
    board_col, board_row, length, sep, start_id = map(lambda x: int(x), board_name.split('_')[4:])
    

    cali = Extrinsic_Calibrator(board_type, board_col, board_row, 
                                board_grid_len=length / 1000,
                                board_grid_gap=sep / 1000,
                                real_grid_len=0.04, board_grid_start_id=start_id)


    x,y,z,w = Rotation.from_rotvec([0, np.pi/2, 0]).as_quat()
    q = [w,x,y,z]

    print("Press enter to quit")
    input()

    cam_in = np.loadtxt('outputs/camera_calibration/mtx.txt')
    cam_dist = np.loadtxt('outputs/camera_calibration/dist.txt')
    cam_dist = None


    y_step = 3
    z_step = 3
    count = 0

    ee_to_base_poses = []

    board_imgs = []

    for y in range(y_step):
        for z in range(z_step):
            
            x = 0.5
            
            angle_step = np.pi/15
            angle_step_2 = np.pi/15

            rot_step = 4
            rot_start = (rot_step - 1)/2

            angle_gap = np.pi/10
            angle_start = rot_start * angle_gap
            
            for k in range(rot_step):
                rv = np.random.rand()
                
                if eye_on_hand:                
                    if rv > 0.5:
                        q = (Rotation.from_rotvec([-angle_start + k * angle_gap, -angle_start + k * angle_gap, -angle_start + k * angle_gap]) * Rotation.from_quat([0, 1, 0, 0])).as_quat()
                    else:
                        q = (Rotation.from_rotvec([-angle_start  + k * angle_gap, -angle_start + k * angle_gap, angle_start - k * angle_gap]) * Rotation.from_quat([0, 1, 0, 0])).as_quat()
        
                    bot.move_to([ 0.235 + z * 0.1/z_step, -0.1 + y * 0.3/y_step, x ], q)

                else:
                    if rv > 0.5:
                        more_rot = Rotation.from_rotvec([-angle_start + k * angle_gap, -angle_start + k * angle_gap, -angle_start + k * angle_gap])
                    else:
                        more_rot = Rotation.from_rotvec([-angle_start + k * angle_gap, -angle_start + k * angle_gap, angle_start - k * angle_gap])
                    _x, _y, _z, _w = (more_rot * Rotation.from_rotvec([0, np.pi/2, 0])).as_quat()
                    
                    q = [_w,_x,_y,_z]

                    bot.move_to([ x, -0.2 + y * 0.4/y_step, 0.55 - z * 0.3/z_step ], q)

                link_state = pybullet.getLinkState(robot, 8)
                pos, quat = link_state[:2]
                mat = np.eye(4)
                mat[:3,:3] = Rotation.from_quat(quat).as_matrix()
                mat[:3,3] = pos

                rgba, depth, mask = cam.get_camera_image()

                bgr = rgba[..., 0:3][..., ::-1]
                # cv2.imwrite('./outputs/board/{:04d}.png'.format(count), bgr)
                
                count += 1

                board_imgs.append(bgr)
                ee_to_base_poses.append(mat)
        
    eye_to_hand_mat = cali.hand_eye_calibration(board_imgs, ee_to_base_poses, not eye_on_hand, cam_in, cam_dist)

    print(eye_to_hand_mat[:3,3])
    
    bot.open()

    input()
