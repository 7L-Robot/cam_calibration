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
from hand_eye_calibration_sim_test import detect_markers, rodrigues_to_rotation_matrix, pre_selection, dual_quaternion_method

from utils import ARUCO_DICT

class Robot():
    def __init__(self, robot_id, cam) -> None:
        joint_num = pybullet.getNumJoints(robot_id)

        joints = []
        names = []

        self.cam = cam

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
            pybullet.setJointMotorControl2(
                self.robot_id,
                joints_ids[j],
                pybullet.POSITION_CONTROL,
                joints_values[j],
                force = max_force)
        
        for _ in range(loop_num):
            pybullet.stepSimulation()
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

            if viz is not None:
                # viz.play(np.transpose(plan), dt=0.1)
                viz.play(plan, dt=0.1)

            plan = list(plan)

            bot.follow_path(plan)

        for _ in range(30):
            pybullet.stepSimulation()

        link_state = pybullet.getLinkState(self.robot_id, 11)
        pos, quat = link_state[:2]
        rot = Rotation.from_quat(quat).as_matrix()
        
        self.cam.set_pose(pos, pos + rot[:3,2] * 0.1, rot[:3,0])
        self.cam.get_camera_image()
        

if __name__ == '__main__':
    urdf_path = "./franka_description/robots/panda_arm_cam.urdf"
    srdf_path = "./franka_description/robots/panda_arm_hand.srdf"

    
    board_name = 'ArUco_DICT_7X7_50_5_7_50_10_0'
    # board_name = 'ChArUco_DICT_7X7_50_5_7_50_10_0'
    # board_name = "chessboard_8_10_30_15"
    

    physcisClient = pybullet.connect(pybullet.GUI)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 1)

    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    planeID = pybullet.loadURDF("plane.urdf")
    robot = pybullet.loadURDF(urdf_path, useFixedBase=True)

    box = pybullet.loadURDF("./franka_description/robots/box.urdf")
    
    vis_params, col_params, body_params = utils.get_param_template()
    col_params['fileName'] = vis_params['fileName'] = os.path.join('assets', board_name + '.obj')
    board_obj = RigidObject(board_name, vis_params, col_params, body_params)
    # board_obj.set_pose_from_pos_and_quat([0.37, 0, 0.01], [0, 0, 0, 1])
    board_obj.set_pose_from_pos_and_quat([0.47, 0, 0.01], [0, 0, 0, 1])
    board_obj.set_pose_from_pos_and_quat([0.47, -0.1, 0.01], [0, 0, 0, 1])

    cam = Camera(640, 480, 57, 3, [1.2, 0, 0.65], [0,0,0.3], [-1, 0, 0], 0.105, 10)

    plan = []

    bot = Robot(robot, cam)

    # planner = Planner(
    #     urdf_path,
    #     None,
    #     srdf=srdf_path,
    #     use_convex=True,
    #     ee_link_name="panda_hand",
    #     timestep=0.1,
    #     joint_vel_limits=1,
    #     joint_acc_limits=1,
    # )

    # Initialize visualizer


    # Initialize environment
    # planner.scene.addBox([0.04, 0.04, 0.12], toSE3([0.7, 0, 0.06]), name="box")

    # planner.scene.addBox(
    #     [0.1, 0.4, 0.2], toSE3([0.6, 0, 0.1]), color=(0, 1, 1, 1), name="obstacle"
    # )

    # obstacle_id = planner.scene.collision_model.getGeometryId('obstacle')
    # obstacle = planner.scene.collision_model.geometryObjects[obstacle_id]

    # box = pybullet.createVisualShape( pybullet.GEOM_BOX, \
    #             halfExtents=[0.05, 0.2, 0.1] )
    # pybullet.createMultiBody(baseVisualShapeIndex=box,
    #                   basePosition=[0.6, 0, 0.1],
    #                   useMaximalCoordinates=True)
    
    if False:
        try:
            viz = MeshcatVisualizer(
                planner.scene.model,
                collision_model=planner.scene.collision_model,
                visual_model=planner.scene.visual_model,
            )
            viz.initViewer(open=True)
            viz.loadViewerModel()
        except ImportError as err:
            print("Install Meshcat for visualization: `pip install meshcat`")
            raise err
    else:
        viz = None

    rgba, depth, mask = cam.get_camera_image()


    all_end_to_base = []
    all_rgb = []
    all_dep = []

    init_joints = bot.get_joints()

    if viz:
        viz.display(init_joints)

    print("Press enter to quit")
    input()

    # # Goal end-effector pose
    # p = [0.75, 0, 0.2]
    q = [0, 1, 0, 0]  # wxyz
    # bot.move_to(p, q)


    # bot.close()

    # p = [0.7, 0.1, 0.2]
    # bot.move_to(p, q)

    p = [0.4, 0., 0.5]
    bot.move_to(p, q)

    mtx = np.loadtxt('outputs/camera_calibration/mtx.txt')
    dist = np.loadtxt('outputs/camera_calibration/dist.txt')
    dist = None
    
    # mtx = np.array([[442.46533402, 0., 320.21559605],
    #                 [  0., 442.53050998, 238.0653064 ],
    #                 [  0., 0., 1.]])
    # dist = np.array([-2.08377767e-02 , 3.58425610e-01, -9.65447072e-04,  3.53331351e-04, -1.55962139e+00])
    # dist = np.array([0.00393127, 0.059709, 0.00024387, -0.00068261, -0.23673137])
    
    # mtx = cam.intrinsic

    # print(mtx)

    if True:
        dict_str = '_'.join(board_name.split('_')[1:4])
        col, row, length, sep, start_id = map(lambda x: int(x), board_name.split('_')[4:])
        aruco_dict = aruco.Dictionary_get(ARUCO_DICT[dict_str])
        board = aruco.GridBoard_create(col, row, length / 1000, sep / 1000, aruco_dict, start_id)
        board_params = aruco.DetectorParameters_create()
        
        chess_board_x_num = row
        chess_board_y_num = col

    y_step = 4
    z_step = 4
    count = 0

    p_c2w_e = []

    chess_board_len = 0.04

    for y in range(y_step):
        for z in range(z_step):
            value = np.random.rand()
            q = [0, 1, 0, 0]
            x = 0.7
            # if value > 0.5:
            #     x = 0.65
            # else:
            #     x = 0.55
            
            rot_step = 3
            rot_start = (rot_step - 1)/2

            angle_gap = np.pi/18
            angle_start = rot_start * angle_gap

            for k in range(rot_step):

                rv = np.random.rand()
                if rv > 0.5:
                    q = (Rotation.from_rotvec([-angle_start + k * angle_gap, -angle_start + k * angle_gap, -angle_start + k * angle_gap]) * Rotation.from_quat([0, 1, 0, 0])).as_quat()
                else:
                    q = (Rotation.from_rotvec([-angle_start  + k * angle_gap, -angle_start + k * angle_gap, angle_start - k * angle_gap]) * Rotation.from_quat([0, 1, 0, 0])).as_quat()

                # q = (Rotation.from_rotvec([-np.pi/18 + k * np.pi/18, -np.pi/18 + k * np.pi/18, 0]) * Rotation.from_quat([0, 1, 0, 0])).as_quat()

                # bot.move_to([ 0.4, -0.2 + y * 0.4/y_step, 0.5 - z * 0.3/z_step ], q)
                bot.move_to([ 0.235 + z * 0.1/z_step, -0.1 + y * 0.3/y_step, x ], q)
                
                link_state = pybullet.getLinkState(robot, 8)
                pos, quat = link_state[:2]
                mat = np.eye(4)
                mat[:3,:3] = Rotation.from_quat(quat).as_matrix()
                mat[:3,3] = pos
                
                pybullet.resetBasePositionAndOrientation(box,
                                                        pybullet.getLinkState(robot, 11)[0],
                                                        quat)

                rgba, depth, mask = cam.get_camera_image()

                bgr = rgba[..., 0:3][..., ::-1]
                # cv2.imwrite('./outputs/board/{:04d}.png'.format(count), bgr)
                # count += 1

                corners, ids, rejected_img_points = detect_markers(bgr, aruco_dict, board, board_params, mtx, dist)
                if ids is not None:
                    
                    rvec, tvec, marker_points = aruco.estimatePoseSingleMarkers(corners, chess_board_len, mtx, dist)

                    draw_frame = bgr.copy()
                    aruco.drawDetectedMarkers(draw_frame, corners)
                    for j in range(rvec.shape[0]):
                        cv2.drawFrameAxes(draw_frame, mtx, dist, rvec[j], tvec[j], 0.02)

                    retval, _rvec, _tvec = aruco.estimatePoseBoard(corners, ids, board, mtx, dist, rvec, tvec)
                    # _recval, _rvec, _tvec = cv2.solvePnP( object_points[ids[:,0]].copy(), np.array(corners)[:, 0, 0].copy(), mtx, dist )
                    
                    t_c2w = _tvec[:, 0]

                    rot_c2w = Rotation.from_rotvec(_rvec[:,0]).as_matrix()
                    
                    curr_p_c2w = np.eye(4)
                    curr_p_c2w[0:3, 0:3], curr_p_c2w[0:3, 3] = rot_c2w, t_c2w
                    
                    p_c2w_e.append(curr_p_c2w)
                    
                    cv2.drawFrameAxes(draw_frame, mtx, dist, _rvec, _tvec, 0.2)
                    cv2.imwrite('./outputs/hand_eye_calibration/{:04d}.png'.format(count), draw_frame)
                    count += 1

                    all_end_to_base.append(mat)
    
    p_c2w_e = np.stack(p_c2w_e, axis=0)
    # all_end_to_base = np.stack(all_end_to_base, axis=0)
    p_b2t_g = np.stack(all_end_to_base, axis=0)

    frame_ids = np.array([[i, j] for i in range(count - 1) for j in range(i + 1, count)])
    ai, aj = p_c2w_e[frame_ids[:, 0]], p_c2w_e[frame_ids[:, 1]]
    bi, bj = p_b2t_g[frame_ids[:, 0]], p_b2t_g[frame_ids[:, 1]]
    
    ma = np.matmul(ai, np.linalg.inv(aj))  # candidate camera motions
    mb = np.matmul(np.linalg.inv(bi), bj)

    sma, smb = pre_selection(ma, mb, 0.0002, 15)
    p_c2t_e = dual_quaternion_method(sma, smb)

    p_b2t_g_inv = np.linalg.inv(p_b2t_g)


    all_tmp = []
    for i in range( len(p_b2t_g) ):
        tmp = (np.linalg.inv(p_b2t_g[i]) * cam.pose * np.linalg.inv(p_c2w_e[i]))[:3,3]
        all_tmp.append(tmp)
    all_tmp = np.array(all_tmp)
    mean_tmp = np.mean(all_tmp, axis=0)
    all_dist = np.linalg.norm( all_tmp - mean_tmp, axis=1 )

    all_ids = np.argsort(all_dist)[:15]
    all_ids = [i for i in range(len(all_dist))]

    R_all_end_to_base_2 = p_b2t_g[:, :3, :3]
    T_all_end_to_base_2 = p_b2t_g[:, :3, 3]

    R_all_end_to_base_1 = p_b2t_g_inv[:, :3, :3]
    T_all_end_to_base_1 = p_b2t_g_inv[:, :3, 3]

    R_all_chess_to_cam_1 = p_c2w_e[:,:3,:3]
    T_all_chess_to_cam_1 = p_c2w_e[:,:3,3]

    R, T = cv2.calibrateHandEye(
        R_all_end_to_base_1, T_all_end_to_base_1, R_all_chess_to_cam_1, T_all_chess_to_cam_1, method=cv2.CALIB_HAND_EYE_TSAI
    )
    R2, T2 = cv2.calibrateHandEye(
        R_all_end_to_base_2, T_all_end_to_base_2, R_all_chess_to_cam_1, T_all_chess_to_cam_1, method=cv2.CALIB_HAND_EYE_TSAI
    )
    
    np.save("./end2base.npy", p_b2t_g)
    np.save("./tag2cam.npy", p_c2w_e)

    print(T)
    print(T2)
    print(p_c2t_e[:3,3])
    # # R2, T2 = cv2.calibrateHandEye(
    # #     R_all_end_to_base_2, T_all_end_to_base_2, R_all_chess_to_cam_1, T_all_chess_to_cam_1
    # # )

    p = [0.4, 0.1, 0.2]
    bot.move_to(p, q)

    bot.open()

    input()
