'''
Code from 

https://github.com/caijunhao/calibration

'''

import numpy as np
from scipy.spatial.transform import Rotation

import cv2
from cv2 import aruco

ARUCO_DICT = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_4X4_1000": aruco.DICT_4X4_1000,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_5X5_250": aruco.DICT_5X5_250,
    "DICT_5X5_1000": aruco.DICT_5X5_1000,
    "DICT_6X6_50": aruco.DICT_6X6_50,
    "DICT_6X6_100": aruco.DICT_6X6_100,
    "DICT_6X6_250": aruco.DICT_6X6_250,
    "DICT_6X6_1000": aruco.DICT_6X6_1000,
    "DICT_7X7_50": aruco.DICT_7X7_50,
    "DICT_7X7_100": aruco.DICT_7X7_100,
    "DICT_7X7_250": aruco.DICT_7X7_250,
    "DICT_7X7_1000": aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": aruco.DICT_APRILTAG_36h11
}

def detect_markers(bgr, aruco_dict, board, params, mtx, dist):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                            parameters=params,
                                                            # cameraMatrix=mtx,
                                                            # distCoeff=dist
                                                            )
    aruco.refineDetectedMarkers(gray, board, corners, ids, rejected_img_points,
                                cameraMatrix=mtx,
                                distCoeffs=dist
                                )
    return corners, ids, rejected_img_points



class Extrinsic_Calibrator(object):

    def __init__(self, board_type='DICT_7X7_50', board_col=5, board_row=7, board_grid_len=0.05, board_grid_gap=0.01, real_grid_len=0.04, board_grid_start_id=0):
        
        aruco_dict = aruco.Dictionary_get(ARUCO_DICT[board_type])
        board = aruco.GridBoard_create(board_col, board_row, board_grid_len, board_grid_gap, aruco_dict, board_grid_start_id)
        board_params = aruco.DetectorParameters_create()

        self.aruco_dict = aruco_dict
        self.board = board
        self.board_params = board_params
        self.real_grid_len = real_grid_len

    def compute_board_to_cam(self, board_imgs, cam_in, cam_dist, viz=False):
        
        board_to_cam_list = []
        
        count = 0

        for board_img in board_imgs:

            if type(board_img) == np.array: continue
            else:
                board_img = cv2.imread(board_img)
            
            corners, ids, rejected_img_points = detect_markers(board_img, self.aruco_dict, self.board, self.board_params, cam_in, cam_dist)

            if ids is not None:
                rvec, tvec, marker_points = aruco.estimatePoseSingleMarkers(corners, self.real_grid_len, cam_in, cam_dist)

                retval, _rvec, _tvec = aruco.estimatePoseBoard(corners, ids, self.board, cam_in, cam_dist, rvec, tvec)
                
                board_to_cam_tran = _tvec[:, 0]

                board_to_cam_rot = Rotation.from_rotvec(_rvec[:,0]).as_matrix()
                
                board_to_cam = np.eye(4)
                board_to_cam[0:3, 0:3], board_to_cam[0:3, 3] = board_to_cam_rot, board_to_cam_tran
                
                board_to_cam_list.append(board_to_cam)

                if viz:
                    draw_frame = board_img.copy()
                    aruco.drawDetectedMarkers(draw_frame, corners)
                    for j in range(rvec.shape[0]):
                        cv2.drawFrameAxes(draw_frame, cam_in, cam_dist, rvec[j], tvec[j], 0.02)

                    cv2.drawFrameAxes(draw_frame, cam_in, cam_dist, _rvec, _tvec, 0.2)
                    # cv2.imwrite('./outputs/hand_eye_calibration/{:04d}.png'.format(count), draw_frame)
                    count += 1

        board_to_cam_poses = np.stack(board_to_cam_list, axis=0)
        
        return board_to_cam_poses
        
        
    def hand_eye_calibration(self, board_imgs, ee_to_base_poses, is_eye_to_hand, cam_in, cam_dist=None ):
        
        ee_to_base_poses = np.array(ee_to_base_poses)

        board_to_cam_poses = self.compute_board_to_cam(board_imgs, cam_in, cam_dist)
        
        board_to_cam_rots = board_to_cam_poses[:,:3,:3]
        board_to_cam_trans = board_to_cam_poses[:,:3,3]


        if is_eye_to_hand:
            base_to_ee_poses = np.linalg.inv(ee_to_base_poses)

            base_to_ee_rots = base_to_ee_poses[:, :3, :3]
            base_to_ee_trans = base_to_ee_poses[:, :3, 3]

            R, T = cv2.calibrateHandEye(
                base_to_ee_rots, base_to_ee_trans, board_to_cam_rots, board_to_cam_trans, method=cv2.CALIB_HAND_EYE_TSAI
            )
            eye_to_hand_mat = np.eye(4)
            eye_to_hand_mat[:3,:3] = R
            eye_to_hand_mat[:3, 3] = T[:,0]

            return eye_to_hand_mat

        else:
            ee_to_base_rots = ee_to_base_poses[:, :3, :3]
            ee_to_base_trans = ee_to_base_poses[:, :3, 3]

            R, T = cv2.calibrateHandEye(
                ee_to_base_rots, ee_to_base_trans, board_to_cam_rots, board_to_cam_trans, method=cv2.CALIB_HAND_EYE_TSAI
            )
            eye_on_hand_mat = np.eye(4)
            eye_on_hand_mat[:3,:3] = R
            eye_on_hand_mat[:3, 3] = T[:,0]
            
            return eye_on_hand_mat

