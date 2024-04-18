
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
    '''
    Code from 

    https://github.com/caijunhao/calibration

    '''

    def __init__(self, board_type='DICT_7X7_50', board_col=5, board_row=7, 
                 board_grid_len=0.05, board_grid_gap=0.01, board_grid_start_id=0):
        '''
        board_grid_len: meter
        board_grid_gap: meter
        '''
        
        aruco_dict = aruco.Dictionary_get(ARUCO_DICT[board_type])
        board = aruco.GridBoard_create(board_col, board_row, board_grid_len, board_grid_gap, aruco_dict, board_grid_start_id)
        board_params = aruco.DetectorParameters_create()

        self.aruco_dict = aruco_dict
        self.board = board
        self.board_params = board_params
        # self.real_grid_len = real_grid_len
        self.real_grid_len = board_grid_len - board_grid_gap

    def compute_board_to_cam(self, board_imgs, cam_in, cam_dist, viz=False):
        
        board_to_cam_list = []
        
        count = 0

        for board_img in board_imgs:

            if type(board_img) == str:
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



class Intrinsic_Calibrator(object):
    """
    https://github.com/zhiyuanyou/Calibration-ZhangZhengyou-Method

    Calibrate the Camera with Zhang Zhengyou Method.

    By You Zhiyuan, 2022.07.04, zhiyuanyou@foxmail.com


    reorganize by Juzhan.

    """
    def __init__(self, board_inner_col, board_inner_row, board_grid_len):
        """
        --parameters--
        shape_inner_corner: the shape of inner corner, Array of int, (h, w)
        size_grid: the real size of a grid in calibrator, float
        visualization: whether visualization, bool
        """
        self.board_grid_len = board_grid_len

        # create the conner in world space
        # cp_int: corner point in int form, save the coordinate of corner points in world sapce in 'int' form
        # like (0,0,0), (1,0,0), (2,0,0) ...., (10,7,0)
        cp_int = np.zeros((board_inner_col * board_inner_row, 3), np.float32)
        cp_int[:,:2] = np.mgrid[0:board_inner_col,0:board_inner_row].T.reshape(-1,2)
        # cp_world: corner point in world space, save the coordinate of corner points in world space
        self.cp_world = cp_int * board_grid_len

        self.board_inner_col = board_inner_col
        self.board_inner_row = board_inner_row


    def calibrate_camera(self, board_imgs, viz=False):

        # criteria: only for subpix calibration, which is not used here
        # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        points_world = [] # the points in world space
        points_pixel = [] # the points in pixel space (relevant to points_world)
        for img in board_imgs:

            if type(img) == str:
                img = cv2.imread(img)
            
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # find the corners, cp_img: corner points in pixel space
            ret, cp_img = cv2.findChessboardCorners(gray_img, (self.board_inner_col, self.board_inner_row), None)
            # if ret is True, save
            if ret:
                # cv2.cornerSubPix(gray_img, cp_img, (11,11), (-1,-1), criteria)
                points_world.append(self.cp_world)
                points_pixel.append(cp_img)
                # view the corners
                if viz:
                    cimg = img.copy()
                    cv2.drawChessboardCorners(cimg, (self.board_inner_col, self.board_inner_row), cp_img, ret)
                    cv2.imshow('FoundCorners', cimg)
                    cv2.waitKey(500)

        # calibrate the camera
        ret, cam_intrinsic, coff_dis, v_rot, v_trans = cv2.calibrateCamera(points_world, points_pixel, gray_img.shape[::-1], None, None)

        # calculate the error of reproject
        # total_error = 0
        # for i in range(len(points_world)):
        #     points_pixel_repro, _ = cv2.projectPoints(points_world[i], v_rot[i], v_trans[i], cam_intrinsic, coff_dis)
        #     error = cv2.norm(points_pixel[i], points_pixel_repro, cv2.NORM_L2) / len(points_pixel_repro)
        #     total_error += error
        # avg_proj_error = total_error / len(points_world)
        # print("Average error of reproject: {}".format(avg_proj_error))
        

        return cam_intrinsic, coff_dis


    def dedistortion(self, img, cam_intrinsic, coff_dis):
        
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_intrinsic, coff_dis, 
                                                          (self.board_inner_col, self.board_inner_row), 0, 
                                                          (self.board_inner_col, self.board_inner_row))
        dst = cv2.undistort(img, cam_intrinsic, self.coff_dis, None, newcameramtx)
        # clip the image
        # x, y, w, h = roi
        # dst = dst[y:y+h, x:x+w]
        return dst