# -*- coding: utf-8 -*-
"""
https://github.com/zhiyuanyou/Calibration-ZhangZhengyou-Method

Calibrate the Camera with Zhang Zhengyou Method.

By You Zhiyuan, 2022.07.04, zhiyuanyou@foxmail.com
"""

import cv2
import numpy as np


class Intrinsic_Calibrator(object):
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
        # print ("ret: {}".format(ret))
        # print ("intrinsic matrix: \n {}".format(cam_intrinsic))
        # NOTE in the form of (k_1, k_2, p_1, p_2, k_3)
        # print ("distortion cofficients: \n {}".format(coff_dis))
        # print ("rotation vectors: \n {}".format(v_rot))
        # print ("translation vectors: \n {}".format(v_trans))

        # calculate the error of reproject
        total_error = 0
        for i in range(len(points_world)):
            points_pixel_repro, _ = cv2.projectPoints(points_world[i], v_rot[i], v_trans[i], cam_intrinsic, coff_dis)
            error = cv2.norm(points_pixel[i], points_pixel_repro, cv2.NORM_L2) / len(points_pixel_repro)
            total_error += error
        
        avg_proj_error = total_error / len(points_world)
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