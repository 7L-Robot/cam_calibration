
from simulation.simulator import Camera, RigidObject
from simulation import utils

import numpy as np
import pybullet as p
import pybullet_data

import os

from calibrator import Intrinsic_Calibrator


def calibrate():
    
    p.connect(p.GUI)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF('plane.urdf')
    vis_params, col_params, body_params = utils.get_param_template()

    board_name = 'chessboard_8_10_30_15'
    
    num_img = 40
    board_inner_col = 7 # 8-1
    board_inner_row = 9 # 10-1
    board_grid_len = 0.03 # 30/1000 米

    col_params['fileName'] = vis_params['fileName'] = os.path.join('franka_description', 'board', board_name + '.obj')
    board_obj = RigidObject(board_name, vis_params, col_params, body_params)
    board_obj.set_pose_from_pos_and_quat([0, 0, 0.1], [0, 0, 0, 1])
    
    
    cam = Camera(640, 480, 57, 3, [0, 0, 0.67], [0,0,0], [-1, 0, 0], 0.055, 10)

    cali = Intrinsic_Calibrator(board_inner_col, board_inner_row, board_grid_len)
    
    board_imgs = []

    print("Collecting data")
    for _ in range(num_img):
        # capturing chessboard with various distance will achieve more precise estimation
        radius = np.random.uniform(0.47, 0.67)
        cam.sample_a_pose_from_a_sphere(np.array([0, 0, 0]), radius)
        rgba, depth, mask = cam.get_camera_image()
        bgr = rgba[..., 0:3][..., ::-1]
        
        board_imgs.append(bgr)
    
    print("Calibrating")
    cam_intrinsic, coff_dis = cali.calibrate_camera(board_imgs, viz=False)

    print('GT: \n', cam.intrinsic)
    print('Result: \n', cam_intrinsic)

    np.savetxt("./outputs/camera_calibration/cam_in.txt", cam_intrinsic)
    np.savetxt("./outputs/camera_calibration/coff_dis.txt", coff_dis)

if __name__ == '__main__':
    calibrate()


