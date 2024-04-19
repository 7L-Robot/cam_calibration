import os
import numpy as np
from scipy.spatial.transform import Rotation

import pybullet
import pybullet_data

from simulation.simulator import Robot, Camera, RigidObject
from simulation import utils

from calibrator import Extrinsic_Calibrator



# if __name__ == '__main__':
if __name__ in {"__main__", "__mp_main__"}:

    from nicegui import ui

    ui.label('Hello NiceGUI!')
    ui.button('BUTTON', on_click=lambda: ui.notify('button was pressed'))

    ui.run()

    # 是否眼在手上
    eye_on_hand = False
    
    if eye_on_hand:
        urdf_path = "./franka_description/robots/panda_arm_cam.urdf"
    else:
        urdf_path = "./franka_description/robots/panda_arm_hand.urdf"
    srdf_path = "./franka_description/robots/panda_arm_hand.srdf"

    board_name = 'ArUco_DICT_7X7_50_5_7_500_100_0'
    

    physcisClient = pybullet.connect(pybullet.GUI)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 1)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeID = pybullet.loadURDF("plane.urdf")
    robot = pybullet.loadURDF(urdf_path, useFixedBase=True)

    vis_params, col_params, body_params = utils.get_param_template()
    col_params['fileName'] = vis_params['fileName'] = os.path.join('franka_description/board', board_name + '.obj')
    
    # 加载二维码板子
    if eye_on_hand:
        board_obj = RigidObject(board_name, vis_params, col_params, body_params)
        board_obj.set_pose_from_pos_and_quat([0.47, -0.1, 0.01], [0, 0, 0, 1])

    # 加相机
    cam = Camera(640, 480, 57, 3, [1.5, -0.1, 0.4], [0,0,0.37], [-1, 0, 0], 0.055, 10)

    # 加机器人
    bot = Robot(robot, cam, eye_on_hand)

    board_type = '_'.join(board_name.split('_')[1:4])
    board_col, board_row, length, sep, start_id = map(lambda x: int(x), board_name.split('_')[4:])

    cali = Extrinsic_Calibrator(board_type, board_col, board_row, 
                                board_grid_len=length / 10000, # 米，上面图片名字记录的是毫米mm
                                board_grid_gap=sep / 10000, # 米，上面图片名字记录的是毫米mm
                                board_grid_start_id=start_id)

    x,y,z,w = Rotation.from_rotvec([0, np.pi/2, 0]).as_quat()
    q = [w,x,y,z]

    input("Press enter to start")

    # 加内参，可以用 cali_intrinsic.py 计算
    try:
        cam_in = np.loadtxt('outputs/camera_calibration/cam_in.txt')
        cam_dist = np.loadtxt('outputs/camera_calibration/coff_dis.txt')
    except:
        print("no cam_in, please run 'example_intrinsic.py' at first")
        exit(-1)
    # NOTE 畸变参数不用反而更准
    cam_dist = None

    y_step = 4
    z_step = 4
    count = 0

    ee_to_base_poses = []

    board_imgs = []

    for y in range(y_step):
        for z in range(z_step):
            
            if eye_on_hand:
                x = 0.7
                rot_step = 3
                angle_gap = np.pi/18
            else:
                x = 0.5
                rot_step = 4
                angle_gap = np.pi/10

            rot_start = (rot_step - 1)/2
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
        
    eye_to_hand_mat = cali.hand_eye_calibration(board_imgs, ee_to_base_poses, eye_on_hand, cam_in, cam_dist)


    if eye_on_hand:
        print('GT: [0.2 0.1 0.0]')
        # See panda_hand_cam.urdf line 295
    else:
        print('GT: ', cam.pose[:3,3])
    print('Result: ', eye_to_hand_mat[:3,3])

    
    # bot.open()

    input("Press enter to close")
