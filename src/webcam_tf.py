#!/usr/bin/python

import rospy
from james_ur_kinematics.srv import *
from record_data import move_to_random_pose, cast_input, pkg_dir

from sensor_msgs.msg import Image
import ros_numpy, cv2

from solve_tf import *
import numpy as np

cam_topic = '/usb_cam/image_raw'
cam_res_H = 640
cam_res_V = 480
cam_fov_H = 70.42 * np.pi/180.0
cam_fov_V = 43.3 * np.pi/180.0

tf_seed = [ 1.0, 0.0, 0.3, 0.0, 0.5, 3.1416 ]

if __name__ == '__main__':
    
    ### ROS stuff

    rospy.init_node('ur5_webcam_calib_script', anonymous=True)
       
    ik_topic_ = 'ur5_kin/IK'
    rospy.wait_for_service(ik_topic_)
    ik_svc_ = rospy.ServiceProxy(ik_topic_, IK)

    m2j_topic_ = 'ur5/command/moveToJoints'
    rospy.wait_for_service(m2j_topic_)
    m2j_svc_ = rospy.ServiceProxy(m2j_topic_, MoveTo)

    ### record samples
    
    # image GUI (left-clicking)
    u_in_ = -1; v_in_ = -1
    def mouse_cb(event, x,y, flags,params):
        if event == cv2.EVENT_LBUTTONDOWN:
            global u_in_, v_in_
            u_in_ = x; v_in_ = y
    
    window_name_ = 'webcam_tf.py'
    cv2.startWindowThread(); cv2.namedWindow(window_name_); 
    cv2.setMouseCallback(window_name_, mouse_cb)

    samples_ = []
    while True:
        # move robot
        j6_,p7_ = move_to_random_pose(ik_svc_, m2j_svc_)
        # get image position of end-effector TCP
#        u_in_ = cast_input(int, '  Horizontal px co-ordinate: ')
#        v_in_ = cast_input(int, '  Vertical   px co-ordinate: ')        
        u_in_ = -1; v_in_ = -1
        while True:
            img_ = ros_numpy.numpify(rospy.wait_for_message(cam_topic, Image))
            cv2.imshow(window_name_, img_); cv2.waitKey(50)
            if u_in_ != -1 and v_in_ != -1: break            
        # append, then check if to continue
        samples_.append(p7_[:3] + map(float, [ u_in_, v_in_ ]))
        in_ = raw_input('%d samples. Press Enter to record more, or input anything else to calibrate ...' % len(samples_))
        if in_ != '': break
        
    cv2.destroyWindow(window_name_)
        
    samples_ = np.asarray(samples_)
    np.save(pkg_dir+'calib_data_camera.npy', samples_)
    
    ### fit (assuming pinhole camera), and save
    
    if not ('samples_' in locals()): samples_ = np.load(pkg_dir+'calib_data_camera.npy')
    n_samples_ = samples_.shape[0]
    ur_tcp_positions_ = np.concatenate((samples_[:,:3].T,np.ones((1,n_samples_))), axis=0)
    cam_uv_ = samples_[:,3:]
    
    px_per_rad_of_yaw_ = cam_res_H / cam_fov_H
    px_per_rad_of_pitch_ = cam_res_V / cam_fov_V

    def xyzrpy_to_inv_4x4HTM(xyz_rpy):
        htm_ = np.empty([4,4])
        htm_[:3,:3] = euler_to_rotM(xyz_rpy[3:]).T
        htm_[:3,3] = -np.matmul(htm_[:3,:3], xyz_rpy[:3])
        htm_[3,:] = [ 0.0, 0.0, 0.0, 1.0 ]
        
        return htm_
    
    def get_tf_px_hypot(tf):
        P_ = np.matmul(xyzrpy_to_inv_4x4HTM(tf), ur_tcp_positions_).T
        yaw_ = np.arctan2(P_[:,1], P_[:,0]).reshape((n_samples_,1))
        pitch_ = -np.arctan2(P_[:,2], P_[:,0]).reshape((n_samples_,1))
        
        u_ = cam_res_H/2 - (yaw_ * px_per_rad_of_yaw_)
        v_ = cam_res_V/2 + (pitch_ * px_per_rad_of_pitch_)
        uv_ = np.concatenate((u_,v_), axis=1)
                
        return np.sqrt(np.sum(np.square(cam_uv_-uv_), axis=1))
        
    tf_opt_ = optimize_TF(get_tf_px_hypot, np.asarray(tf_seed))
    tf_opt_[3:] *= -1 # ???
    write_TF_roslaunch(pkg_dir+'cam_ur_tf.launch', tf_opt_, child_frame='camera')
