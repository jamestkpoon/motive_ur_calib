#!/usr/bin/python

import rospy, tf_conversions
from james_ur_kinematics.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion
from natnet_msgs.msg import MarkerList

import numpy as np
from time import sleep

motive_samples_fp = '/home/rll-ur5/catkin_ws/src/hardware/motive_ur_calib/calib_data.npy'

px_r_ = [ 0.6, 0.7 ]; py_r_ = [ -0.2, 0.2 ]; pz_r_ = [ 0.1, 0.2 ]
rr_r_ = [ -np.pi/2, np.pi/2 ]; rp_r_ = [ 0.0, np.pi/8 ]; ry_r_ = [ -np.pi/6, np.pi/6 ]
n_samples_ = 20
marker_readings_per_sample_ = 10
ur_move_time_ = 5.0

if __name__ == '__main__':
    rospy.init_node('ur5_motive_calib_rec_script', anonymous=True)

    ### UR control
       
    m2j_topic_ = 'ur5/command/moveToJoints'
    rospy.wait_for_service(m2j_topic_)
    m2_ = rospy.ServiceProxy(m2j_topic_, MoveTo)

    ik_topic_ = 'ur5_kin/IK'
    rospy.wait_for_service(ik_topic_)
    ik_ = rospy.ServiceProxy(ik_topic_, IK)

    ### pose generation
    
    pose_ranges_ = [ px_r_, py_r_, pz_r_, rr_r_, rp_r_, ry_r_ ]
    pose_mindiff_ = [ [ np.min(pose_ranges_[i]), np.ptp(pose_ranges_[i]) ] for i in range(6) ]
    quaternion_from_euler = tf_conversions.transformations.quaternion_from_euler
    
    def random_pose():
        while True:
            r_ = np.random.random_sample(6)
            pose6_ = [ pose_mindiff_[i][0] + r_[i]*pose_mindiff_[i][1] for i in range(6) ]
            ql_ = list(quaternion_from_euler(*pose6_[3:]))
            ik_req_ = IKRequest(ee_pose=Pose(
                position=Point(*pose6_[:3]), orientation=Quaternion(*ql_)))
            jangs_ = ik_(ik_req_).joint_angles
            if len(jangs_) > 0: return list(jangs_[:6]), pose6_[:3]+ql_

    ### marker data handling

    def get_loose_markers_msg():
        while True:
            msg_ = rospy.wait_for_message(
                '/natnet_client/markers/leftovers', MarkerList)
            if len(msg_.ids) != 0: return msg_
            else: sleep(0.1)

    def get_marker_ID():
        while True:
            try:
                return int(raw_input('  Please enter a marker ID: '))
            except:
                print('  Invalid input, please try again ...')
            
    def get_marker_xyz(mID):
        while True:
            motive_msg_ = get_loose_markers_msg()
            if not (mID in motive_msg_.ids):
                mID = get_marker_ID()
            else:                
                i_ = motive_msg_.ids.index(mID)
                xyz_ = [ motive_msg_.positions[i_].x,
                    motive_msg_.positions[i_].y, motive_msg_.positions[i_].z ]
                return xyz_, mID

    mID_ = None

    ### actual data gettin'
    
    motive_samps_all_ = np.empty([n_samples_,10])
    for i in range(n_samples_):
        # move robot
        j6_,p7_ = random_pose()
        m2_(MoveToRequest(ur_state=j6_+[ur_move_time_], gripper_state=True))
        # marker positions
        motive_samps_ = np.empty([marker_readings_per_sample_,3])
        for j in range(marker_readings_per_sample_):
            motive_samps_[j,:],mID_ = get_marker_xyz(mID_)
        motive_samps_mean_ = np.mean(motive_samps_, axis=0)
        # append
        motive_samps_all_[i,:7] = p7_
        motive_samps_all_[i,7:] = motive_samps_mean_
        print('Recorded %d/%d' % (i+1,n_samples_))

    np.save(motive_samples_fp, motive_samps_all_)
