#!/usr/bin/python

import rospy, tf_conversions, rospkg
from james_ur_kinematics.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion
from natnet_msgs.msg import MarkerList

import numpy as np
from time import sleep

pkg_dir = rospkg.RosPack().get_path('motive_ur_calib') + '/'
motive_samples_fp = pkg_dir + 'calib_data_motive.npy'

### UR5 control

px_r = [ 0.6, 0.7 ]; py_r = [ -0.2, 0.2 ]; pz_r = [ 0.1, 0.2 ]
rr_r = [ -np.pi/2, np.pi/2 ]; rp_r = [ 0.0, np.pi/8 ]; ry_r = [ -np.pi/6, np.pi/6 ]
ur_move_time = 5.0

pose_ranges_ = [ px_r, py_r, pz_r, rr_r, rp_r, ry_r ]
pose_mindiff_ = [ [ np.min(pose_ranges_[i]), np.ptp(pose_ranges_[i]) ] for i in range(6) ]
quaternion_from_euler = tf_conversions.transformations.quaternion_from_euler

def gen_random_pose(ik_svc):
    while True:
        r_ = np.random.random_sample(6)
        pose6_ = [ pose_mindiff_[i][0] + r_[i]*pose_mindiff_[i][1] for i in range(6) ]
        ql_ = list(quaternion_from_euler(*pose6_[3:]))
        ik_req_ = IKRequest(ee_pose=Pose(
            position=Point(*pose6_[:3]), orientation=Quaternion(*ql_)))
        jangs_ = ik_svc(ik_req_).joint_angles
        if len(jangs_) > 0: return list(jangs_[:6]), pose6_[:3]+ql_
        
def move_to_random_pose(ik_svc, m2j_svc):
    j6_,p7_ = gen_random_pose(ik_svc)
    m2j_svc(MoveToRequest(ur_state=j6_+[ur_move_time], gripper_state=True))
    
    return j6_, p7_

### user input with typecast

def cast_input(datatype, s=''):
    while True:
        try:
            return datatype(raw_input(s))
        except:
            print('    Invalid input, please try again ...')

if __name__ == '__main__':

    ### ROS stuff

    rospy.init_node('ur5_motive_calib_rec_script', anonymous=True)

    ik_topic_ = 'ur5_kin/IK'
    rospy.wait_for_service(ik_topic_)
    ik_svc_ = rospy.ServiceProxy(ik_topic_, IK)

    m2j_topic_ = 'ur5/command/moveToJoints'
    rospy.wait_for_service(m2j_topic_)
    m2j_svc_ = rospy.ServiceProxy(m2j_topic_, MoveTo)

    ### marker data handling

    def get_loose_markers_msg():
        while True:
            msg_ = rospy.wait_for_message(
                '/natnet_client/markers/leftovers', MarkerList)
            if len(msg_.ids) != 0: return msg_
            else: sleep(0.1)
            
    def get_marker_xyz(mID):
        while True:
            motive_msg_ = get_loose_markers_msg()
            if not (mID in motive_msg_.ids):
                mID = cast_input(int, '  Please enter a marker ID: ')
            else:                
                i_ = motive_msg_.ids.index(mID)
                xyz_ = [ motive_msg_.positions[i_].x,
                    motive_msg_.positions[i_].y, motive_msg_.positions[i_].z ]
                return xyz_, mID

    mID_ = None

    ### actual data gettin'
    
    n_samples_ = 20
    marker_readings_per_sample_ = 10
    
    motive_samps_all_ = np.empty([n_samples_,10])
    for i in range(n_samples_):
        # move robot
        j6_,p7_ = move_to_random_pose(ik_svc_, m2j_svc_)
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
