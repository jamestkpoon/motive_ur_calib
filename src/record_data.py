#!/usr/bin/python

import rospy, tf_conversions
from james_ur_kinematics.srv import *
from natnet_msgs.msg import MarkerList

import numpy as np
from time import sleep

quaternion_from_euler = tf_conversions.transformations.quaternion_from_euler

def gen_gripper_positions(Xrange,Yrange,Zrange, d):
    positions_ = []; x_ = Xrange[0]
    while x_ <= Xrange[1]:
        y_ = Yrange[0]
        while y_ <= Yrange[1]:
            z_ = Zrange[0]
            while z_ <= Zrange[1]:
                positions_.append([ x_, y_, z_ ])
                z_ += d[2]
            y_ += d[1]
        x_ += d[0]
    return positions_

xr_ = [ 0.6, 0.7 ]; yr_ = [ -0.2, 0.0 ]; zr_ = [ 0.36, 0.4 ]
shift_ = [ 0.02, 0.05, 0.01 ]
gripper_positions = gen_gripper_positions(xr_,yr_,zr_, shift_)

gripper_rpyA = [ 0.0, -np.pi/2, 0.0 ]
gripper_rpyB = [ 0.0, -np.pi/2, -np.pi ]
gripper_qA_ = list(quaternion_from_euler(
    gripper_rpyA[0],gripper_rpyA[1],gripper_rpyA[2]))
gripper_qB_ = list(quaternion_from_euler(
    gripper_rpyB[0],gripper_rpyB[1],gripper_rpyB[2]))
move_dur_ = 5.0

motive_samples_fp = '/home/rll-ur5/catkin_ws/src/motive_ur_calib/calib_data.npy'
motive_samp_N_ = 20

def get_loose_markers_msg():
    while True:
        msg_ = rospy.wait_for_message(
            '/natnet_client/markers/leftovers', MarkerList)
        if len(msg_.ids) != 0: return msg_
        else: sleep(0.1)

def get_marker0_ID():
    while True:
        motive_msg_ = get_loose_markers_msg()
        if len(motive_msg_.ids) == 1: return motive_msg_.ids[0]
        else: sleep(0.1)
        
def get_marker_xyz(mID):
    while True:
        motive_msg_ = get_loose_markers_msg()
        if not mID in motive_msg_.ids:
            input_ = raw_input('  Please enter a new marker ID: ')
            mID = get_marker0_ID() if input_ == '' else int(input_)
        else:                
            i_ = motive_msg_.ids.index(mID)
            xyz_ = [ motive_msg_.positions[i_].x,
                motive_msg_.positions[i_].y, motive_msg_.positions[i_].z ]
            return xyz_, mID

if __name__ == '__main__':    
    rospy.init_node('ur5_motive_calib_script', anonymous=True)
    rospy.wait_for_service('ur5/command/moveTo')
    m2_ = rospy.ServiceProxy('ur5/command/moveTo', MoveTo)

    motive_samps_all_ = np.empty([ len(gripper_positions), 3 ])
    
    m0_id_ = get_marker0_ID()
    print('  Got marker0 ID: %d' % m0_id_)
        
    for i in range(len(gripper_positions)):
        motive_samps_ = np.empty([2*motive_samp_N_,3])
        
        pose_tuple_ = gripper_positions[i] + gripper_qA_ + [ move_dur_ ]
        m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))
        for j in range(0, motive_samp_N_):
            motive_samps_[j,:], m0_id_ = get_marker_xyz(m0_id_) 

        pose_tuple_ = gripper_positions[i] + gripper_qB_ + [ move_dur_ ]
        m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))
        for j in range(motive_samp_N_, 2*motive_samp_N_):
            motive_samps_[j,:], m0_id_ = get_marker_xyz(m0_id_)
        
        motive_samps_all_[i,:] = np.mean(motive_samps_, axis=0)            
        print('Recorded %d/%d' % (i+1,len(gripper_positions)))

    def hypot_vec(M):
        diff_ = M[1:,:] - M[:-1,:]
        return np.sqrt(np.sum(np.square(diff_), axis=1))

    gripper_hyp_ = hypot_vec(gripper_positions)
    motive_hyp_ = hypot_vec(motive_samps_all_)
    maxdiff_ = np.max(np.abs(gripper_hyp_ - motive_hyp_))
    print('  Max hypotenuse error: %f' % maxdiff_)
    
    np.save(motive_samples_fp, motive_samps_all_)
