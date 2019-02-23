#!/usr/bin/python

import rospy, tf_conversions
from motive_ur_calib.srv import *
from geometry_msgs.msg import Point, Quaternion, Pose

import numpy as np

def ROSPts_to_4xN_arr(P):
    ls_ = [ [ p.x, p.y, p.z, 1.0 ] for p in P ]
    return np.asarray(ls_).T
    
def x1_arr_to_ROSPt(M):
    return Point(x=M[0], y=M[1], z=M[2])

def xN_arr_to_ROSPts(M):
    return [ x1_arr_to_ROSPt(M[:,i]) for i in range(M.shape[1]) ]
    
def ROSQuat_to_xyzw(q):
    return [ q.x, q.y, q.z, q.w ]
    
def ROSPose_to_TM(pose):
    f_ = tf_conversions.posemath.fromMsg(pose)
    return tf_conversions.posemath.toMatrix(f_)
    
def TM_to_ROSPose(M):
    f_ = tf_conversions.posemath.fromMatrix(M)
    return tf_conversions.posemath.toMsg(f_)

class MotiveTF_node():
    def __init__(self):
        rospy.init_node('motiveTF_node', anonymous=True)
        
        htm_list_ = rospy.get_param("~htm").split(',')
        self._htm = np.asarray(map(float,htm_list_)).reshape(4,4)

        self._rb_rot_adj = tf_conversions.transformations.quaternion_from_matrix(self._htm)
        self._rb_rot_adj[3] = -self._rb_rot_adj[3] # invert by taking w = -w
        
        rospy.Service('motiveTF', MotiveTF, self._tf_cb)
        
        rospy.spin()
        
    def _tf_cb(self, req):
        res_ = MotiveTFResponse()
        
        # marker positions
        res_.markers = xN_arr_to_ROSPts(
            np.matmul(self._htm, ROSPts_to_4xN_arr(req.markers)))
        
        # rigid body poses
        n_ = len(req.rigid_bodies); res_.rigid_bodies = []
        for i in range(n_):
            quat_rot_adj_ = tf_conversions.transformations.quaternion_multiply(
                ROSQuat_to_xyzw(req.rigid_bodies[i].orientation), self._rb_rot_adj)
            req_pose_rot_adj_ = Pose(position=req.rigid_bodies[i].position,
                orientation=Quaternion(*quat_rot_adj_))           
            M_tf_ = np.matmul(self._htm, ROSPose_to_TM(req_pose_rot_adj_))
            res_.rigid_bodies.append(TM_to_ROSPose(M_tf_))
    
        return res_
        
if __name__ == '__main__':
    mTF_node_ = MotiveTF_node()
