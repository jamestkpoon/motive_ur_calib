#!/usr/bin/python

import rospy
from motive_ur_calib.srv import *
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

rigid_body_name_ = 'Gear_big'

rospy.init_node('motive_ur_calib_test', anonymous=True)

rospy.wait_for_service('motiveTF')
svc_ = rospy.ServiceProxy('motiveTF', MotiveTF)

tfbc_ = TransformBroadcaster()

def broadcast_pose_TF(pose, parent_frame='ur_base_link', child_frame='tf_test'):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation = pose.position
    t.transform.rotation = pose.orientation
    tfbc_.sendTransform(t)

def pose_cb(msg):
    req_ = MotiveTFRequest(rigid_bodies = [ msg.pose ])
    rigid_body_pose_tf_ = svc_(req_).rigid_bodies[0]
    broadcast_pose_TF(rigid_body_pose_tf_, child_frame=rigid_body_name_+'_TF')

rigid_body_pose_topic_ = '/natnet_client/rigid_bodies/' + rigid_body_name_ + '/pose'
rospy.Subscriber(rigid_body_pose_topic_, PoseStamped, pose_cb)
rospy.spin()
