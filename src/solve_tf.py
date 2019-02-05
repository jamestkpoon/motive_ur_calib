#!/usr/bin/python

import numpy as np

### get stuff from rec script

from record_data import gripper_positions,gripper_rpyA, motive_samples_fp

gripper_positions_ = np.asarray(gripper_positions)
motive_samples = np.load(motive_samples_fp)

### Optitrack -> robot base TF

## offset TCP to centre of marker

def euler_to_rotM(rpy):
    sin_ = np.sin(rpy); cos_ = np.cos(rpy)
    Ax_ = np.asarray([ 1, 0, 0, 0, cos_[0], -sin_[0], 0, sin_[0], cos_[0] ]).reshape(3,3)
    Ay_ = np.asarray([ cos_[1], 0, sin_[1], 0, 1, 0, -sin_[1], 0, cos_[1] ]).reshape(3,3)
    Az_ = np.asarray([ cos_[2], -sin_[2], 0, sin_[2], cos_[2], 0, 0, 0, 1 ]).reshape(3,3)
    
    return np.matmul(np.matmul(Ax_,Ay_), Az_)
    
def apply_rpy(rpy, P):
    return np.matmul(euler_to_rotM(rpy), P.T).T

# should probably use a combination of gripper_rpyA and gripper_rpyB ?
tcp_marker_offset = np.asarray([ 0.005, 0.0, 0.0 ])
ur_marker_positions = gripper_positions_ + apply_rpy(gripper_rpyA, tcp_marker_offset)

if __name__ == '__main__':
    
    ## fit
        
    def transform_and_get_hypot_to_ur_positions(tf):
        P_ = tf[:3] + apply_rpy(tf[3:], motive_samples)
        hypot_ = np.sqrt(np.sum(np.square(ur_marker_positions-P_), axis=1))
        return hypot_, np.square(np.sum(hypot_))
        
    def rew_fn_singular(x):
        return transform_and_get_hypot_to_ur_positions(x)[1]

    def normalize_ang_ndarray(M):
        while True:
            I_ = M > np.pi
            if np.any(I_): M[I_] -= 2 * np.pi
            else: break
        while True:
            I_ = M < -np.pi
            if np.any(I_): M[I_] += 2 * np.pi
            else: break
        
    from scipy.optimize import minimize

    motive_ur_xyzrpy_ = minimize(rew_fn_singular, np.zeros(6)).x
    normalize_ang_ndarray(motive_ur_xyzrpy_[3:])
    print('  Got motive -> UR transform:')
    print(motive_ur_xyzrpy_)
        
    ## save        

    def flatten_matrix_to_string(M, decimal_places=6, separator=','):
        format_str_ = '{:.' + str(decimal_places) + 'f}'
        str_list_ = [ format_str_.format(x) for x in M.ravel().tolist() ]
        return separator.join(str_list_)

    def gen_euler_rot_comment(xyz_rpy):
        euler_ypr_ = 180.0/np.pi * np.asarray(xyz_rpy[3:][::-1])
        euler_ypr_str_ = flatten_matrix_to_string(euler_ypr_, 3, ' ')
        return '  <!-- Euler YPR (deg): ' + euler_ypr_str_ + ' -->'

    def gen_tf2_pub_node_str(xyz_rpy, parent_frame,child_frame):
        # static TF2 publisher
        xyz_ypr_ = np.copy(xyz_rpy); xyz_ypr_[3:] = xyz_ypr_[3:][::-1]
        node_str_ = '  <node name="motive_ur_stf" pkg="tf2_ros" type="static_transform_publisher" '
        args_str_ = 'args="' + flatten_matrix_to_string(xyz_ypr_, separator=' ') \
            + ' ' + parent_frame + ' ' + child_frame + '"/>'

        return node_str_ + args_str_

    def xyzrpy_to_4x4HTM(xyz_rpy):
        htm_ = np.empty([4,4])
        htm_[:3,:3] = euler_to_rotM(xyz_rpy[3:])
        htm_[:3,3] = xyz_rpy[:3]
        htm_[3,:] = [ 0.0, 0.0, 0.0, 1.0 ]
        
        return htm_

    def write_roslaunch(fp, xyz_rpy, parent_frame='ur_base_link', child_frame='mocap'):
        # Euler rotations in a comment for convenience
        euler_rot_comment_ = gen_euler_rot_comment(xyz_rpy)
        # static TF2 publisher
        tf2_pub_node_str_ = gen_tf2_pub_node_str(xyz_rpy, parent_frame,child_frame)
        
        # ./tf_node.py instance
        htm_str_ = flatten_matrix_to_string(xyzrpy_to_4x4HTM(xyz_rpy))
        tf_node_str_ = '  <node name="motive_ur_tf_node" pkg="motive_ur_calib" type="tf_node.py" output="screen">\n' \
            + '    <param name="htm" type="string" value="' + htm_str_ + '"/>\n  </node>'

        # write .launch file
        with open(fp, 'w') as f:
            f.write('<launch>\n\n')
            f.write(euler_rot_comment_); f.write('\n')
            f.write(tf2_pub_node_str_); f.write('\n\n')
            f.write(tf_node_str_); f.write('\n\n')
            f.write('</launch>')
            
    tf_fp_ = '/home/rll-ur5/catkin_ws/src/motive_ur_calib/motive_ur_tf.launch'
    write_roslaunch(tf_fp_, motive_ur_xyzrpy_)
