#!/usr/bin/python

import numpy as np
from scipy.optimize import minimize

### math

def euler_to_rotM(rpy):
    sin_ = np.sin(rpy); cos_ = np.cos(rpy)
    Ax_ = np.asarray([ 1, 0, 0, 0, cos_[0], -sin_[0], 0, sin_[0], cos_[0] ]).reshape(3,3)
    Ay_ = np.asarray([ cos_[1], 0, sin_[1], 0, 1, 0, -sin_[1], 0, cos_[1] ]).reshape(3,3)
    Az_ = np.asarray([ cos_[2], -sin_[2], 0, sin_[2], cos_[2], 0, 0, 0, 1 ]).reshape(3,3)
    
    return np.matmul(np.matmul(Ax_,Ay_), Az_)

def normalize_ang_ndarray(M):
    while True:
        I_ = M > np.pi
        if np.any(I_): M[I_] -= 2 * np.pi
        else: break
    while True:
        I_ = M < -np.pi
        if np.any(I_): M[I_] += 2 * np.pi
        else: break
        
def optimize_TF(err_fn, seed=np.zeros(6)):
    def err_fn_singular(x):
        return np.square(np.sum(err_fn(x)))
        
    tf_opt_ = minimize(err_fn_singular, seed).x
    normalize_ang_ndarray(tf_opt_[3:])
    
    print(' TF optimization finished. Mean error: %f' % np.mean(err_fn(tf_opt_)))
    
    return tf_opt_
        
### roslaunch file gen

def gen_euler_rot_comment(xyz_rpy):
    euler_ypr_ = 180.0/np.pi * np.asarray(xyz_rpy[3:][::-1])
    euler_ypr_str_ = flatten_matrix_to_string(euler_ypr_, 3, ' ')
    return '  <!-- Euler YPR (deg): ' + euler_ypr_str_ + ' -->'

def flatten_matrix_to_string(M, decimal_places=6, separator=','):
    format_str_ = '{:.' + str(decimal_places) + 'f}'
    str_list_ = [ format_str_.format(x) for x in M.ravel().tolist() ]
    return separator.join(str_list_)

def gen_tf2_pub_node_str(xyz_rpy, parent_frame,child_frame):
    # static TF2 publisher
    xyz_ypr_ = np.copy(xyz_rpy); xyz_ypr_[3:] = xyz_ypr_[3:][::-1]
    node_name_ = parent_frame + '_' + child_frame + '_stf'
    node_str_ = '  <node name="' + node_name_ + '" pkg="tf2_ros" type="static_transform_publisher" '
    args_str_ = 'args="' + flatten_matrix_to_string(xyz_ypr_, separator=' ') \
        + ' ' + parent_frame + ' ' + child_frame + '"/>'

    return node_str_ + args_str_

def write_TF_roslaunch(fp, xyz_rpy, parent_frame='ur_base_link',child_frame='motive', extras=[]):
    # Euler rotations in a comment for convenience
    euler_rot_comment_ = gen_euler_rot_comment(xyz_rpy)
    # static TF2 publisher
    tf2_pub_node_str_ = gen_tf2_pub_node_str(xyz_rpy, parent_frame,child_frame)

    # write .launch file
    with open(fp, 'w') as f:
        f.write('<launch>\n\n')
        f.write(euler_rot_comment_); f.write('\n')
        f.write(tf2_pub_node_str_); f.write('\n\n')
        for e in extras:
            f.write(e); f.write('\n\n')
        f.write('</launch>')
        
        

if __name__ == '__main__':
    
    ### fit with data from recording script

    from record_data import motive_samples_fp, pkg_dir

    samples_all_ = np.load(motive_samples_fp)
    ur_tcp_positions_ = samples_all_[:,:3]
    motive_samples_ = samples_all_[:,7:]
        
    def get_tf_hypot(tf):
        P_ = tf[:3] + np.matmul(euler_to_rotM(tf[3:]), motive_samples_.T).T
        return np.sqrt(np.sum(np.square(ur_tcp_positions_-P_), axis=1))

    tf_opt_ = optimize_TF(get_tf_hypot)
        
    ### save

    def xyzrpy_to_4x4HTM(xyz_rpy):
        htm_ = np.empty([4,4])
        htm_[:3,:3] = euler_to_rotM(xyz_rpy[3:])
        htm_[:3,3] = xyz_rpy[:3]
        htm_[3,:] = [ 0.0, 0.0, 0.0, 1.0 ]
        
        return htm_
          
    # ./tf_node.py instance
    htm_str_ = flatten_matrix_to_string(xyzrpy_to_4x4HTM(tf_opt_))
    tf_node_str_ = '  <node name="motive_ur_tf_node" pkg="motive_ur_calib" type="tf_node.py" output="screen">\n' \
        + '    <param name="htm" type="string" value="' + htm_str_ + '"/>\n  </node>'
    
    write_TF_roslaunch(pkg_dir+'motive_ur_tf.launch', tf_opt_, extras=[ tf_node_str_ ])
