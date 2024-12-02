
from dofbot import DofbotEnv
import numpy as np
import time
import copy
from scipy.spatial.transform import Rotation as R
import time
import math

if __name__ == '__main__':
    env = DofbotEnv()
    env.reset()
    Reward = False

    '''
    constants here
    '''
    GRIPPER_DEFAULT_ANGLE = 20. / 180. * 3.1415
    GRIPPER_CLOSE_ANGLE = -20. / 180. * 3.1415
    GRIPPER_GRASP_ANGLE = -30. / 180. * 3.1415

    # define state machine
    INITIAL_STATE = 0
    GRASP_STATE = 1
    LIFT_STATE = 2
    PUT_STATE_1 = 3
    PUT_STATE_2 = 4
    BACK_STATE = 5
    current_state = INITIAL_STATE


    initial_jointposes = [1.57, 0., 1.57, 1.57, 1.57]

    # offset to grasp object
    obj_offset = [-0.023, -0.023, 0.09]
    obj_offset2 = [-0.032, 0.032, 0.13]
    obj_offset3 = [-0.025, 0.025, 0.005]
    obj_offset4 = [-0.025, 0.025, -0.15]

    block_pos, block_orn = env.get_block_pose()

    start_time = time.time()

    while not Reward:
        '''
        #获取物块位姿、目标位置和机械臂位姿，计算机器臂关节和夹爪角度，使得机械臂夹取绿色物块，放置到紫色区域。
        '''

        '''
        code here
        '''
        if current_state == INITIAL_STATE:
            # Move to initial position
            jointPoses = initial_jointposes
            env.dofbot_control(jointPoses, GRIPPER_DEFAULT_ANGLE)
            current_state = GRASP_STATE
        elif current_state == GRASP_STATE:
            # Move to block position
            pos_grasp = np.array(block_pos) + np.array(obj_offset)
            # orn_grasp = block_orn
            jointPoses = env.dofbot_setInverseKine(pos_grasp)
            env.dofbot_control(jointPoses, GRIPPER_DEFAULT_ANGLE)
            pos_t, orn_t = env.get_dofbot_pose()
            pos_b, orn_b = env.get_block_pose()
            dist_grasp = np.sqrt((pos_t[0] - pos_b[0]) ** 2 + (pos_t[1] - pos_b[1]) ** 2)
            if dist_grasp < 0.006:
                env.dofbot_control(jointPoses, GRIPPER_GRASP_ANGLE)
            if dist_grasp < 0.0015:
                current_state = LIFT_STATE
        elif current_state == LIFT_STATE:
            # Lift the block
            pos_lift = [0.2, 0.1, 0.18]
            jointPoses = env.dofbot_setInverseKine(pos_lift)
            env.dofbot_control(jointPoses, GRIPPER_CLOSE_ANGLE)
            pos_b, orn_b = env.get_block_pose()
            dist_lift = np.sqrt((pos_b[0] - pos_lift[0]) ** 2 + (pos_b[1] - pos_lift[1]) ** 2 + (pos_b[2] - pos_lift[2])**2)
            if dist_lift < 0.1:
                 env.dofbot_control(jointPoses, GRIPPER_DEFAULT_ANGLE)
                 current_state = PUT_STATE_1
        elif current_state == PUT_STATE_1:
            # Move to the top of target position
            pos_target = env.get_target_pose()
            pos_put = np.array(pos_target) + np.array(obj_offset3)
            jointPoses = env.dofbot_setInverseKine(pos_put)
            env.dofbot_control(jointPoses, GRIPPER_CLOSE_ANGLE)
            pos_t, orn_t = env.get_dofbot_pose()
            dist_put = np.sqrt((pos_t[0] - pos_put[0]) ** 2 + (pos_t[1] - pos_put[1]) ** 2 + (pos_t[2] - pos_put[2])**2)
            if dist_put < 0.03:
                current_state = PUT_STATE_2
        elif current_state == PUT_STATE_2:
            # Move to target position
            pos_target = env.get_target_pose()
            pos_move = np.array(env.get_target_pose()) + np.array(obj_offset4)
            jointPoses = env.dofbot_setInverseKine(pos_move)
            env.dofbot_control(jointPoses, GRIPPER_CLOSE_ANGLE)
            pos_b, orn_b = env.get_block_pose()
            dist_target = np.sqrt((pos_b[0] - pos_move[0]) ** 2 + (pos_b[1] - pos_move[1]) ** 2)
            final_dist,final_z_pos = env.get_final_data()
            if dist_target < 0.01:
                env.dofbot_control(jointPoses, GRIPPER_DEFAULT_ANGLE)

        Reward = env.reward()

        if Reward:
            final_dist, final_z_pos = env.get_final_data()
            print('success，at', final_dist, final_z_pos)
            break