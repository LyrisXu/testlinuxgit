#! /usr/bin/env python
import rospy
import yaml
import sys
import math
import numpy as np
import pandas as pd
import random

from itertools import product
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetLinkStateResponse
from std_srvs.srv import Empty

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import LinkStates, ModelStates
from std_msgs.msg import Float64

#  ATTENTION!! Use global var -- must declare -- global xxx
EPISODE = 10000
MOVE_STEP = 5
DURATION_TIME = 0.2

STATE_SPACE = []
ACTION_SPACE = []
STATE_NUM = 1369
qlist = []
# initial msg
msg = Float64()
msg1 = Float64()
msg2 = Float64()
t_action_u1 = Float64()
t_action_u2 = Float64()

roll = pitch = yaw = 0.0
theta = phi = omega = 0
PI = 3.1416

FINISH_FLAG = False

save_count=0


#############################  LATEST  602 ###################################
# CHANGE LEARNING RATE FROM 0.01 TO 0.1
# CHANGE ROSPY.RATE 10 TO 2

class SarsaTable:

    def __init__(self, actions, learning_rate=0.1, reward_decay=0.9, e_greedy=0.9):
        self.actions = actions  # ACTION_SPACE index's list
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon = e_greedy
        self.q_table = pd.DataFrame(columns=self.actions, dtype=np.float64)
        self.q_table.index.name = 'state'

        # state_str format: angle not radian!
        for i in range(STATE_NUM):
            state_str = str(STATE_SPACE[i])
            self.q_table = self.q_table.append(
                pd.Series(
                    [0] * len(actions),
                    index=self.q_table.columns,
                    name=state_str,
                )
            )
        # print(self.q_table)

    def choose_action(self, observation):
        # np.random.rand()
        if np.random.rand() < self.epsilon:
            # choose best action
            state_action = self.q_table.loc[observation, :]
            state_action = state_action.reindex(np.random.permutation(
                state_action.index))     # some actions have same value
            action = state_action.idxmax()
        else:
            # choose random action/ type: int64
            action = np.random.choice(self.actions)
            #action = random.choice(self.actions)
        return action

    # learning process
    # s str; a int
    def learn(self, s, a, r, s_, a_):
        q_predict = self.q_table.loc[s, a]
        print('q_predict:', q_predict)
        if not FINISH_FLAG:
            # next state is not terminal
            q_target = r + self.gamma * self.q_table.loc[s_, a_]
        else:
             # next state is terminal
            q_target = r

        print('q_target:', q_target)
        print('\n')
        #loction = np.where(self.q_table.index == s)
        #print('state location index', loction)
        self.q_table.loc[s, a] += self.lr * (q_target - q_predict)  # update


#######################################################################

def init_state_space():
    global STATE_SPACE
    for x in range(STATE_NUM):
        m = int(x / 37)
        n = int(x % 37)
        STATE_SPACE.append((-180 + m * 10, -180 + n * 10))
    #state_index_list = list(range(len(STATE_SPACE)))
    # print(STATE_SPACE)


def init_action_space():
    global ACTION_SPACE
    u1 = [-40, -20, 0, 20, 40]
    u2 = [-40, -20, 0, 20, 40]
    ACTION_SPACE = list(product(u1, u2))
    # print(ACTION_SPACE)
    #action_index_list = list(range(len(ACTION_SPACE)))


def get_observation_state():
    # use ROBOT model state to cal theta phi    / getmodelstate service 
    global phi, theta, omega
    # rospy.wait_for_service('/gazebo/get_model_state')
    # model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    # model_angle = model('robot', 'world')
    # qx = model_angle.pose.orientation.x
    # qy = model_angle.pose.orientation.y
    # qz = model_angle.pose.orientation.z
    # qw = model_angle.pose.orientation.w
    # qlist = [qx,qy,qz,qw]

    rospy.wait_for_service('/gazebo/get_link_state')
    # use base_link state to cal theta phi
    get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    base_orientation = get_link_state('base_link','world').link_state.pose.orientation
    #model_angle = model(link_name='base_link', reference_frame='world')

    qlist = [base_orientation.x, base_orientation.y, base_orientation.z, base_orientation.w]

    (roll, pitch, yaw) = euler_from_quaternion(qlist)
    # print('state rpy', roll, pitch, yaw)

    # rounding step by 10
    # roll z  pitch x  yaw y
    # roll X  pitch Y  yaw Z  ?  which one is right? in gazebo

    # phi angle from X, rotate along Y;
    # theta angle from Y, rotate along X
    # phi = int(round(yaw / PI * 18)) * 10
    # theta = int(round(pitch / PI * 18)) * 10
    phi = int(round(pitch / PI * 18)) * 10
    theta = int(round(roll / PI * 18)) * 10
    omega = int(round(yaw / PI * 18)) * 10
    observation = (phi, theta)

    print( 'theta phi omega:', theta, phi, omega)

    return observation


def take_action(action):
    # t_action is the selected action(u1,u2)
    global t_action_u1, t_action_u2
    t_action = ACTION_SPACE[action]
    print('----------   ACTION -----------')
    print('------Take which action?-------')
    print(t_action[0], t_action[1])
    #print('\n')

    # u1
    t_action_u1.data = t_action[0] / 180.0 * PI
    t_action_u2.data = t_action[1] / 180.0 * PI
    #a = t_action[0] / 180.0 * PI
    #print('taction radian:', t_action_u1.data, t_action_u2.data, a)

    msg1.data += t_action_u1.data
    msg2.data += t_action_u2.data
    print('msg data:', msg1, msg2)

    base_to_lower.publish(msg1)
    base_to_upper.publish(msg2)

    # need to put get_observation_state() here??
    s_ = observation_ = get_observation_state()
    reward = 900.0 / (abs(phi + 1) * abs(theta + 1)) - \
        0.1 * (abs(phi + 1) + abs(theta + 1))

    return s_, reward


def reset():

    global msg1, msg2
    msg1.data = 0.0
    msg2.data = 0.0
    base_to_lower.publish(msg1)
    base_to_upper.publish(msg2)

    rospy.wait_for_service('/gazebo/reset_simulation')
    rospy.wait_for_service('/gazebo/reset_world')
    reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    reset_simulation()
    reset_world()
    # rospy.loginfo('reset')
    print('reset\n')


# def link_state_callback(lmsg):
#     # rospy.loginfo('call')
#     a = lmsg.pose[1].position.x
#     #rospy.loginfo("right!!link! with X %f",a)


# def model_state_callback(mmsg):
#     # rospy.loginfo('call')
#     # orientation q
#     q = mmsg.pose[1].orientation
#     q_list = [q.x, q.y, q.z, q.w]
#     (roll,pitch,yaw) = euler_from_quaternion(q_list)
#     # print(yaw)
#     #rospy.loginfo('quar x:%f,y:%f,z:%f,w:%f',quarx,quary,quarz,quarw)


def episode_loop():

    reset()
    global msg1, msg2,save_count

    while not rospy.is_shutdown():
        rospy.loginfo('ROS is OK')

        for i in range(EPISODE):
            reset()
            save_count += 1
            n=save_count%1000
            if n == 0:
                RL.q_table.to_csv("qtable.csv")
                print('printed')

            print('##################   NEW EPISODE %d START  ##################'%(i))
            # rospy.loginfo('New Episode %d Starts', i)

            # msg1.data = 0.0
            # msg2.data = 0.0

            print(msg1,msg2)

            for move_index in range(MOVE_STEP):
                print('--------------- Move %d ---------------'%(move_index))
                # rospy.loginfo('-----Move %d-----', move_index)

                if move_index == MOVE_STEP - 1:
                    FINISH_FLAG = True
                else:
                    FINISH_FLAG = False  # move_index=0,1
                #print('FINISH?', FINISH_FLAG)
                # init ob (0,0)
                observation = get_observation_state()
                print('observation: ', observation)

                action = RL.choose_action(str(observation))
                #print('action: ', action)

                observation_, reward = take_action(action)

                print('ob_ , r: ', observation_, reward)
                #print('\n')

                action_ = RL.choose_action(str(observation_))

                # update q_table
                RL.learn(str(observation), action, reward,
                         str(observation_), action_)

                observation = observation_
                action = action_

                # rate.sleep()
                # Time for One episode = duration * move_step
                d = rospy.Duration(DURATION_TIME, 0)
                rospy.sleep(d)
            # print(RL.q_table)
        rospy.loginfo('FINISH ALL EPISODES')
        break


if __name__ == '__main__':
    # initial node
    rospy.init_node('talker')
    rospy.loginfo('Start Simulation!!')

    base_to_lower = rospy.Publisher(
        '/cat_low_controller/command', Float64, queue_size=1)
    base_to_upper = rospy.Publisher(
        '/cat_up_controller/command', Float64, queue_size=1)

    #rospy.Subscriber('/gazebo/link_states', LinkStates, link_state_callback)
    #rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback)

    rate = rospy.Rate(5)

    # initial state_space, action_space, q_table={0}
    init_state_space()
    init_action_space()

    # actions: [0,1,2...,24]
    RL = SarsaTable(actions=list(range(len(ACTION_SPACE))))

    try:
        episode_loop()
    except rospy.ROSInterruptException:
        pass
