import os
import sys
import subprocess

import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from gym_rotors.envs import rotors_env

def goal_distance(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a - goal_b, axis=-1)

class BebopEnv(rotors_env.RotorsEnv):

    def __init__(self,launchfile,init_drone_pos,n_actions,reward_type):

        self.position_pub = rospy.Publisher('/bebop2/desired_position',Vector3, queue_size=1)
        self.launchfile = launchfile
        self.reward_type = reward_type
        self.distance_threshold = 0.05
        super(BebopEnv, self).__init__(launchfile,init_drone_pos,n_actions)

    def compute_reward(self, achieved_goal, goal, info, obs, params):
        # Compute distance between goal and the achieved goal.
        ret = 0
        d = goal_distance(achieved_goal, goal)
        if self.reward_type == 'sparse':
            ret = -(d > self.distance_threshold).astype(np.float32)
        else:
            ret = -d

    def _sample_goal(self):
        """Samples a new goal and returns it.
        """
        goal_pos = self.np_random.uniform(-2.0,2.0,size=3)
        goal_pos[2] = 1+goal_pos[2]/2
        return goal_pos

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        pos_msg = Vector3()
        pos_msg.x = action[0]
        pos_msg.y = action[1]
        pos_msg.z = action[2]

        self.position_pub.publish(pos_msg)

    def _get_obs(self):

        odometry_data = None
        while odometry_data is None:
            try:
                odometry_data = rospy.wait_for_message("/bebop2/ground_truth/odometry", Odometry)
            except:
                rospy.loginfo("Current drone front camera not ready yet, retrying for getting Odometry data")

        pose_data = None
        while pose_data is None:
            try:
                pose_data = rospy.wait_for_message('/bebop2/ground_truth/pose',Pose, timeout=5)
            except:
                rospy.loginfo("Current drone pose not ready yet, retrying for getting robot pose")


        drone_pos = np.array([pose_data.position.x,pose_data.position.y,pose_data.position.z])
        drone_quat = np.array([pose_data.orientation.w,pose_data.orientation.x,pose_data.orientation.y,pose_data.orientation.z])
        drone_linvel = np.array([odometry_data.twist.twist.linear.x,odometry_data.twist.twist.linear.y,odometry_data.twist.twist.linear.z]) 
        drone_angvel = np.array([odometry_data.twist.twist.angular.x,odometry_data.twist.twist.angular.y,odometry_data.twist.twist.angular.z])

        obs = np.concatenate([drone_pos, drone_quat, drone_linvel, drone_angvel])

        achieved_goal = drone_pos.copy()


        return {
            'observation': obs.copy(),
            'achieved_goal': achieved_goal.copy(),
            'desired_goal': self.goal.copy(),
        }


    def _is_success(self, achieved_goal, desired_goal):
        d = goal_distance(achieved_goal, desired_goal)
        return (d < self.distance_threshold).astype(np.float32)