import os
import copy
import sys
import os
import signal
import subprocess
import time
import random
import numpy as np

import rospy
import rospkg
from std_srvs.srv import Empty

import gym
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gym import error, spaces
from gym.utils import seeding

from gym_rotors.envs.gazebo_connection import GazeboConnection


class RotorsEnv(gym.GoalEnv):
    def __init__(self,launchfile,init_drone_pos,n_actions):

        assert len(init_drone_pos)==3

        # #Init Port
        # random_number = random.randint(10000, 15000)
        # self.port_ros = str(random_number) #os.environ["ROS_PORT_SIM"]
        # self.port_gazebo = str(random_number+1) #os.environ["ROS_PORT_SIM"]
        # os.environ["ROS_MASTER_URI"] = "http://localhost:"+self.port_ros
        # os.environ["GAZEBO_MASTER_URI"] = "http://localhost:"+self.port_gazebo
        # print("ROS_MASTER_URI=http://localhost:"+self.port_ros + "\n")
        # print("GAZEBO_MASTER_URI=http://localhost:"+self.port_gazebo + "\n")

        # self.ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))
        # rospack = rospkg.RosPack()

        # if launchfile.startswith("/"):
        #     fullpath = launchfile
        # else:
        #     fullpath = os.path.join(rospack.get_path('rotors_gazebo'),"launch",launchfile)
        # if not os.path.exists(fullpath):
        #     raise IOError("File "+fullpath+" does not exist")

        #Action Space
        self.action_space = spaces.Box(-1., 1., shape=(n_actions,), dtype='float32')

        #target_postion_service_proxy
        self.target_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        #Seed
        self.seed()
        self.goal = self._sample_goal()

        # #Drone Initial Pos
        # x = "x:="+str(init_drone_pos[0])
        # y = "y:="+str(init_drone_pos[1])
        # z = "z:="+str(init_drone_pos[2])
        # #Target Initial Pos
        # tx = "tx:="+str(self.goal[0])
        # ty = "ty:="+str(self.goal[1])
        # tz = "tz:="+str(self.goal[2])

    
        # self._roslaunch = subprocess.Popen([sys.executable, os.path.join(self.ros_path, b"roslaunch"), "-p", self.port_ros, fullpath,
        #                                     x,y,z,tx,ty,tz])
        # print ("Gazebo launched!")

        # self.gzclient_pid = 0
        self.gconn = GazeboConnection()

        # time.sleep(6)


        # ready = int(subprocess.check_output(["rosparam", "get",'/bebop2/position_step_node/ready']))
        # while ready==0:
        #     time.sleep(0.5)
        #     int(subprocess.check_output(["rosparam", "get",'/bebop2/position_step_node/ready']))
        # print("READY!")

    @property
    def dt(self):
        raise NotImplementedError()

    # Env methods
    # ----------------------------

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        # self.gconn.unpauseSim()
        self._set_action(action)
        # time.sleep(1)
        # self.gconn.pauseSim()
        self._step_callback()
        obs = self._get_obs()
        obsTemp = obs['observation']
        params = {}

        done = False
        info = {
            'is_success': self._is_success(obs['achieved_goal'], self.goal),
        }
        reward = self.compute_reward(obs['achieved_goal'], self.goal, info, obsTemp, params)

        return obs, reward, done, info

    def reset_target_position(self):

        msg = ModelState() 
        msg.pose.position.x = random.uniform(-2,2)
        msg.pose.position.y = random.uniform(-2,2)
        msg.pose.position.z = random.uniform(0.5,2)
        msg.model_name = "target"
        msg.reference_frame = "world"
         
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self.target_proxy(msg)
        except rospy.ServiceException as e:
            print ("/gazebo/set_model_state service call failed")

    def reset(self,goal=None):
        # Attempt to reset the simulator. Since we randomize initial conditions, it
        # is possible to get into a state with numerical issues (e.g. due to penetration or
        # Gimbel lock) or we may not achieve an initial condition (e.g. an object is within the hand).
        # In this case, we just keep randomizing until we eventually achieve a valid initial
        # configuration.
        # obs = self._get_obs()
        # return obs
        
        # 1st: resets the simulation to initial values
        self.gconn.resetSim()

        # target set at random position
        self.reset_target_position()

        # 2nd: Unpauses simulation
        self.gconn.unpauseSim()

        # 4th: takes an observation of the initial condition of the robot
        # observation = self._get_obs()

        # # 5th: pauses simulation
        # self.gconn.pauseSim()

        # return observation


    def close(self):
        raise NotImplementedError()

    def render(self, mode='human'):
        raise NotImplementedError()

    def _get_viewer(self):

        raise NotImplementedError()

    # Extension methods
    # ----------------------------

    def _reset_sim(self):
        """Resets a simulation and indicates whether or not it was successful.
        If a reset was unsuccessful (e.g. if a randomized state caused an error in the
        simulation), this method should indicate such a failure by returning False.
        In such a case, this method will be called again to attempt a the reset again.
        """
        raise NotImplementedError()

    def _get_obs(self):
        """Returns the observation.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _is_success(self, achieved_goal, desired_goal):
        """Indicates whether or not the achieved goal successfully achieved the desired goal.
        """
        raise NotImplementedError()

    def _sample_goal(self):
        """Samples a new goal and returns it.
        """
        raise NotImplementedError()

    def _env_setup(self, initial_qpos):
        """Initial configuration of the environment. Can be used to configure initial state
        and extract information from the simulation.
        """
        pass

    def _viewer_setup(self):
        """Initial configuration of the viewer. Can be used to set the camera position,
        for example.
        """
        pass

    def _render_callback(self):
        """A custom callback that is called before rendering. Can be used
        to implement custom visualizations.
        """
        pass

    def _step_callback(self):
        """A custom callback that is called after stepping the simulation. Can be used
        to enforce additional constraints on the simulation state.
        """
        pass
