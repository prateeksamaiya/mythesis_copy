from gym import utils
from gym_rotors.envs import bebop_env

class BebopReachPositionEnv(bebop_env.BebopEnv, utils.EzPickle):
    def __init__(self, reward_type='sparse'):
        bebop_env.BebopEnv.__init__(self, launchfile='bebop_reach_position.launch',
        init_drone_pos=[0,0,0.5],n_actions=3,reward_type=reward_type)
        utils.EzPickle.__init__(self)