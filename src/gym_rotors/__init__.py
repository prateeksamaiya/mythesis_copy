from gym.envs.registration import register

for reward_type in ['sparse', 'dense']:
    suffix = 'Dense' if reward_type == 'dense' else ''
    kwargs = {
        'reward_type': reward_type,
    }


register(
    id='BebopReachPosition-v0',
    entry_point='gym_rotors.envs:BebopReachPositionEnv',
    kwargs=kwargs,
    max_episode_steps=50,
)
