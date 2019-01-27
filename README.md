# gym-rotors
A Gym Wrapper for RotorS simulator - https://github.com/WarrG3X/rotors_simulator

**WIP**


![drone-gif](https://github.com/WarrG3X/gym-rotors/blob/master/misc/video.gif)

Currently consists of -
 - BebopReachPositionEnv

These environments are primarily being developed for the Bebop2 Drone and can later be extended for other drones.

NOTE - We make use of this fork https://github.com/WarrG3X/rotors_simulator instead of the [original](https://github.com/ethz-asl/rotors_simulator) as that doesn't have the models for Bebop2.


## Dependencies
  - Ubuntu 16.04
  - ROS-Kinetic
  - Gazebo7
  - gym >= 0.10.9


## Basic Installation
Firstly install https://github.com/WarrG3X/rotors_simulator by follwing the instructions there.

Then to install gym-rotors,
```bash
git clone https://github.com/WarrG3X/gym-rotors
cd gym-rotors
pip install -e .
```
### Usage
```python
import gym
import gym_rotors
env=gym.make('BebopReachPosition-v0')
```

### References
https://github.com/erlerobot/gym-gazebo

https://bitbucket.org/theconstructcore/drone_training

https://github.com/ayushgaud/rotors_simulator
# mythesis_copy
