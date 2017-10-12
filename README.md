# roboticsControl

Repository for the class 'Software Technology Course with a Varying Content' and the topic 'Introduction in Artificial Intelligence'.

This repository contains two different controlling algorithms. PID and Reinforcement learning (Q-Learning). The purpose of the lecture was to control a segway-like platform and compare the controlling behavior.

The result is that the PID controller is able to control the model. But it takes a lot of optimization to get the best parameters.
Also the reinforcement learning agent is able to control the model. It is learning really fast and after four to five failures it balances the segway multiple minutes in place.

Some useful links:

- [V-Rep Doku](http://coppeliarobotics.com/helpFiles/index.html)

# Installation

1. Download, extract and install the education version of VREP ([Download-Link](http://www.coppeliarobotics.com/downloads.html))
2. Clone this repository
3. Start VREP and open one of the scenes in the folder *vrep*
  - There are three different sizes of inverted pendulums. All work but have different behavior as mass and height changes
4. Run *python rl_simulation.py* from a console
5. See how the the reinforcement learning agent learns to balance the inverted pendulum

# More Information
If you want to have more information of the reinforcement agent, change the debug flag at line 14 in the file *rl_simulation.py* from 0 to 1

# PID
In this section you find the evaluated values for a good controlling with a PID controller

## PID tunings

A few different tunings found out to work more or less well.

### Bullet engine

#### |a|:
- 50ms - 13.7 0.199 1286
- 10ms - 56.25, 2.736, 1389

#### a**2 + v**2:
- 50ms - 13.76 0.1990 1609

#### a**2 + |x|
- 50ms - 13.7 0.2197 1194

#### a**2 + x**2
- 50ms - 36.72, 0.5333, 2036 (bad)
- 10ms - ?

#### a**2 + x**2 / t**2
- 10ms - 26.11, 1.971, 2925

#### |a| + |x| + |v| / t^2
- 10ms - 12.73 0.3582 3042

#### Pitch-delta
- Doomed to drift...
- 50ms - 0.5, 0.005, 12.5

#### Oldies but goldies
- 50ms: 21.000000 9.009500 16.550000
- 10ms: 79.0 19.9 41.4

### ODE engine

#### a**2 + x**2 / t**2
- 10ms - 324 3.24 4500

#### |a| + |x| + |v| / t^2
- 10ms - 80.77 0.5011 330.0

## Reinforcement Learning / Q-Learning

The basic Q-Learning approach is used to implement an easy and fast solution to control the segway.
