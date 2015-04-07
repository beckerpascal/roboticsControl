#!/usr/bin/env python
import sys
from rl_qlearning import *

actions = 2 # accelerate/deccelerate
lastState = None
lastAction = None

if __name__ == '__main__':
  ai = None
  ai = QLearning(actions=range(actions), alpha=0.1, gamma=0.9, epsilon=0.1)


  # state = getStateInSimulation == angle of object
  for i in range(100):

    reward = -100
    if i % 10 == 0:
      reward = 100
    if lastState is not None:
      ai.learn(lastState, lastAction, reward, state)
      lastState = None

    state = random.randint(0,100) # calcState()
    action = ai.chooseAction(state)
    lastState = state
    lastAction = action 

    print 'i: ' + str(i) + ' lastState: ' + str(lastState) + ' lastAction: ' + str(lastAction)
  print ai.q 
  print ai.actions
