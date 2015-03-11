#!/usr/bin/env python
import random

class QLearning():

  def __init__(self, actions, epsilon=0.1, alpha=0.2, gamma=0.9):
    self.q = {}
    self.actions = actions
    self.epsilon = epsilon
    self.alpha = alpha
    self.gamma = gamma

  def getQ(self, state, action):
    if state <= 0 and action == 1:
      reward = 1
    elif state >= 0 and action == 0:
      reward = 1
    else:
      reward = 0
    print 'getQ: state: ' + str(state) + ' action: ' + str(action) + ' Q: ' + str(self.q.get((state, action), reward))
    return self.q.get((state, action), reward)

  def learn(self, state1, action1, reward, state2):
    maxQNew = max([self.getQ(state2, a) for a in self.actions])
    self.learnQ(state1, action1, reward, reward + self.gamma*maxQNew)

  def learnQ(self, state, action, reward, value):
    oldv = self.q.get((state, action), None)
    if oldv is None:
      self.q[(state, action)] = reward
    else:
      self.q[(state, action)] = oldv + self.alpha * (value - oldv)

  def chooseAction(self, state):
    q = [self.getQ(state, a) for a in self.actions]
    maxQ = max(q)

    if random.random() > 1:
      best = [i for i in range(len(self.actions)) if q[i] == maxQ]
      i = random.choice(best)
    else:
      i = q.index(maxQ)

    action = self.actions[i]
    return action
