#!/usr/bin/env python
import random

#approach based on the example from 'Reinforcement Learning: An Introduction' by Richard S. Sutton and Andrew G. Barto

one_degree = 0.0174532    # 2pi/360
six_degrees = 0.1047192
twelve_degrees = 0.2094384
fifty_degrees = 0.87266

max_distance = 2.4
max_speed = 0.5
max_angle = twelve_degrees

class QLearning():

  def __init__(self):
    self.n_states = 162
    self.alpha = 1000     # learning rate for action weights
    self.beta = 0.5       # learning rate for critic weights
    self.gamma = 0.95     # discount factor for critic
    self.lambda_w = 0.9   # decay rate for w
    self.lambda_v = 0.8   # decay rate for v
    self.max_failures = 100
    self.max_steps = 100000

    self.w = [0] * self.n_states
    self.v = [0] * self.n_states
    self.e = [0] * self.n_states
    self.xbar = [0] * self.n_states

    #position, velocity, angle, angle velocity
    self.x, self.dx, self.t, self.dt = 0, 0, 0, 0

  # matches the current state to an integer between 1 and n_states
  def get_state(self):
    state = 0

    # failed
    if self.x < -max_distance or self.x > max_distance or self.t < -max_angle or self.t > max_angle:
      return -1

    #position
    if self.x < -0.8:
      state = 0
    elif self.x < 0.8:
      state = 1
    else:
      state = 2

    #velocity
    if self.dx < -max_speed:
      state += 0
    elif self.dx < 0.5:
      state += 3
    else:
      state += 6

    #angle
    if self.t < -six_degrees:
      state += 0
    elif self.t < -one_degree:
      state += 9
    elif self.t < 0:
      state += 27
    elif self.t < one_degree:
      state += 36
    else:
      state += 45

    #angle velocity
    if self.dt < -fifty_degrees:
      state += 0
    elif self.dt < fifty_degrees:
      state += 54
    else:
      state = 108

    return state

  # reset state to zero
  def reset(self):
    self.x, self.dx, self.t, self.dt = 0, 0, 0, 0

  # executes action and updates x, dx, t, dt
  def do_action(self, action):
    #TODO execute action,
    print r


if __name__ == '__main__':

  cart = QLearning()

  p, oldp, rhat, r = 0, 0, 0, 0

  state, i, y, steps, failures, failed = 0, 0, 0, 0, 0, False

  # get start state
  state = cart.get_state()

  while steps < cart.max_steps and failures < cart.max_failures:

    y = random.random()

    cart.e[state] += (1 - cart.lambda_w) * (y - 0.5)
    cart.xbar[state] += (1 - cart.lambda_v)
    oldp = cart.v[state]
    state = cart.get_state()

    # failure
    if state < 0:
      failed = True
      failures += 1
      steps = 0
      cart.reset()
      state = cart.get_state()
      r = -1
      p = 0
    # no failure
    else:
      failed = 0
      r = 0
      p = cart.v[state]

    rhat = r + cart.gamma * p - oldp
    for i in range(cart.n_states):
      cart.w[i] += cart.alpha * rhat * cart.e[i]
      cart.v[i] += cart.beta * rhat * cart.xbar[i]

      if cart.v[i] < -1.0:
        cart.v[i] = 0

      if failed == True:
        cart.e[i] = 0
        cart.xbar[i] = 0
      else:
        cart.e[i] *= cart.lambda_w
        cart.xbar[i] *= cart.lambda_v

    steps += 1

