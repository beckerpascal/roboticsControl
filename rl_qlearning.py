#!/usr/bin/env python
import random
import math
import sys
from controller import *
from util import *
from vrep import *

#approach based on the example from 'Reinforcement Learning: An Introduction' by Richard S. Sutton and Andrew G. Barto

one_degree = 0.0174532    # 2pi/360
six_degrees = 0.1047192
twelve_degrees = 0.2094384
fifty_degrees = 0.87266

max_distance = 2.4
max_speed = 0.5
max_angle = twelve_degrees

class QLearning():

  def __init__(self, controller):
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

    self.controller = controller

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
      state += 18
    elif self.t < one_degree:
      state += 27
    elif self.t < six_degrees:
      state += 36
    else:
      state += 45

    #angle velocity
    if self.dt < -fifty_degrees:
      state += 0
    elif self.dt < fifty_degrees:
      state += 54
    else:
      state += 108

    return state

  # reset state to zero
  def reset(self):
    self.x, self.dx, self.t, self.dt = 0, 0, 0, 0

  # executes action and updates x, dx, t, dt
  def do_action(self, action):
    if action > 0:
      self.controller.set_target_velocities(max_speed,max_speed)
    else:
      self.controller.set_target_velocities(-max_speed,-max_speed)

    self.x = self.controller.get_current_position()[0]
    self.dx = self.controller.get_current_ground_speed()[0]
    self.t = self.controller.get_angle_degree(1)
    self.dt = self.controller.get_current_angle_speed()[1]

if __name__ == '__main__':

  print '-- Starting master client'
  simxFinish(-1)  # just in case, close all opened connections

  # localhost:19997 connects to V-REP global remote API
  addr = '127.0.0.1'
  port = 19997
  print '-- Connecting to %s:%d' % (addr, port)
  client = simxStart(addr, port, True, True, 5000, 5)

  if client != -1:
    log(client, 'Master client connected to client %d at port %d' % (client, port))

    err, objs = simxGetObjects(client, sim_handle_all, simx_opmode_oneshot_wait)
    if err == simx_return_ok:
        log(client, 'Number of objects in the scene: %d' % len(objs))
    else:
        log(client, 'ERROR GetObjects code %d' % err)

    controller = SegwayController(client)
    controller.setup("body", "leftMotor", "rightMotor")

    cart = QLearning(controller)

    p, oldp, rhat, r = 0, 0, 0, 0

    state, i, y, steps, failures, failed = 0, 0, 0, 0, 0, False

    # get start state
    state = cart.get_state()

    while steps < cart.max_steps and failures < cart.max_failures:
      err = simxStartSimulation(controller.client, simx_opmode_oneshot_wait)

      action = (random.random() < (1.0 / (1.0 + math.exp(-max(-50, min(cart.w[state], 50))))))

      cart.e[state] += (1 - cart.lambda_w) * (y - 0.5)
      cart.xbar[state] += (1 - cart.lambda_v)
      oldp = cart.v[state]
      cart.do_action(action)
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
        err = simxStopSimulation(controller.client, simx_opmode_oneshot_wait)

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

