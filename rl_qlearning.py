#!/usr/bin/env python
import random
import math
import sys
from time import sleep
from controller import *
from ReinforcementLearner import *
from util import *
from vrep import *

#approach based on the example from 'Reinforcement Learning: An Introduction' by Richard S. Sutton and Andrew G. Barto

debug = 0

one_degree = 0.0174532    # 2pi/360
six_degrees = 0.1047192
twelve_degrees = 0.2094384
fifty_degrees = 0.87266

max_distance = 2.4
max_speed = 1
max_angle = twelve_degrees

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

    cart = ReinforcementLearner(controller)

    p, oldp, rhat, r = 0, 0, 0, 0

    state, i, y, steps, failures, failed, startSim = 0, 0, 0, 0, 0, False, True

    while steps < cart.max_steps and failures < cart.max_failures:
      # start simulation in the first step
      if startSim == True:
        err = simxStartSimulation(controller.client, simx_opmode_oneshot_wait)
        # get start state
        cart.read_variables()
        state = cart.get_state()
        startSim = False
        if debug == 1:
          print "XXXXXXXXXXXXXXXXXXXXX RESTART XXXXXXXXXXXXXXXXXXXXX"

      random1 = random.random()/((2**31) - 1)
      random2 = (1.0 / (1.0 + math.exp(-max(-50, min(cart.w[state], 50)))))
      if debug == 1:
        print "random: " + str(random1) + " random2: " + str(random2)
      action = (random1 < random2)

      #update traces
      cart.e[state] += (1 - cart.lambda_w) * (y - 0.5)
      cart.xbar[state] += (1 - cart.lambda_v)
      oldp = cart.v[state]      # remember prediction for the current state
      cart.do_action(action)    # do action
      cart.read_variables()     # read new values TODO maybe a bit to close after doing action?!
      state = cart.get_state()  # get new x, dx, t, dt
      if debug == 1:
        print "state: " + str(state) + " x: " + str(cart.x) + " t: " + str(cart.t)

      # failure
      if state < 0:
        failed = True
        failures += 1
        print "Trial " + str(failures) + " was " + str(steps) + " steps"        
        steps = 0
        err = simxStopSimulation(controller.client, simx_opmode_oneshot_wait)  
        sleep(0.5)      
        state = cart.get_state()
        cart.read_variables()        
        r = -1  # reward = -1
        p = 0   # prediction of failure
        startSim = True
      else: # no failure
        failed = False
        r = 0   # reward = 0
        p = cart.v[state]

      rhat = r + cart.gamma * p - oldp

      # update all weights
      for i in range(cart.n_states):
        cart.w[i] += cart.alpha * rhat * cart.e[i]
        cart.v[i] += cart.beta * rhat * cart.xbar[i]

        if cart.v[i] < -1.0:
          cart.v[i] = cart.v[i]

        if failed == True:
          cart.e[i] = 0
          cart.xbar[i] = 0
        else:
          cart.e[i] = cart.e[i] * cart.lambda_w
          cart.xbar[i] = cart.xbar[i] * cart.lambda_v

      steps += 1

    if failures == cart.max_failures:
      print "Pole not balanced. Stopping after " + str(failures) + " failures \n"
    else:
      print "Pole balanced successfully for at least " + str(steps) + " steps \n"
    print "e: " + str(cart.e)
    print "v: " + str(cart.v)
    print "w: " + str(cart.w)
    print "xbar: " + str(cart.xbar)
