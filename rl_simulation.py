#!/usr/bin/env python
import random
import math
import sys
from time import *

from controller import *
from reinforcementLearner import *
from util import *
from vrep import *

# Python file to launch simulation and handling vrep

debug = 0

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

        if debug == 1:
            err, objs = simxGetObjects(client, sim_handle_all, simx_opmode_oneshot_wait)
            if err == simx_return_ok:
                log(client, 'Number of objects in the scene: %d' % len(objs))
            else:
                log(client, 'ERROR GetObjects code %d' % err)

        # initialize controller for handling vrep connection
        controller = SegwayController(client)
        controller.setup("body", "leftMotor", "rightMotor")

        # create a new reinforcement learner
        cart = ReinforcementLearner(controller)

        # initialize values
        p, oldp, rhat, r = 0, 0, 0, 0
        state, i, y, steps, failures, failed, startSim = 0, 0, 0, 0, 0, False, True

        while steps < cart.max_steps and failures < cart.max_failures:
            # start simulation in the first step
            if startSim == True:
                err = simxStartSimulation(controller.client, simx_opmode_oneshot_wait)
                now = int(time())
                # get start state
                cart.read_variables()
                state = cart.get_state()
                startSim = False
                if debug == 1:
                    print "----------------------- RESTART --------------------------"

            # guess new action depending on the weight of the current state
            action = (random.random()/((2**31) - 1) < (1.0 / (1.0 + math.exp(-max(-50, min(cart.action_weights[state], 50))))))
            if debug == 1:
                print "action: " + str(action)

            #update traces
            cart.action_weights_elig[state] += (1 - cart.lambda_w) * (y - 0.5)
            cart.critic_weights_elig[state] += (1 - cart.lambda_v)
            oldp = cart.critic_weights[state]     # remember prediction for the current state
            cart.do_action(action)                # do action
            cart.read_variables()                 # read new values TODO maybe a bit to close after doing action?!
            state = cart.get_state()              # get new x, dx, t, dt
            if debug == 1:
                print "state: " + str(state) + " x: " + str(cart.x) + " t: " + str(cart.t)

            # failure
            if state < 0:
                failed = True
                failures += 1
                print "Trial " + str(failures) + " was " + str(steps) + " steps or " + str(int(time()) - now) + " seconds"        
                steps = 0
                # restart simulation and get initial start values
                err = simxStopSimulation(controller.client, simx_opmode_oneshot_wait)  
                sleep(0.5)      
                state = cart.get_state()
                cart.read_variables()        
                r = -1  # reward = -1
                p = 0   # prediction of failure
                startSim = True
            else:     # no failure
                failed = False
                r = 0   # reward = 0
                p = cart.critic_weights[state]

            rhat = r + cart.gamma * p - oldp

            # update all weights
            cart.update_all_weights(rhat, failed)

            # going to next step
            steps += 1

        # finished the loop
        if failures == cart.max_failures:
            print "Pole not balanced. Stopping after " + str(failures) + " failures \n"
        else:
            print "Pole balanced successfully for at least " + str(steps) + " steps \n"
        print "critic weights: " + str(cart.critic_weights)
        print "action weights: " + str(cart.action_weights)