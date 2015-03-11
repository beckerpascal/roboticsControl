#!/usr/bin/env python
import sys
from time import sleep
from vrep import *
from util import *
from rl_qlearning import *

actions = 2 # accelerate/deccelerate
lastState = None
lastAction = None
M_PI = 3.14159265359    

def set_target_velocities(self, left_vel, right_vel):
    err_l = None
    err_r = None
    # Pause comms to sync the orders
    simxPauseCommunication(self.client, True)
    if left_vel is not None:
        err_l = simxSetJointTargetVelocity(self.client, self.left_motor,
                                           left_vel, simx_opmode_streaming)
    if right_vel is not None:
        err_r = simxSetJointTargetVelocity(self.client, self.right_motor,
                                           right_vel, simx_opmode_streaming)
    err = err_l or err_r
    if err > 1:
        log(self.client, 'ERROR SetJointTargetVelocity code %d' % err)
    # Re-enable comms to push the commands
    simxPauseCommunication(self.client, False)

##############################################################################
# MAIN
##############################################################################
if __name__ == '__main__':

    ai = None
    ai = QLearning(actions=range(actions), alpha=0.1, gamma=0.9, epsilon=0.1)

    print '-- Starting master client'
    simxFinish(-1)  # just in case, close all opened connections

    # localhost:19997 connects to V-REP global remote API
    addr = '127.0.0.1'
    port = 19997
    print '-- Connecting to %s:%d' % (addr, port)
    client = simxStart(addr, port, True, True, 5000, 5)

    if client != -1:
        log(client, 'Master client connected to client %d at port %d' % (client, port))

        #setupBody
        err, body = simxGetObjectHandle(client, "body", simx_opmode_oneshot_wait)
        if err:
            log(client, 'ERROR GetObjectHandle code %d' % err)

        #setupMotors
        err_l, left_motor = simxGetObjectHandle(client, "left_motor", simx_opmode_oneshot_wait)
        err_r, right_motor = simxGetObjectHandle(client, "right_motor", simx_opmode_oneshot_wait)
        err = err_l or err_r
        if err:
            log(client, 'ERROR GetObjectHandle code %d' % err)


        err, objs = simxGetObjects(client, sim_handle_all, simx_opmode_oneshot_wait)
        if err == simx_return_ok:
            log(client, 'Number of objects in the scene: %d' % len(objs))
        else:
            log(client, 'ERROR GetObjects code %d' % err)

        err = simxStartSimulation(client, simx_opmode_oneshot_wait)

        err_l = simxSetJointTargetVelocity(client, left_motor,
                                               -100, simx_opmode_streaming)
        err_r = simxSetJointTargetVelocity(client, right_motor,
                                               -100, simx_opmode_streaming)

        for i in range(300):

            
            err_or, reward = simxGetObjectOrientation(client,body,-1,simx_opmode_oneshot_wait)
            state = round(reward[1]*180/M_PI, 0)
            print str(int(state)) + ' degrees'
            # if i % 10 == 0:
            #   reward = 100
            # if lastState is not None:
            #   ai.learn(lastState, lastAction, reward, state)
            #   lastState = None

            # state = random.randint(0,100) # calcState()
            # action = ai.chooseAction(state)
            # lastState = state
            # lastAction = action 

            # print 'i: ' + str(i) + ' lastState: ' + str(lastState) + ' lastAction: ' + str(lastAction)

        err = simxStopSimulation(client, simx_opmode_oneshot_wait)


        # # Create a simulation controller to run the tuning
        # simulation_controller = SimulationController(client)
        # # Defaults will do for the setup unless we change the model
        # simulation_controller.setup()
        # best_params = simulation_controller.run()
        # print str(best_params)

    else:
        print '-- Failed connecting to remote API server'

    #simxFinish(-1) # Buggy?
    print '-- Terminating master client'