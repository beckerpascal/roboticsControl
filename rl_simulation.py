#!/usr/bin/env python
import sys
from time import sleep
from vrep import *
from util import *
from rl_qlearning import *
from controller import *

actions = 2 # accelerate/deccelerate
lastState = None
lastAction = None
max_angle = 70


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

        controller = SegwayController(client)
        controller.setup("body", "leftMotor", "rightMotor")

        roundCounter = 0        
        while roundCounter < 10:
            roundCounter += 1
            err = simxStartSimulation(controller.client, simx_opmode_oneshot_wait)
  
            controller.set_target_velocities(0, 0)

            state = 0
            stopCounter = 0

            while True:
                if max_angle < abs(state):
                    stopCounter = stopCounter + 1
                if stopCounter > 5:
                    print 'BREAK ========================'
                    break

                cur_vel = controller.get_current_ground_speed()
                cur_ang_vel = controller.get_current_angle_speed()
                print 'Vel: ' + str(cur_vel)
                print 'Angle vel: ' + str(cur_ang_vel)
                state = round(controller.get_angle_degree(1), 0)
                print str(int(state)) + ' degrees'
                reward = (80 - abs(state))/80
                if lastState is not None:
                  ai.learn(lastState, lastAction, reward, state)
                  lastState = None

                state = random.randint(0,100) # calcState()
                action = ai.chooseAction(state)
                print 'Reward: ' + str(reward)
                print 'Action: ' + str(action)
                vel = 10
                if action == 0:
                    controller.set_target_velocities(-vel, -vel)
                else:
                    controller.set_target_velocities(vel, vel)

                lastState = state
                lastAction = action 

            # print 'i: ' + str(i) + ' lastState: ' + str(lastState) + ' lastAction: ' + str(lastAction)

            err = simxStopSimulation(controller.client, simx_opmode_oneshot_wait)
            print ai.q

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