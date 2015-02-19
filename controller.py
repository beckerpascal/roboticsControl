#!/usr/bin/env python
import sys
from vrep import *
from util import *

class SegwayController(object):

    def __init__(self, client):
        self.client = client

    def setup_motors(self, left_motor_name, right_motor_name):
        err_l, self.left_motor = simxGetObjectHandle(self.client, left_motor_name, simx_opmode_oneshot_wait)
        err_r, self.right_motor = simxGetObjectHandle(self.client, right_motor_name, simx_opmode_oneshot_wait)
        err = err_l or err_r
        if err:
            log(self.client, 'ERROR GetObjectHandle code %d' % err)

    def setup_sensors(self, gyro, height):
        # Placeholder
        pass

    def set_target_velocities(self, left_vel, right_vel):
        err_l = None
        err_r = None
        if left_vel is not None:
            err_l = simxSetJointTargetVelocity(self.client, self.left_motor,
                                             left_vel, simx_opmode_oneshot_wait)
        if right_vel is not None:
            err_r = simxSetJointTargetVelocity(self.client, self.right_motor,
                                             right_vel, simx_opmode_oneshot_wait)
        err = err_l or err_r
        if err: # != simx_return_ok
            log(self.client, 'ERROR SetJointTargetVelocity code %d' % ('simxSetJointTargetVelocity'))


##############################################################################
# MAIN
##############################################################################

if __name__ == '__main__':
    print '-- Starting Python client'
    simxFinish(-1) # just in case, close all opened connections

    addr = '127.0.0.1'
    port = int(sys.argv[1] if len(sys.argv) > 1 else 19999)
    print '-- Connecting to %s:%d' % (addr, port)

    client = simxStart(addr, port, True, True, 5000, 5)

    if client != -1:
        print '-- Connected to remote API server with id: %d' % (client)
        log(client, 'Client connected at port %d' % port)

        segway_controller = SegwayController(client)
        segway_controller.setup_motors('leftMotor', 'rightMotor')

        segway_controller.set_target_velocities(1, -1)
        log(client, 'Speed set to %d, %d' % (1, -1))
    else:
        print '-- Failed connecting to remote API server'

    simxFinish(-1)
    print '-- Terminating Python client'
