#!/usr/bin/env python
import sys
from vrep import *
from util import *
from pid import PID

class SegwayController(object):

    def __init__(self, client):
        self.client = client

    def setup_body(self, body_name='body'):
        err, self.body = simxGetObjectHandle(self.client, body_name, simx_opmode_oneshot_wait)
        if err:
            log(self.client, 'ERROR GetObjectHandle code %d' % err)

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
            log(self.client, 'ERROR SetJointTargetVelocity code %d' % err)

    def setup_control(self, balance_controller):
        self.balance_controller = balance_controller

    def run(self):
        #segway_controller.set_target_velocities(1, -1)
        #log(client, 'Speed set to %d, %d' % (1, -1))

        ok = True
        while ok:
            # OPMODE Should should be changed to stream'n'buffer
            err, euler_angles = simxGetObjectOrientation(self.client, self.body, -1, simx_opmode_oneshot_wait)
            if err:
                log(self.client, 'ERROR GetObjectOrientation code %d' % err)
                ok = False
            else:
                log(self.client, euler_angles)
                alpha, beta, gamma = euler_angles
                control = self.balance_controller.control(beta)
                log(self.client, control)
                self.set_target_velocities(control, control)


if __name__ == '__main__':
    print '-- Please use simulation.py instead!'
