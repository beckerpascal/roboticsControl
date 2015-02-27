#!/usr/bin/env python
import sys
from math import sqrt
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

    def setup_motors(self, left_motor_name="leftMotor", right_motor_name="rightMotor"):
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
        if err:  # != simx_return_ok
            log(self.client, 'ERROR SetJointTargetVelocity code %d' % err)

    def setup_control(self, balance_controller):
        self.balance_controller = balance_controller

    def zero_velocity_condition(self, lin_vel, rot_vel):
        """
        Simulation end condition. Expected to return True if the simulation
        should continue until the next cycle and False if a halt is required.
        """
        # Check if velocity is near zero (could cause issues on first cycle!)
        vel_tot = sqrt(reduce(lambda total, value: total + value**2,
                              lin_vel,
                              0.0))
        log(self.client, 'Total velocity: %f' % vel_tot)
        return vel_tot > 10.0 ** -5

    def body_height_condition(self, body_pos):
        """
        Simulation end condition. Expected to return True if the simulation
        should continue until the next cycle and False if a halt is required.
        """
        # Check if velocity is near zero (could cause issues on first cycle!)
        x, y, z = body_pos
        log(self.client, 'Body centroid height: %f' % z)
        return z > 0.04  # Wheel radius is 0.08m

    def run(self, condition=None):
        # Default condition to something sensible
        condition = condition if condition else self.body_height_condition

        niterations = 0.0
        cost = 0.0
        ok = True
        while ok:
            # OPMODE Should should be changed to stream'n'buffer
            err_rot, euler_angles = simxGetObjectOrientation(self.client, self.body, -1, simx_opmode_oneshot_wait)
            err_vel, lin_vel, rot_vel = simxGetObjectVelocity(self.client, self.body, simx_opmode_oneshot_wait)
            err_pos, position = simxGetObjectPosition(self.client, self.body, -1, simx_opmode_oneshot_wait)
            err = err_rot or err_vel
            if err:
                log(self.client, 'ERROR GetObjectOrientation/Velocity code %d' % err)
                ok = False
                break
            log(self.client, 'Euler angles: ' + str(euler_angles))
            # Beta is the one we're primarily interested in for balance control
            alpha, beta, gamma = euler_angles
            control = self.balance_controller.control(beta)
            log(self.client, 'Control value: ' + str(control))
            self.set_target_velocities(control, control)
            # Calculcate the cost (abs(ref-val))
            cost += abs(self.balance_controller.reference - beta)
            log(self.client, 'Cost on cycle: ' + str(cost))
            # Keep track of spent cycles (more is better!)
            # TODO fairly rudimentary!
            niterations += 1.0
            # Check for continuing
            ok = condition(position)  # lin_vel, rot_vel
        log(self.client, 'Cost (final): ' + str(cost))
        log(self.client, 'Cost (final 2): ' + str(cost / niterations))
        return cost / niterations


if __name__ == '__main__':
    print '-- Please use simulation.py instead!'
