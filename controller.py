#!/usr/bin/env python
import sys
from math import sqrt, log10, pi
from vrep import *
from util import *
from pid import PID

M_PI = 3.14159265359

class SegwayController(object):

    def __init__(self, client):
        self.client = client
        self.current_position = 0

    def setup(self, body_name="body", left_motor_name="leftMotor", right_motor_name="rightMotor"):
        self.setup_body(body_name)
        self.setup_motors(left_motor_name, right_motor_name)
        self.setup_streaming()

    def setup_body(self, body_name):
        err, self.body = simxGetObjectHandle(self.client, body_name, simx_opmode_oneshot_wait)
        if err:
            log(self.client, 'ERROR GetObjectHandle code %d' % err)

    def setup_motors(self, left_motor_name, right_motor_name):
        err_l, self.left_motor = simxGetObjectHandle(self.client, left_motor_name, simx_opmode_oneshot_wait)
        err_r, self.right_motor = simxGetObjectHandle(self.client, right_motor_name, simx_opmode_oneshot_wait)
        err = err_l or err_r
        if err:
            log(self.client, 'ERROR GetObjectHandle code %d' % err)
        self._send_target_velocities(0.0, 0.0, simx_opmode_oneshot_wait)

    def setup_wheels(self, left_wheel, right_wheel):
        err_l, self.left_wheel = simxGetObjectHandle(self.client, left_wheel, simx_opmode_oneshot_wait)
        err_r, self.right_wheel = simxGetObjectHandle(self.client, right_wheel, simx_opmode_oneshot_wait)
        err = err_l or err_r
        if err:
            log(self.client, 'ERROR GetObjectHandle code %d' % err)

    def setup_sensors(self, gyro, height):
        # Placeholder
        pass

    def setup_streaming(self):
        # Setup V-REP streaming
        simxGetObjectOrientation(self.client, self.body, -1, simx_opmode_streaming)
        simxGetObjectVelocity(self.client, self.body, simx_opmode_streaming)
        simxGetObjectPosition(self.client, self.body, -1, simx_opmode_streaming)
        simxGetObjectVelocity(self.client, self.left_wheel, simx_opmode_streaming)
        simxGetObjectVelocity(self.client, self.right_wheel, simx_opmode_streaming)

    def set_target_velocities(self, left_vel, right_vel):
        # Pause comms to sync the orders
        simxPauseCommunication(self.client, True)
        self._send_target_velocities(left_vel, right_vel, simx_opmode_streaming)
        # Re-enable comms to push the commands
        simxPauseCommunication(self.client, False)

    def _send_target_velocities(self, left_vel, right_vel, opmode):
        err_l = None
        err_r = None
        if left_vel is not None:
            err_l = simxSetJointTargetVelocity(self.client, self.left_motor,
                                               left_vel, opmode)
        if right_vel is not None:
            err_r = simxSetJointTargetVelocity(self.client, self.right_motor,
                                               right_vel, opmode)
        err = err_l or err_r
        if err > 1:
            log(self.client, 'ERROR SetJointTargetVelocity code %d' % err)

    def setup_control(self, balance_controller):
        self.balance_controller = balance_controller

#####  END CONDITIONS #########################################################

    def zero_velocity_condition(self, lin_vel, rot_vel):
        # Check if velocity is near zero (could cause issues on first cycle!)
        vel_tot = sqrt(reduce(lambda total, value: total + value**2,
                              lin_vel,
                              0.0))
        log(self.client, 'Total velocity: %f' % vel_tot)
        return vel_tot > 10.0 ** -5

    def body_height_condition(self, body_pos):
        x, y, z = body_pos
        return z > 0.04  # Wheel radius is 0.08m

    def get_current_angle(self):
        err_or, angle = simxGetObjectOrientation(self.client, self.body, -1, simx_opmode_streaming)
        if err_or > 0:
            print "Error while getting angle"
        # Returns angle (RPY)
        return angle

    def get_current_position(self):
        err_pos, pos = simxGetObjectPosition(self.client, self.body, -1, simx_opmode_streaming)
        if err_pos > 0:
            print "Error while getting position"
        return pos

    def get_current_ground_speed(self):
        err_vel, lin_vel, rot_vel = simxGetObjectVelocity(self.client, self.body, simx_opmode_streaming)
        if err_vel > 0:
            print "Error while getting velocity"
        return lin_vel

    def get_current_angle_speed(self, part_name="rightMotor"):
        err_vel, lin_vel, rot_vel = simxGetObjectVelocity(self.client, self.body, simx_opmode_streaming)
        if err_vel > 0:
            print "Error while getting velocity"
        return rot_vel

    def simulation_run_condition(self, simulation_time, body_pos):
        if simulation_time < 100:
            return True
        else:
            x, y, z = body_pos
            # Wheel radius 0.25m, box length 1.5m -> pos @1m, time in ms
            height_condition = 0.3 < z < 1.1
            drive_condition = sqrt(x**2 + y**2) < 2.0
            time_condition = simulation_time < 30000
            return height_condition and drive_condition and time_condition

    def run(self, condition=None):
        # Default condition to something sensible
        condition = condition if condition else self.simulation_run_condition

        simulation_time_current = 0
        simulation_time_previous = 0  # ms
        cost = 0.0
        ok = True

        self.setup_streaming()

        while ok and simxGetConnectionId(self.client) != -1:
            simxPauseCommunication(self.client, True)
            err_rot, euler_angles = simxGetObjectOrientation(self.client, self.body, -1, simx_opmode_buffer)
            err_vel, lin_vel, rot_vel = simxGetObjectVelocity(self.client, self.body, simx_opmode_buffer)
            err_pos, position = simxGetObjectPosition(self.client, self.body, -1, simx_opmode_buffer)
            err_wheel_left_vel, wheel_left_lin_vel, wheel_left_rot_vel = simxGetObjectVelocity(self.client, self.left_wheel, simx_opmode_streaming)
            err_wheel_right_vel, wheel_right_lin_vel, wheel_right_rot_vel = simxGetObjectVelocity(self.client, self.right_wheel, simx_opmode_streaming)
            simxPauseCommunication(self.client, False)

            err = err_rot or err_vel or err_pos or err_wheel_left_vel or err_wheel_right_vel
            if err > 1:
                print "-- No data right now!"
                continue

            # Check whether new commands have been executed
            simulation_time_current = simxGetLastCmdTime(self.client)
            if simulation_time_previous == simulation_time_current:
                continue
            # Calculate dt now that we have times available
            dt = simulation_time_current - simulation_time_previous
            # Store the time spent until last fetch'd value
            simulation_time_previous = simulation_time_current

            # Calculate and set control
            roll, pitch, yaw = euler_angles
            droll, dpitch, dyaw = rot_vel
            vx, vy, vz = lin_vel
            x, y, z = position
            dtilt_left = tilt_from_rp(wheel_left_rot_vel[0], wheel_left_rot_vel[1])
            dtilt_right = tilt_from_rp(wheel_right_rot_vel[0], wheel_right_rot_vel[1])
            dtilt = (dtilt_left + dtilt_right) / 2
            control = self.balance_controller.control(pitch, dt)
            self.set_target_velocities(control, control)

            # Calculcate the current cost and sum it to the total only after 100ms to prevent a weird spike on first cycles
            if simulation_time_current > 100:
                cost += abs(tilt_from_rp(roll, pitch)) + abs((pi/2)*x) + abs(dtilt)

            # Check for continuing
            ok = condition(simulation_time_current, position)

        return (log10(cost / max(simulation_time_current, 1)**2), simulation_time_current)


# log(self.client, 'Euler angles: ' + str(euler_angles))
# log(self.client, 'Control value: ' + str(control))
# log(self.client, 'Cost on cycle: ' + str(cost))
# log(self.client, 'Cost (final): ' + str(cost))
# log(self.client, 'Cost (final 2): ' + str(cost / niterations))

if __name__ == '__main__':
    print '-- Please use simulation.py instead!'
