#!/usr/bin/env python
import sys

class QLearning(object):

	# List of all visited states
	states = []

  def __init__(self, client):
    self.client = client

	# What needs to be done to keep object upright (accelarate/deccelarate)
	def choose_action(_self, state):
		

	# How good is the next state, depends on position
	def get_reward(_self, state):


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


if __name__ == '__main__':
    print '-- Please use rl_simulation.py instead!'