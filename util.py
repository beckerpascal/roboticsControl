#!/usr/bin/env python
import sys
from vrep import *
from math import sqrt

def log(client, message_, send=False):
    message = '-- %s' % message_
    print message
    if send:
        msg(client, message)


def msg(client, message):
    err = simxAddStatusbarMessage(client, message, simx_opmode_oneshot_wait)
    if err:
        print 'ERROR AddStatusBarMessage code %d' % err

def square(val):
    return val**2.0

def distN(n, vec, ref):
    return sqrt(sum(map(square, map(lambda zipd: zipd[0]-zipd[1], zip(vec[0:n], ref[0:n])))))

def dist2(vec, ref=[0.0, 0.0]):
    return distN(2, vec, ref)
