#!/usr/bin/env python
import sys
from vrep import *

def log(client, message_):
    message = '-- %s' % message_
    print message
    msg(client, message)


def msg(client, message):
    err = simxAddStatusbarMessage(client, message, simx_opmode_oneshot_wait)
    if err:
        print 'ERROR AddStatusBarMessage code %d' % err
