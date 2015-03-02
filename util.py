#!/usr/bin/env python
import sys
from vrep import *

def log(client, message_, send=False):
    message = '-- %s' % message_
    print message
    if send:
        msg(client, message)


def msg(client, message):
    err = simxAddStatusbarMessage(client, message, simx_opmode_oneshot_wait)
    if err:
        print 'ERROR AddStatusBarMessage code %d' % err
