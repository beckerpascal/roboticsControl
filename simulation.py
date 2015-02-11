#!/usr/bin/env python
import sys
from vrep import *

def log(client, message):
    err = simxAddStatusbarMessage(client, '-- ' + message, simx_opmode_oneshot_wait)
    if err:
        print 'AddStatusBarMessage errors with: %d' % err


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

        res, objs = simxGetObjects(client, sim_handle_all, simx_opmode_oneshot_wait)
        if res == simx_return_ok:
            print 'Number of objects in the scene: ', len(objs)
        else:
            print 'Remote API function call returned with error code: ', res

        res, left_motor = simxGetObjectHandle(client, 'leftMotor', simx_opmode_oneshot_wait)
        if res == simx_return_ok:
            log(client, 'leftMotor handle: %d' % left_motor)
        else:
            print 'Remote API function call returned with error code: ', res
        res, right_motor = simxGetObjectHandle(client, 'rightMotor', simx_opmode_oneshot_wait)
        if res == simx_return_ok:
            log(client, 'rightMotor handle: %d' % right_motor)
        else:
            print 'Remote API function call returned with error code: ', res

        for i in xrange(20):
            errLeft = simxSetJointForce(client, left_motor, 100000, simx_opmode_oneshot_wait)
            errRight = simxSetJointForce(client, right_motor, -100000, simx_opmode_oneshot_wait)
            log(client, str(errLeft) + ' ' + str(errRight))
        simxFinish(-1)
    else:
        print '-- Failed connecting to remote API server'
    print '-- Terminating Python client'
