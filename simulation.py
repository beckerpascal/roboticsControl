#!/usr/bin/env python

import sys
import vrep

print 'Program started'
vrep.simxFinish(-1) # just in case, close all opened connections

addr = '127.0.0.1'
port = int(sys.argv[1] if len(sys.argv) > 1 else 19999)
print 'Connecting to %s:%d' % (addr, port)

clientID=vrep.simxStart(addr, port, True, True, 5000, 5)
if clientID!=-1:
    print 'Connected to remote API server with id: %d' % (clientID,)
    errorCode = vrep.simxAddStatusbarMessage(clientID, 'Python says hi.', vrep.simx_opmode_oneshot_wait)
    print 'AddStatusBarMessage status: %d' % errorCode
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait)
    if res==vrep.simx_return_ok:
        print 'Number of objects in the scene: ',len(objs)
    else:
        print 'Remote API function call returned with error code: ',res
    print 'asd'
    vrep.simxFinish(-1)
    print 'asd'
else:
    print 'Failed connecting to remote API server'
print 'Program ended'
