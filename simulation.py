#!/usr/bin/env python
import sys
import vrep

print '-- Starting Python client'
vrep.simxFinish(-1) # just in case, close all opened connections

addr = '127.0.0.1'
port = int(sys.argv[1] if len(sys.argv) > 1 else 19999)
print '-- Connecting to %s:%d' % (addr, port)

client = vrep.simxStart(addr, port, True, True, 5000, 5)

if client != -1:
    print '-- Connected to remote API server with id: %d' % (client)
    errorCode = vrep.simxAddStatusbarMessage(client, '-- Client connected at port %d' % port, vrep.simx_opmode_oneshot_wait)

    res, objs = vrep.simxGetObjects(client, vrep.sim_handle_all, vrep.simx_opmode_oneshot_wait)
    if res == vrep.simx_return_ok:
        print 'Number of objects in the scene: ', len(objs)
    else:
        print 'Remote API function call returned with error code: ', res
    vrep.simxFinish(-1)
else:
    print '-- Failed connecting to remote API server'
print '-- Terminating Python client'
