#!/usr/bin/env python

import vrep
import sys

print 'Program started'
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',int (sys.argv[1]),True,True,5000,5)
if clientID!=-1:
    print 'Connected to remote API server'
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait)
    if res==vrep.simx_return_ok:
        print 'Number of objects in the scene: ',len(objs)
    else:
        print 'Remote API function call returned with error code: ',res
    vrep.simxFinish(clientID)
else:
    print 'Failed connecting to remote API server'
print 'Program ended'