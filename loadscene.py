#!/usr/bin/env python
from vrep import *

print '-- Loading scene for V-REP'

simxFinish(-1)
client = simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if client != -1:
    simxStopSimulation(client, simx_opmode_oneshot_wait)

    # 1 for path relative to client
    err = simxLoadScene(client, 'vrep/segway.ttt', 1, simx_opmode_oneshot_wait)
    if err:
        print '-- ERROR loading scene: %d' % (err)
else:
    print '-- Failed connecting to remote API server'

print '-- Scene loaded!'
