#!/usr/bin/env python
import sys
from vrep import *
from util import *
from controller import *


##############################################################################
# MAIN
##############################################################################
if __name__ == '__main__':
    print '-- Starting master client'
    simxFinish(-1) # just in case, close all opened connections

    # localhost:19997 connects to V-REP global remote API
    addr = '127.0.0.1'
    port = 19997
    print '-- Connecting to %s:%d' % (addr, port)
    client = simxStart(addr, port, True, True, 5000, 5)

    if client != -1:
        log(client, 'Master client connected to client %d at port %d' % (client, port))

        err, objs = simxGetObjects(client, sim_handle_all, simx_opmode_oneshot_wait)
        if err == simx_return_ok:
            log(client, 'Number of objects in the scene: %d' % len(objs))
        else:
            log(client, 'ERROR GetObjects code %d' % err)

        # Create a segway controller
        segway_controller = SegwayController(client)
        segway_controller.setup_body('body')
        segway_controller.setup_motors('leftMotor', 'rightMotor')

        balance_PID = PID(5.0, 0.1, 2.0, 0.0, 0.0)
        segway_controller.setup_control(balance_PID)

        # Start a simulation
        err = simxStartSimulation(client, simx_opmode_oneshot_wait)
        if err > 1:
            log(client, 'ERROR StartSimulation code %d' % err)
        segway_controller.run()

        simxFinish(-1)
    else:
        print '-- Failed connecting to remote API server'

    simxFinish(-1)
    print '-- Terminating master client'
