#!/usr/bin/env python
import sys
from time import sleep
from vrep import *
from util import *
from rl_qlearning import *

##############################################################################
# MAIN
##############################################################################
if __name__ == '__main__':
    print '-- Starting master client'
    simxFinish(-1)  # just in case, close all opened connections

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

        # # Create a simulation controller to run the tuning
        # simulation_controller = SimulationController(client)
        # # Defaults will do for the setup unless we change the model
        # simulation_controller.setup()
        # best_params = simulation_controller.run()
        # print str(best_params)

    else:
        print '-- Failed connecting to remote API server'

    #simxFinish(-1) # Buggy?
    print '-- Terminating master client'