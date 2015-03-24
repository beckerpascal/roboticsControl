#!/usr/bin/env bash
# Script directory
DIR=$( cd "$( dirname "$0" )" && pwd )
VREP_WAIT=10s
SCENE_WAIT=3s
# Execute headless V-REP
cd $DIR/V-REP/
echo ">> Launching V-REP"
./vrep.sh -h &
echo ">> V-REP launching, wait for $VREP_WAIT"
sleep $VREP_WAIT
echo ">> Waited for $VREP_WAIT!"
cd $DIR
echo ">> Load V-REP scene"
python loadscene.py
echo ">> Done loading scene"
echo ">> Wait for $SCENE_WAIT"
sleep $SCENE_WAIT
echo ">> Start twiddling"
./simulation.py
echo ">> End of twiddling"
echo ">> Now kill all!"
#pkill -TERM -P $$
pkill -TERM vrep
echo ">> All dead"
