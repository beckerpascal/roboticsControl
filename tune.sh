#!/usr/bin/env bash
# Script directory
DIR=$( cd "$( dirname "$0" )" && pwd )
SCENE="segway_big.ttt"
VREP_WAIT=5s
# Execute headless V-REP
cd $DIR/V-REP/
echo ">> Launching V-REP"
./vrep.sh -h $DIR/vrep/$SCENE &
echo ">> V-REP launching, wait for $VREP_WAIT"
sleep $VREP_WAIT
echo ">> Waited for $VREP_WAIT!"
echo ">> Start twiddling"
cd $DIR
./simulation.py "$@"
echo ">> End of twiddling"
echo ">> Now kill all!"
pkill -TERM -P $(pgrep -P $$)
sleep 0.1s
echo ">> All dead"
exit
