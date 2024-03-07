#!/bin/bash

# docker exec -it --user docker_user strategic_node bash -c "source /opt/ros/humble/setup.bash; cd strategic_node; source install/setup.bash; cd src/behaviour_tree/behaviour_tree; python3 main.py"
docker exec -it --user docker_mpc mpc bash -c "source /home/docker_mpc/catkin_ws/env.sh; roslaunch mpc_planner node.launch"