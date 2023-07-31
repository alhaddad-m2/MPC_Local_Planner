# MPC_Local_Planner
Local Planner for a differential drive mobile robot using MPC and Acados with ROS Implementation. It has been tested on Husky Mobile Robot.

## Prerequisites
- Install [Acados](https://github.com/acados/acados) and make sure that it works by testing examples in exampls/acados_python
- The ROS Implamentation requires a global planner, occupancy grid and tf

## Run
- Replace <acados_dir> in env.sh with the directory of acados folder
- $ source env.sh
- In folder script Run: python3 create_solver in order to generate c-code solver
- $ roslaunch mpc_planner node.launch

## Tested
- Ubuntu 20.04 , 18.04
- [Results video](https://www.youtube.com/watch?v=_dgcIUr5awI)
 

