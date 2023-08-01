cmake_minimum_required(VERSION 3.0.2)
project(mpc_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  std_msgs
  tf
  sensor_msgs

)

set(acados_include "/home/muhammad/acados/include")
set(acados_lib "/home/muhammad/acados/lib")

include_directories(
include
${catkin_INCLUDE_DIRS}
scripts/c_generated_code
${PROJECT_SOURCE_DIR}/scripts/c_generated_code/bicycle_model_model
${PROJECT_SOURCE_DIR}/scripts/c_generated_code/bicycle_model_cost
${PROJECT_SOURCE_DIR}/scripts/c_generated_code
${acados_include}
${acados_include}/blasfeo/include
${acados_include}/hpipm/include
${acados_include}/acados/
${acados_include}/qpOASES_e
)


catkin_package(

)

link_directories(${PROJECT_SOURCE_DIR}/scripts/c_generated_code)

add_executable(mpc_planner src/main.cpp src/MPC_Planner.cpp src/MPC_ROS_Utils.cpp)
target_link_libraries(mpc_planner
	${PROJECT_SOURCE_DIR}/scripts/c_generated_code/libacados_ocp_solver_robot_model.so
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)


