from robot_model import robot_model
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
import math
import yaml
import create_solver
from math import pi, cos, sin , sqrt , exp , atan

def test_solver():

    factor = 1

    acados_solver = create_solver.Solver()
    nx = 4
    nu = 2
    ny = nx + nu
    n_obst = 40
    n_o = n_obst
    N = 20

    yref = np.zeros([N,ny+n_o])
    x_edge = np.array([ 15 , 20 ,     20.88, 25])*factor   # 22.7 , 22.2,
    y_edge = np.array([ 40 , 40,      40 , 40])*factor  #43.5, 42.7 ,

    x_ref_points = np.array([19 , 20.25 , 20.25 , 18.5 ])
    y_ref_points = np.array([39 , 39.1 , 40.5 , 40.8])

    theta_0 = 0
    theta_e = 3.14
    v_0 = 0
    v_e = 0
    v_path = 0.2


    x_ref = []
    y_ref = []
    theta = []
    theta_ref = []
    init_x = []
    init_y = []
    init_theta = []
    len_segments = []
    theta = np.append(theta , theta_0 )
    theta_ref = np.append(theta_ref , theta_0 )
    num_segment = len(x_ref_points)-1
    length_path = 0
    for i in range(num_segment):
        length_path = length_path + math.sqrt((x_ref_points[i+1]-x_ref_points[i])**2+(y_ref_points[i+1]-y_ref_points[i])**2)
        theta = np.append(theta , math.atan2(y_ref_points[i+1]-y_ref_points[i], x_ref_points[i+1]-x_ref_points[i]))
        len_segments = np.append(len_segments , math.sqrt((x_ref_points[i+1]-x_ref_points[i])**2+(y_ref_points[i+1]-y_ref_points[i])**2))

    step_line = length_path / N

    

    new_time_step = ((length_path)/(v_path*N)) * np.ones(N)

    k = 0
    x_ref = np.append(x_ref , x_ref_points[0])
    y_ref = np.append(y_ref , y_ref_points[0])
    for i in range(N+1):
        x_ref = np.append(x_ref , x_ref[i] + step_line * math.cos(theta[k+1]))
        y_ref = np.append(y_ref , y_ref[i] + step_line * math.sin(theta[k+1]))
        theta_ref = np.append(theta_ref , theta[k+1])
        d = math.sqrt((x_ref[-1]-x_ref_points[k])**2+(y_ref[-1]-y_ref_points[k])**2)
        if(d>len_segments[k] and k<(num_segment-1)):
            k = k+1
            x_ref[i] = x_ref_points[k]
            y_ref[i] = y_ref_points[k]
        elif (k>(num_segment-1)):
            break
    x0 = np.array([x_ref_points[0],y_ref_points[0],v_0,theta_0])

    init_x = x_ref[0:N+1]
    init_y = y_ref[0:N+1]
    init_theta = theta_ref[0:N+1]
    x_goal = np.array([init_x[-1],init_y[-1], v_e,theta_e])


##########################################
    paramters_static = [100 , 100] * n_obst


    paramters_static[0] = 20
    paramters_static[1] = 40
    paramters_static[2] = 20.88
    paramters_static[3] = 40
    parameter_values = np.concatenate([paramters_static])

    yref[:,0]=init_x[0:N]
    yref[:,1]=init_y[0:N]
    yref[:,2] = 0.2
    yref[:,3] = init_theta[0:N]

    a = np.zeros(n_o)
    yref_e = np.concatenate([x_goal,a])  
    x_traj_init = np.transpose([ yref[:,0] , yref[:,1] , yref[:,2], yref[:,3]])


    simX = np.zeros((N+1, nx))
    simU = np.zeros((N, nu))

    #acados_solver.constraints.x0 = x0
    for i in range(N):
        acados_solver.set(i,'p',parameter_values)
        acados_solver.set(i,'y_ref',yref[i])
        acados_solver.set(i, 'x', x_traj_init[i])
        acados_solver.set(i, 'u', np.array([0.0, 0.0]))
    acados_solver.set(N, 'p',  parameter_values)
    acados_solver.set(N, 'y_ref', yref_e)
    acados_solver.set(N, 'x', x_goal)
    acados_solver.set(0,'lbx', x0)
    acados_solver.set(0,'ubx', x0)
    acados_solver.options_set('rti_phase', 0)
    acados_solver.set_new_time_steps(new_time_step)

    t = time.time()
   
    status = acados_solver.solve()
    ROB_x = np.zeros([N+1,5])
    ROB_y = np.zeros([N+1,5])
    elapsed = 1000 * (time.time() - t)
    for i in range(N + 1):
        x = acados_solver.get(i, "x")
        
        print(i , x[0],x[1],x[2],x[3])
        simX[i,:]=x
        ROB_x[i,0] = simX[i,0] + 0.6 * cos(simX[i,3]-0.59)
        ROB_x[i,1] = simX[i,0] + 0.6 * cos(simX[i,3]+0.59)
        ROB_x[i,2] = simX[i,0] - 0.6 * cos(simX[i,3]-0.59)
        ROB_x[i,3] = simX[i,0] - 0.6 * cos(simX[i,3]+0.59)
        ROB_x[i,4] = simX[i,0] + 0.6 * cos(simX[i,3]-0.59)
        ROB_y[i,0] = simX[i,1] + 0.6 * sin(simX[i,3]-0.59)
        ROB_y[i,1] = simX[i,1] + 0.6 * sin(simX[i,3]+0.59)
        ROB_y[i,2] = simX[i,1] - 0.6 * sin(simX[i,3]-0.59)
        ROB_y[i,3] = simX[i,1] - 0.6 * sin(simX[i,3]+0.59)
        ROB_y[i,4] = simX[i,1] + 0.6 * sin(simX[i,3]-0.59)
        
    for i in range(N):
        u = acados_solver.get(i, "u")
        simU[i,:]=u
        print(i , u[0],u[1])
    print("status" , status)
    cost = acados_solver.get_cost()
    print("Elapsed time: {} ms".format(elapsed))
    print("cost", cost)


    fig, ax2 = plt.subplots(1)

    for i in range(0,4,2):
       circle1 = plt.Circle((paramters_static[i], paramters_static[i+1]),  0.34, color='r')

       ax2.add_patch(circle1)
    
    for i in range(N):
       ax2.plot([ROB_x[i,0],ROB_x[i,1],ROB_x[i,2],ROB_x[i,3],ROB_x[i,0]], [ROB_y[i,0],ROB_y[i,1],ROB_y[i,2],ROB_y[i,3],ROB_y[i,0]], color='k')
    

    for i in range(0,len(x_edge),2):
        ax2.plot([x_edge[i] , x_edge[i+1]], [y_edge[i] , y_edge[i+1]], linewidth=2, color='k')
    
 #   ax1.plot([10,50],[0,20])
    ax2.plot(simX[:, 0], simX[:, 1] , linewidth=4 , marker='o')
    ax2.plot(init_x , init_y, marker='o', color='g')
    ax2.axis('equal')
    ax2.set_xlim(18.5, 21)
    ax2.set_ylim(38.5, 41)
    ax2.set_title("MPC with Obstacle Avoidance")

   # plt.grid()
    plt.xlabel("x")
    plt.ylabel("y")
    

    tt = np.linspace(0,N*new_time_step,N+1)
    fig, ax3 = plt.subplots(4)
    ax3[0].plot(tt, simX[:,0])
    ax3[0].grid()
    ax3[1].grid()
    ax3[1].plot(tt, simX[:,1])
    ax3[2].plot(tt, simX[:,3])
    ax3[2].grid()
    ax3[3].plot(tt, simX[:,2])
    ax3[3].grid()
   # ax2[3].set_ylim([0, 1.1])
    ax3[0].set_title("x (m)")
    ax3[1].set_title("y (m)")
    ax3[2].set_title("theta (rad)")
    ax3[3].set_title("v (m/sec)")
    plt.show()
   

    return

    

test_solver()

