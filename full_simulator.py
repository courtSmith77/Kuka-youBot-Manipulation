import numpy as np
import modern_robotics as mr
from simulator import NextState
from generator import TrajectoryGenerator
from control import FeedbackControl
import matplotlib.pyplot as plt

# Properties of robot
Tsb = np.array([[1,0,0,0.0],[0,1,0,0.0],[0,0,1,0.0963],[0,0,0,1]])
Tb0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])
M0e = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])
Blist = np.array([[0,0,1,0.0,0.033,0.0],
                [0,-1,0,-0.5076,0.0,0.0],
                [0,-1,0,-0.3526,0.0,0.0],
                [0,-1,0,-0.2176,0.0,0.0],
                [0,0,1,0,0,0]]).T
dt = 0.01

def endEffectorPos(config, Blist, Tb0):
    """ Calculate EE position from joint angles """
    theta_list_arm = config[3:8]
    phi,x,y = config[0:3]
    T0e = mr.FKinBody(M0e, Blist, theta_list_arm)
    Tsb = np.array([[np.cos(phi), -np.sin(phi),0,x],[np.sin(phi),np.cos(phi),0,y],[0,0,1,0.0963],[0,0,0,1]])

    return Tsb@Tb0@T0e

def refToMat(traj):
    """ Calculate EE transform matrix from reference trajectory """
    return np.array([[traj[0],traj[1],traj[2],traj[9]],[traj[3],traj[4],traj[5],traj[10]],[traj[6],traj[7],traj[8],traj[11]],[0.0,0.0,0.0,1.0]])

def FullSimulation(Tse_init, Tsc_init, Tsc_goal, current_config, p_gain, i_gain) :
    """ 
    Calculate full simulation
    
    Inputs:
        Tse_init : initial configuration of the end-effector
        Tsc_init : initial cube configuration
        Tsc_final : final desired cube configuration
        current_config : current joint angles of robot
        Kp : proportional control gain
        Ki : integral control gain

    """
    # Getting End Effector Trajectory
    print('Calculating Reference Trajectory')
    ref = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_goal)

    # control gain matrix
    Kp = np.dot(p_gain, np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]]))
    Ki = np.dot(i_gain, np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]]))

    integral_error = 0
    cc = current_config
    controlled_states = [[cc[0], cc[1], cc[2], cc[3], cc[4], cc[5], cc[6], cc[7], cc[8], cc[9], cc[10], cc[11], 0.0]]
    control_error = []
    for rt in range(len(ref)-1):

        traj0 = refToMat(ref[rt])
        traj1 = refToMat(ref[rt+1])

        Xee = endEffectorPos(current_config, Blist, Tb0)

        # calculates the needed controls to move to the desired trajectory location
        Vt, controls, Xerr, integral_error = FeedbackControl(Xee, traj0, traj1, Kp, Ki, integral_error, dt, current_config)

        # calculates the next state based on the controls found in the previous line
        new_state = NextState(current_config, controls, dt, 1000.0)

        full_state = [new_state[0], new_state[1], new_state[2], new_state[3], new_state[4], new_state[5], new_state[6], new_state[7], new_state[8], new_state[9], new_state[10], new_state[11], ref[rt+1][-1]]

        controlled_states.append(full_state)
        control_error.append(Xerr)

        current_config = new_state

    print('Writing CSV files')
    np.savetxt('Playback.csv', controlled_states, delimiter=',')


