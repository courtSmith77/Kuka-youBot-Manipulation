import numpy as np
import modern_robotics as mr

def NextState(current_config, controls, dt, max_speed):
    """"
    Calculate the next state using 1st-order Euler Step
    
    Inputs:
        current_config :    12-vector of robot configuration (3 chassis, 5 arm, 4 wheel)
        curr_control :      9-vector of wheel and joint speeds (4 wheel, 5 joint)
        dt :                time-step
        max_speed :         scalar indicating speed limit of wheels and joints

    Output:
        new_config :    12-vector of new robot configuration
    """
    # check if speed is outside the max_speed range
    for cc in range(len(controls)):
        if abs(controls[cc]) > max_speed and controls[cc] > 0 :
            controls[cc] = max_speed
        elif abs(controls[cc]) > max_speed and controls[cc] < 0 :
            controls[cc] = -1*max_speed

    new_arms = current_config[3:8] + np.dot(dt, controls[4:])
    new_wheels = current_config[8:] + np.dot(dt, controls[:4])

    # # properties of the robot
    r = 0.0475
    w = 0.15
    l = 0.235

    # calcualte H matrix
    h1 = np.dot((1/(r*np.cos(-np.pi/4))),[l*np.sin(-np.pi/4) - w*np.cos(-np.pi/4), np.cos(-np.pi/4 + current_config[0]), np.sin(-np.pi/4 + current_config[0])])
    h2 = np.dot((1/(r*np.cos(np.pi/4))),[l*np.sin(np.pi/4) + w*np.cos(np.pi/4), np.cos(np.pi/4 + current_config[0]), np.sin(np.pi/4 + current_config[0])])
    h3 = np.dot((1/(r*np.cos(-np.pi/4))),[-l*np.sin(-np.pi/4) + w*np.cos(-np.pi/4), np.cos(-np.pi/4 + current_config[0]), np.sin(-np.pi/4 + current_config[0])])
    h4 = np.dot((1/(r*np.cos(np.pi/4))),[-l*np.sin(np.pi/4) - w*np.cos(np.pi/4), np.cos(np.pi/4 + current_config[0]), np.sin(np.pi/4 + current_config[0])])

    H_phi = np.array([h1,h2,h3,h4])
    H_inv = np.linalg.pinv(H_phi, 1e-4)

    qdot = np.matmul(H_inv, controls[:4])

    # calculate new chassis state
    new_q = current_config[:3] + qdot*dt

    new_config = [new_q[0], new_q[1], new_q[2], new_arms[0], new_arms[1], new_arms[2], new_arms[3], new_arms[4], new_wheels[0], new_wheels[1], new_wheels[2], new_wheels[3]]
    
    return new_config

