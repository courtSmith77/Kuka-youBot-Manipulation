import numpy as np
from full_code import FullSimulation

print('Running Best')
# Control Gains
Kp = 0.6
Ki = 0.0
print('Feedback Gains:')
print(f'Kp = {Kp}')
print(f'Ki = {Ki}')

# initial robot joint configuration
current_config = np.array([-1.1,-0.1,0.15,0.0,0.79,0.79,-1.6,0.0,0.0,0.0,0.0,0.0])
print(f'Initial Configuration = {current_config}')

# Initial Conditions of Robot and Cube
Tse_init = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
Tsc_init = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
Tsc_goal = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])
print('Box Positions:')
print(f'Initial = {Tsc_init}')
print(f'Goal = {Tsc_goal}')

FullSimulation(Tse_init, Tsc_init, Tsc_goal, current_config, Kp, Ki)