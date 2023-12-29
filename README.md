# Kuka-youBot-Manipulation
Author: Courtney Smith



Calculates the desired dynamics and controls for the mobile manipulation of the KUKA youBot to complete a pick and place task in CoppeliaSim.

All equations can be found in the textbook "Modern RObotics: Mechanics, Planning, and control" by Frank Chongwoo Park and Kevin M. Lynch. Additionally the Modern Robotics Library was used to perform matrix calculations.

Files

`generator.py` - generates the reference trajectory for the end effector to complete the desired pick and place task

`simulator.py` - calculates the next state of the robot using 1st-order Euler Step based on a given control command

`control.py` - calculates the feedforward + feedback control law for the robot to achieve the reference trajectory position

`full_simulator.py` - combines the previous 3 functions to calculate the robot state at each time stamp and save it to a csv to be run in CoppeliaSim

`run_script.py` - runs the simulation with the desired control gains and box initial and final positions.
