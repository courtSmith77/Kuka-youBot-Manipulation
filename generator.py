import numpy as np
import modern_robotics as mr


def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final):
    """
    Generate reference trajectory for the end-effector frame

    Inputs:
        Tse_init :      initial configuration of the end-effector
        Tsc_init :      initial cube configuration
        Tsc_final :     final desired cube configuration
        Tce_grasp :     ee config when grasping
        Tce_standoff : ee config above cube (during standoff)
            ^^ relative to cube frame before lowering ex: Tce_grasp
    
    Outputs:
        trajectory_path : representaiton of N configurations of the ee
            ^^ N ~ 30*k/0.01 sequential reference configs

    Functions from mr:
        CartesianTrajectory :
          origin follows a straight line
            Inputs:
                Xstart - initial configuration of ee
                Xend - final configuration of ee
                Tf - Total time to complete trajectory
                N - number of waypoints between start and end
                method - 3 for cubic or 5 for quintic
            Output:
                traj - trajectory list of ee configurations SE(3)

    """
    # 90 degree rotation for end effector
    rot_y_90deg = np.array([[np.cos(2*np.pi/4), 0, np.sin(2*np.pi/4)],
                            [0, 1, 0],
                            [-np.sin(2*np.pi/4), 0, np.cos(2*np.pi/4)]])
    
    # end effector orientation with respect to block orientation
    Tsc_i_R = Tsc_init[:3,:3]
    new_i_R = rot_y_90deg@Tsc_i_R

    Tsc_f_R = Tsc_final[:3,:3]
    new_f_R = Tsc_f_R@rot_y_90deg

    # standoff and grasping positions for the end effector
    Tse_standoff_1 = np.copy(Tsc_init)
    Tse_standoff_1[:3,:3] = new_i_R
    Tse_standoff_1[2,3] += 0.3

    Tse_grasping_1 = np.copy(Tsc_init)
    Tse_grasping_1[:3,:3] = new_i_R

    Tse_standoff_2 = np.copy(Tsc_final)
    Tse_standoff_2[:3,:3] = new_f_R
    Tse_standoff_2[2,3] += 0.3

    Tse_grasping_2 = np.copy(Tsc_final)
    Tse_grasping_2[:3,:3] = new_f_R

    ###### Generating Trajectories #######

    all_traj = []

    # Segment 1
    traj1 = mr.CartesianTrajectory(Tse_init, Tse_standoff_1, 15.0, 1500, 3)
    for ii in range(len(traj1)):

        t = traj1[ii]
        tr = np.array([t[0,0], t[0,1], t[0,2], t[1,0], t[1,1], t[1,2], t[2,0], t[2,1], t[2,2], t[0,3], t[1,3], t[2,3], 0.0])

        all_traj.append(tr)

    # Segment 2
    end_prev_traj = traj1[-1]
    traj2 = mr.CartesianTrajectory(end_prev_traj, Tse_grasping_1, 5.0, 500, 3)
    for ii in range(len(traj2)):
        t = traj2[ii]
        tr = np.array([t[0,0], t[0,1], t[0,2], t[1,0], t[1,1], t[1,2], t[2,0], t[2,1], t[2,2], t[0,3], t[1,3], t[2,3], 0.0])


        all_traj.append(tr)

    # Segment 3
    end_prev_traj = traj2[-1]
    traj3 = mr.CartesianTrajectory(end_prev_traj, end_prev_traj, 0.625, 63, 3)
    for ii in range(len(traj3)):
        t = traj3[ii]
        tr = np.array([t[0,0], t[0,1], t[0,2], t[1,0], t[1,1], t[1,2], t[2,0], t[2,1], t[2,2], t[0,3], t[1,3], t[2,3], 1.0])

        all_traj.append(tr)

    # Segment 4
    end_prev_traj = traj3[-1]
    traj4 = mr.CartesianTrajectory(end_prev_traj, Tse_standoff_1, 5.0, 500, 3)
    for ii in range(len(traj4)):
        t = traj4[ii]
        tr = np.array([t[0,0], t[0,1], t[0,2], t[1,0], t[1,1], t[1,2], t[2,0], t[2,1], t[2,2], t[0,3], t[1,3], t[2,3], 1.0])

        all_traj.append(tr)

    # Segment 5
    end_prev_traj = traj4[-1]
    traj5 = mr.CartesianTrajectory(end_prev_traj, Tse_standoff_2, 15.0, 1500, 3)
    for ii in range(len(traj5)):
        t = traj5[ii]
        tr = np.array([t[0,0], t[0,1], t[0,2], t[1,0], t[1,1], t[1,2], t[2,0], t[2,1], t[2,2], t[0,3], t[1,3], t[2,3], 1.0])

        all_traj.append(tr)

    # Segment 6
    end_prev_traj = traj5[-1]
    traj6 = mr.CartesianTrajectory(end_prev_traj, Tse_grasping_2, 5.0, 500, 3)
    for ii in range(len(traj6)):
        t = traj6[ii]
        tr = np.array([t[0,0], t[0,1], t[0,2], t[1,0], t[1,1], t[1,2], t[2,0], t[2,1], t[2,2], t[0,3], t[1,3], t[2,3], 1.0])

        all_traj.append(tr)

    # Segment 7
    end_prev_traj = traj6[-1]
    traj7 = mr.CartesianTrajectory(end_prev_traj, end_prev_traj, 0.625, 63, 3)
    for ii in range(len(traj7)):
        t = traj7[ii]
        tr = np.array([t[0,0], t[0,1], t[0,2], t[1,0], t[1,1], t[1,2], t[2,0], t[2,1], t[2,2], t[0,3], t[1,3], t[2,3], 0.0])

        all_traj.append(tr)

    # Segment 8
    end_prev_traj = traj7[-1]
    traj8 = mr.CartesianTrajectory(end_prev_traj, Tse_standoff_2, 5.0, 500, 3)
    for ii in range(len(traj8)):
        t = traj8[ii]
        tr = np.array([t[0,0], t[0,1], t[0,2], t[1,0], t[1,1], t[1,2], t[2,0], t[2,1], t[2,2], t[0,3], t[1,3], t[2,3], 0.0])

        all_traj.append(tr)

    return all_traj
