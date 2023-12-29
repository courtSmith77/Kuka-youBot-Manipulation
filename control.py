import numpy as np
import modern_robotics as mr

def FeedbackControl(Xse, Xd, Xd_next, Kp, Ki, integral_error, dt, theta_list):
    """
    Calculate the taskspace feedforward plus feeback control law

    Inputs:
        Xse:      The current end effector config, Tse
        Xd:       The current reference end effector config
        Xd_next:  The next reference end effector config
        Kp, Ki:   gains for the PI controller
        dt:       time step between ref trajectory configs
    
    Output:
        Vee:      end effector twist in the end effector frame
    
    """
    dot = np.matmul(mr.TransInv(Xse), Xd)
    Xerr = mr.se3ToVec(mr.MatrixLog6(dot))
    adj = mr.Adjoint(dot)
    
    dot = np.matmul(mr.TransInv(Xd), Xd_next)
    vLog = mr.se3ToVec(mr.MatrixLog6(dot))
    Vd = (1.0/dt)*vLog

    integral_error += Xerr*dt

    Vt = np.matmul(adj,Vd) + Kp@Xerr + Ki@integral_error

    # finding Jacobians
    theta_list_arm = theta_list[3:8]
    Blist = np.array([[0,0,1,0.0,0.033,0.0],
                    [0,-1,0,-0.5076,0.0,0.0],
                    [0,-1,0,-0.3526,0.0,0.0],
                    [0,-1,0,-0.2176,0.0,0.0],
                    [0,0,1,0,0,0]]).T
    M0e = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])
    Tb0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])

    # Finding Jacobian of the base
    r = 0.0475
    w = 0.15
    l = 0.235
    F_6 = np.dot(r/4, [[0,0,0,0],[0,0,0,0],[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],[1, 1, 1, 1],[-1, 1, -1, 1],[0,0,0,0]])
    T0e_inv = mr.TransInv(mr.FKinBody(M0e, Blist, theta_list_arm))
    Tb0_inv = mr.TransInv(Tb0)
    adj = mr.Adjoint(np.matmul(T0e_inv, Tb0_inv))

    J_base = np.matmul(adj,F_6)
    J_arm = mr.JacobianBody(Blist, theta_list_arm)
    J_ee = np.append(J_base, J_arm, axis=1)

    # seudo inverse of J
    J_pinv2 = np.linalg.pinv(J_ee, 1.0e-4)

    controls = np.matmul(J_pinv2, Vt.T)

    return Vt, controls, Xerr, integral_error


