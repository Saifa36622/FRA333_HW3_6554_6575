# Import Library
import HW3_utils
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3,base
import math
#
#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float],ref:int)->list[float]:
    R,P,R_e,p_e = HW3_utils.FKHW3(q) # Get Rotation Matrix and Traslation Matrix from HW3_utils
    p_01 = P[:,0]
    p_02 = P[:,1]
    p_03 = P[:,2]
    p_0e = p_e
    # Spilt position from base to joint 1 2 3 and end-effector
    R_e0 = np.linalg.inv(R_e) # Inverse Rotation Matrix from base to end-effector to end-effector to base 
    R_e1 = R_e0 @ R[:,:,0]
    R_e2 = R_e0 @ R[:,:,1]
    R_e3 = R_e0 @ R[:,:,2]
    #Create Rotation Matrix from end-effector to joint 1 2 3
    z_1 = np.array([0.0,0.0,1.0]).reshape(3,1)
    z_2 = np.array([0.0,0.0,1.0]).reshape(3,1)
    z_3 = np.array([0.0,0.0,1.0]).reshape(3,1)
    #Defind axis of joint rotation
    z_01 = R[:,:,0] @ z_1
    z_02 = R[:,:,1] @ z_2
    z_03 = R[:,:,2] @ z_3
    #Create axis of joint rotation reference to base frame
    z_e1 = R_e1 @ z_1
    z_e2 = R_e2 @ z_2
    z_e3 = R_e3 @ z_3
    #Create axis of joint rotation reference to end-effector frame
    if ref == 0:
        J_01 = np.vstack((base.cross(z_01,base.vector_diff(p_0e,p_01,'r')).reshape(3,1),z_01))
        J_02 = np.vstack((base.cross(z_02,base.vector_diff(p_0e,p_02,'r')).reshape(3,1),z_02))
        J_03 = np.vstack((base.cross(z_03,base.vector_diff(p_0e,p_03,'r')).reshape(3,1),z_03))
        #Create Jacobian Matrix of each joint reference to base frame (Equation in Jacobian.jpg)
    else:
        J_01 = np.vstack((R_e0 @ base.cross(z_01,base.vector_diff(p_0e,p_01,'r')).reshape(3,1),z_e1))
        J_02 = np.vstack((R_e0 @ base.cross(z_02,base.vector_diff(p_0e,p_02,'r')).reshape(3,1),z_e2))
        J_03 = np.vstack((R_e0 @ base.cross(z_03,base.vector_diff(p_0e,p_03,'r')).reshape(3,1),z_e3))
        #Create Jacobian Matrix of each joint reference to end-effector frame (Equation in Jacobian.jpg)
    J_e = np.hstack((J_01,J_02,J_03))
    #Stack Jacobian Matrix of each joint reference to end-effector frame to Matrix 6x3
    return J_e #Return Jacobian Matrix
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float],ref:int)->bool:
    J_e = endEffectorJacobianHW3(q,ref) #Get Jacobian Matrix from endEffectorJacobianHW3 function
    J_re = J_e[:3,:] #Reduce Jacobian Matrix to made it can find det and inverse
    value = base.det(J_re) #Find det of Jacobian Matrix
    if abs(value) < 0.001: #If it less than 0.001 return 1 :Mean it near to singularity
        return 1
    else:
        return 0
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float],ref:int)->list[float]:
    J_e = endEffectorJacobianHW3(q,ref) #Get Jacobian Matrix from endEffectorJacobianHW3 function
    J_ret = np.transpose(J_e) #Transpose Jacobian Matrix
    w_t = np.array(w) #Transpose wrench Matrix to 6x1
    tau = J_ret @ w_t #Find tau from Transpose Jacobian Matrix dot wrench Matrix
    return tau #Return Joint forces/torques due to w
#==============================================================================================================#

#==============================================Input===========================================================#
q = [0.0,0,0.0] #joint [joint1,joint2,joint3] recommend[0.0,-math.pi/2,-0.2] for singularity test
w = [1.0,1.0,5.0,1.0,2.0,1.0] # wrench [Fx,Fy,Fz,Mx,My,Mz]
ref = 1 # Reference frame 0:base 1:end-effector
#==============================================Input===========================================================#

print(endEffectorJacobianHW3(q,ref))
print(checkSingularityHW3(q,ref))
print(computeEffortHW3(q,w,ref))
#print each function output 