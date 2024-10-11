# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.
2.
3.
'''

import roboticstoolbox as rtb
from spatialmath import SE3
from math import pi
import numpy as np

# Define the tool transformation matrix with the correct Y translation
tool_transformation = (
    SE3.Tx(-0.39243-0.082) @  # Translation along X-axis
    SE3.Tz(0.109) @     # Translation along Z-axis
    SE3.Ty(-0.093) @    # Translation along Y-axis (negative)
    SE3.Ry(-pi/2)      # Rotation around Y-axis by 
)
# Define the robot using the specified MDH parameters and tool transformation
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=0.0892, offset=pi),
        rtb.RevoluteMDH(alpha=pi/2),
        rtb.RevoluteMDH(a=-0.425),
    ],
    tool=tool_transformation,
    name="RRR_Robot"
)

#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float],ref: str = "0")->list[float]:
    if ref == "0":
        J_e = robot.jacob0(q)
    elif ref == "e":
        J_e = robot.jacobe(q)
    return J_e
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    epsilon = 0.001
    J_e = robot.jacob0(q)
    J_linear = J_e[:3, :]
    # คำนวณ Determinant ของ Jacobian เชิงเส้น
    det_J_linear = np.linalg.det(J_linear)

    # print(f"Determinant for q = {q}: {det_J_linear:.6f}")
    treshhold = 0.01
    # print(abs(epsilon - abs(det_J_linear)))
    if abs(epsilon - abs(det_J_linear)) < treshhold:
        return 1 
    else :
        return 0
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J = robot.jacob0(q)
    # J_T = np.transpose(J) #Transpose Jacobian Matrix
    # w_t = np.array(w) #Transpose wrench Matrix to 6x1
    # tau = J_T @ w_t
    tau = robot.pay(w,q,J)

    return tau
#==============================================================================================================#

# q_test_1 = [0.0, 0.0, 0.0] 
# q_test_2 = [0.0, pi/2, 0]
# q_test_3 = [0.0, 0.0, pi/2]  

# w_example = [10, 0, 0, 0, 0, 0]

# J = endEffectorJacobianHW3(q_test_1)

# for i in J :
#     for j in range(len(i)) :
#         i[j] = round(i[j],2)
#     print(i)
    
# flag_1 = checkSingularityHW3(q_test_1)
# flag_2 = checkSingularityHW3(q_test_2)
# flag_3 = checkSingularityHW3(q_test_3)

# print(flag_1)
# print(flag_2)
# print(flag_3)

# check = computeEffortHW3(q_test_1,w_example)
# print(check)