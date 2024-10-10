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
    SE3.Tx(-0.39243) @  # Translation along X-axis
    SE3.Tz(0.109) @     # Translation along Z-axis
    SE3.Ty(-0.093) @    # Translation along Y-axis (negative)
    SE3.Ry(-pi/2) @       # Rotation around Y-axis by -π
    SE3.Tz(0.082)       # Final translation along Z-axis
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
# q = [0,0,0] 
# robot.plot(q)
# input("hold")
#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    J_e = robot.jacob0(q)
    # J_e = robot.jacobe(q)
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

    print(f"Determinant for q = {q}: {det_J_linear:.6f}")
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
    tau = np.dot(J.T, w)
    # M = w[0:2]
    # F= w[3:5]

    return tau
#==============================================================================================================#

q_test_1 = [0.0, 0.0, 0.0]  # กำหนดตำแหน่งเอกฐานที่คาดว่าจะเกิดขึ้น
q_test_2 = [0.0, pi/2, 0]
q_test_3 = [0.0, 0.0, pi/2]  # กำหนดตำแหน่งที่คาดว่าจะไม่เกิดสถานะเอกฐาน

w_example = [10, 0, 0, 0, 0, 0]

J = endEffectorJacobianHW3(q_test_2)

for i in J :
    for j in i :
        print(round(j,2))

flag_1 = checkSingularityHW3(q_test_1)
flag_2 = checkSingularityHW3(q_test_2)

print(flag_1)
print(flag_2)

check = computeEffortHW3(q_test_1,w_example)
print(check)