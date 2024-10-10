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
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    pass
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    pass
#==============================================================================================================#