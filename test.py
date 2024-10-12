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

# Test the forward kinematics at q = [0, 0, 0]
q = [0.0, 0.0, pi/2] 
robot.plot(q)
input("")
# T = robot.fkine(q)
# print("End-effector transformation matrix at q = [0, 0, 0]:")
# print(T)


# Jacobian = robot.jacob0(q)
# print(Jacobian)
