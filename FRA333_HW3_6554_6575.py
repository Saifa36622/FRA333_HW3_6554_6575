# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.
2.
3.
'''
import roboticstoolbox as rtb
from spatialmath import SE3
from math import pi
import numpy as np
from HW3_utils import FKHW3



#===========================================<ข้อ 1>====================================================#
#code here
# ฟังก์ชันสร้าง Skew-symmetric matrix จากเวกเตอร์
def skew_matrix(v):
    """ สร้าง Skew-symmetric matrix จากเวกเตอร์ v """
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

# ฟังก์ชันคำนวณ Jacobian ของ End-Effector
def endEffectorJacobianHW3(q: list[float], ref: int = 0) -> list[float]:
    """
    คำนวณ Jacobian ของหุ่นยนต์
    q: มุมของข้อต่อในรูปแบบ [q1, q2, q3]
    ref: 0 สำหรับ Base Frame และ 1 สำหรับ End-Effector Frame
    """
    # เรียกใช้ Forward Kinematics จากฟังก์ชัน FKHW3
    R, P, R_e, p_e = FKHW3(q)

    # กำหนดจำนวนข้อต่อ n = 3 สำหรับ RRR Manipulator
    n = 3

    # สร้าง Jacobian Matrix (Linear Velocity และ Angular Velocity)
    J_v = np.zeros((3, n))  # เมตริกซ์จาโคเบียนเชิงเส้นขนาด 3x3
    J_omega = np.zeros((3, n))  # เมตริกซ์จาโคเบียนเชิงมุมขนาด 3x3

    # คำนวณหา Jacobian ทีละข้อต่อ
    for i in range(n):
        # ตำแหน่งของข้อต่อ i
        p_0_i = P[:, i]  # ตำแหน่งของจุดกำเนิดเฟรม F_i

        # แกนหมุนของข้อต่อที่ i
        z_0_i = R[:, 2, i]  # แกน Z ของเฟรม F_i

        # คำนวณความเร็วเชิงเส้น (Linear Velocity)
        J_v[:, i] = np.cross(z_0_i, (p_e - p_0_i))

        # คำนวณความเร็วเชิงมุม (Angular Velocity)
        J_omega[:, i] = z_0_i

    if ref == 1:  # ในกรณีที่ต้องการ Jacobian อ้างอิงกับ End-Effector Frame
        # คำนวณการแปลงค่า Jacobian จาก Base Frame ไป End-Effector Frame
        R_e0 = np.linalg.inv(R_e)  # Inverse Rotation Matrix จาก Base ไป End-Effector
        J_v = R_e0 @ J_v
        J_omega = R_e0 @ J_omega

    # รวมเมตริกซ์จาโคเบียนเชิงเส้นและเชิงมุม
    J_e_manual = np.vstack((J_v, J_omega))

    # แปลงเป็น list และส่งค่ากลับ
    return J_e_manual



#==============================================================================================================#

#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
def checkSingularityHW3(q: list[float]) -> bool:
    
    epsilon = 0.001
    treshhold = 0.01

    # คำนวณ Jacobian แบบ manual
    J_e_manual = endEffectorJacobianHW3(q, ref=0)

    # ดึงเฉพาะส่วนเชิงเส้น (3x3 matrix)
    J_linear_manual = J_e_manual[:3, :]  # Jacobian เชิงเส้น

    # คำนวณ Determinant ของ Jacobian เชิงเส้น
    det_J_linear_manual = np.linalg.det(J_linear_manual)

    # print(f"Determinant for q = {q}: {det_J_linear_manual:.6f}")

    # ตรวจสอบว่า Jacobian ใกล้เคียงกับ Singular Point หรือไม่
    if abs(epsilon - abs(det_J_linear_manual)) < treshhold:
        return 1  # Singular
    else:
        return 0  # Non-singular

#==============================================================================================================#

#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
def computeEffortHW3(q: list[float], w: list[float]) -> list[float]:
    """
    ฟังก์ชันคำนวณค่าแรงบิด (Torque) โดยใช้ Jacobian ที่คำนวณด้วยมือ
    q: มุมของข้อต่อในรูปแบบ [q1, q2, q3]
    w: แรงกระทำ (Wrench) ในรูปแบบ [Fx, Fy, Fz, Mx, My, Mz]
    """
    # คำนวณ Jacobian แบบ manual
    J_manual = endEffectorJacobianHW3(q, ref=0)

    # ทรานสโพส Jacobian Matrix
    J_T_manual = np.transpose(J_manual)

    # แปลง wrench เป็น numpy array
    w_t = np.array(w)  # แปลง w ให้เป็นเวกเตอร์ 6x1

    # คำนวณค่าแรงบิด (Torque)
    tau_manual = J_T_manual @ w_t

    return tau_manual
#==============================================================================================================#