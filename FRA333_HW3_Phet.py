# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.
2.
3.
'''

import numpy as np
from HW3_utils import FKHW3
from math import pi

#=============================================<คำตอบข้อ 1>======================================================#
# ฟังก์ชันสร้าง Skew-symmetric matrix จากเวกเตอร์


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
    J_e = np.vstack((J_v, J_omega))

    # แปลงเป็น list และส่งค่ากลับ
    return J_e


#==============================================================================================================#

#=============================================<คำตอบข้อ 2>======================================================#
#code here
# def checkSingularityHW3(q: list[float], epsilon: float = 0.001) -> int:
#     """
#     ตรวจสอบสภาวะ Singularity ของหุ่นยนต์
#     Arguments:
#     q -- ค่า joint configuration [q1, q2, q3]
#     epsilon -- ค่าความคลาดเคลื่อนที่ใช้ในการตรวจสอบ Singularity (default = 0.001)

#     Returns:
#     flag -- 1: อยู่ในสภาวะ Singularity, 0: อยู่ในสภาวะปกติ
#     """
#     # คำนวณ Jacobian Matrix (J) โดยใช้ฟังก์ชัน endEffectorJacobianHW3
#     J = np.array(endEffectorJacobianHW3(q))

#     # ลดรูป Jacobian Matrix ให้เหลือแค่ส่วนของความเร็วเชิงเส้น (Linear Velocity)
#     J_linear = J[:3, :]  # J_v เป็นเมทริกซ์ขนาด 3x3 (แถว 1-3 ของ Jacobian)

#     # คำนวณ Determinant ของ Jacobian เชิงเส้น
#     det_J_linear = np.linalg.det(J_linear)

#     # แสดงค่า Determinant ของ Jacobian Linear Matrix
#     print(f"Determinant for q = {q}: {det_J_linear:.6f}")

#     # ตรวจสอบว่า |det(J*)| < epsilon หรือไม่
#     return 1 if abs(det_J_linear) < epsilon else 0

# # ตัวอย่างการทดสอบฟังก์ชัน checkSingularityHW3
# q_test_1 = [0.0, np.pi/2, 0.0]  # กำหนดตำแหน่งเอกฐานที่คาดว่าจะเกิดขึ้น
# q_test_2 = [0.0, 0.0, 0.0]  # กำหนดตำแหน่งที่คาดว่าจะไม่เกิดสถานะเอกฐาน

# J = endEffectorJacobianHW3(q_test_2)
# for i in J :
#     for j in i :
#         print(round(j,2))

# # เรียกใช้ฟังก์ชันเพื่อตรวจสอบสถานะเอกฐานในทั้งสองกรณี
# flag_1 = checkSingularityHW3(q_test_1)
# flag_2 = checkSingularityHW3(q_test_2)

# print(f"Singularity flag for q = {q_test_1}: {flag_1} (ควรได้ 1 สำหรับสถานะเอกฐาน)")
# print(f"Singularity flag for q = {q_test_2}: {flag_2} (ควรได้ 0 สำหรับสถานะปกติ)")
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    pass
#==============================================================================================================#
q_test_1 = [0.0, 0.0, 0.0] 
q_test_2 = [0.0, pi/2, 0]
q_test_3 = [0.0, 0.0, pi/2]  

w_example = [10, 0, 0, 0, 0, 0]

J = endEffectorJacobianHW3(q_test_1)

for i in J :
    for j in range(len(i)) :
        i[j] = round(i[j],2)
    print(i)