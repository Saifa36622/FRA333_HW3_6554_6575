# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.
2.
3.
'''
import numpy as np
from HW3_utils import FKHW3


#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
from FRA333_HW3_Phet import endEffectorJacobianHW3
def test_endEffectorJacobianHW3():
    """
    ทดสอบความถูกต้องของฟังก์ชัน endEffectorJacobianHW3 กับค่าที่คาดหวัง
    """
    # กรณีทดสอบที่ 1: ค่าเริ่มต้น [0, 0, 0]
    q_test_1 = [0, 0, 0]
    # คำนวณค่า Jacobian ด้วยฟังก์ชัน endEffectorJacobianHW3
    J_e_1 = np.array(endEffectorJacobianHW3(q_test_1))

    # กรณีทดสอบที่ 2: ค่า q = [pi/4, pi/4, 0]
    q_test_2 = [np.pi/4, np.pi/4, 0]
    J_e_2 = np.array(endEffectorJacobianHW3(q_test_2))

    # กรณีทดสอบที่ 3: ค่า q = [pi/2, pi/2, pi/2]
    q_test_3 = [np.pi/2, np.pi/2, np.pi/2]
    J_e_3 = np.array(endEffectorJacobianHW3(q_test_3))

    # พิมพ์ค่า Jacobian Matrix เพื่อตรวจสอบ
    print("Jacobian Matrix for q = [0, 0, 0]:")
    print(J_e_1)

    print("Jacobian Matrix for q = [pi/4, pi/4, 0]:")
    print(J_e_2)

    print("Jacobian Matrix for q = [pi/2, pi/2, pi/2]:")
    print(J_e_3)

    # เพิ่มการตรวจสอบด้วยเงื่อนไข assert
    assert J_e_1.shape == (6, 3), f"รูปแบบ Jacobian Matrix ไม่ถูกต้อง: {J_e_1.shape}"
    assert J_e_2.shape == (6, 3), f"รูปแบบ Jacobian Matrix ไม่ถูกต้อง: {J_e_2.shape}"
    assert J_e_3.shape == (6, 3), f"รูปแบบ Jacobian Matrix ไม่ถูกต้อง: {J_e_3.shape}"

    print("การทดสอบทั้งหมดผ่านสำเร็จ!")

# เรียกใช้การทดสอบ
test_endEffectorJacobianHW3()

#==============================================================================================================#

#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
from FRA333_HW3_Phet import checkSingularityHW3  # นำเข้าฟังก์ชันที่สร้างไว้

# ตั้งค่ามุมข้อต่อสำหรับการตรวจสอบสถานะเอกฐาน
q_test_singularity = [0.0, np.pi/2, 0.0]  # ค่าที่คาดว่าจะเกิดสถานะเอกฐาน
q_test_normal = [0.0, 0.0, 0.0]  # ค่าที่คาดว่าจะไม่เกิดสถานะเอกฐาน

# ตรวจสอบสถานะเอกฐานของหุ่นยนต์
flag_singularity = checkSingularityHW3(q_test_singularity)
flag_normal = checkSingularityHW3(q_test_normal)

# แสดงผลลัพธ์ที่ได้
print(f"Singularity flag for q = {q_test_singularity}: {flag_singularity} (ควรได้ 1 สำหรับสถานะเอกฐาน)")
print(f"Singularity flag for q = {q_test_normal}: {flag_normal} (ควรได้ 0 สำหรับสถานะปกติ)")

# ตรวจสอบผลลัพธ์ด้วย assert
try:
    assert flag_singularity == 1, f"ข้อผิดพลาด: ควรได้ค่า Singularity flag = 1 สำหรับ q = {q_test_singularity}"
    assert flag_normal == 0, f"ข้อผิดพลาด: ควรได้ค่า Singularity flag = 0 สำหรับ q = {q_test_normal}"
    print("ผลลัพธ์ของฟังก์ชัน `checkSingularityHW3` ถูกต้อง")
except AssertionError as e:
    print(e)

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here

#==============================================================================================================#