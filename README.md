# FRA333_HW3_6554_6575
## การตรวจคำตอบ

โดยการตรวจคำตอบ เราใช้ robotic toolbox ในการตรวจสอบ โดยเริ่มจากการสร้าง DH parameters ที่ตรงกับตัวหุ่นยนต์

![image](https://github.com/user-attachments/assets/04ee85cb-26a1-4597-a46e-81c515fe317d)

จากนั้นทำการ plot หุ่นยนต์ออกมาเพื่อเช็คว่าโครงสร้างใน home configuration นั้นถูกต้องหรือไม่

![image](https://github.com/user-attachments/assets/98a016c7-da7f-423d-a043-05e52722978a)

จากนั้นทำการตรวจสอบค่า Forward Kinematics (FK) จากฟังก์ชัน FKHW3 และจากการคำนวณ DH parameters ว่ามีค่าตรงกันหรือไม่

#### <ins>ที่ Home config (q1,q2,q3 = 0)</ins>

ค่า FK จาก DH-parameter

![image](https://github.com/user-attachments/assets/eb4134ed-e0b9-4779-8194-6298180340cd)

ค่าจาก FKHW3 

![image](https://github.com/user-attachments/assets/11c157b5-8543-47b1-9927-37522fb0a056)

#### <ins>ตัวอย่างค่าต่างๆ เช่น (q1 = pi/2 ,q2 = 0,q3 = 0)</ins>

ค่า FK จาก DH-parameter

![image](https://github.com/user-attachments/assets/73f52551-f0e5-4fd0-a07a-c803dd783e59)

ค่าจาก FKHW3 

![image](https://github.com/user-attachments/assets/4a9649fe-3819-42b8-927c-0df3f0c0e62d)

ดังนั้นจึงสามารถพิสูจน์ได้ว่า ค่า DH parameters ที่สร้างมานั้นสามารถนำมาใช้ตรวจสอบคำตอบได้อย่างถูกต้อง

### ข้อที่ 1 ตรวจสอบ Jacobian Matrix

เพื่อเช็คว่า Jacobian Matrix ที่เราสร้างขึ้นนั้นตรงกับค่าที่ได้จาก robotic toolbox หรือไม่ เราจึงสร้างฟังก์ชัน endEffectorJacobianHW3_robotic ขึ้นมา


![image](https://github.com/user-attachments/assets/e2ef12fb-fd6e-4ff1-9960-db97cb40fc56)


เพื่อนำ matrix jacobian ที่เป็นคำตอบของพวกเรานั้นมาทำการตรวจสอบกับค่าที่ได้จาก matrix cobian จาก robotic-toolbox และจำลองค่า q ต่างๆ เข้าไป 

![image](https://github.com/user-attachments/assets/649d78bc-271a-4d7d-9e12-fc56d240a063)

ตัวอย่างผลลัพธ์

![image](https://github.com/user-attachments/assets/bcbc0d74-6fd3-4c5a-b156-ba260bdaa1b4)

### **โดยโค้ดคำตอบและ test script สามารถเลือกแกนอ้างอิง (ref) ของ Jacobian Matrix ได้ โดยหากไม่ระบุ จะได้คำตอบอ้างอิงกับแกน 0

ตัวอย่างการเปลี่ยนแกนอ้างอิงเป็น End-Effector Frame**

#### ตัวอย่างการเปลี่ยนแกน ref (เปลี่ยนแกน ref เป็นแกน end effector)

![image](https://github.com/user-attachments/assets/26d5c810-b730-4e39-8242-591ff349b3d5)


ผลลัพธ์ที่ได้

![image](https://github.com/user-attachments/assets/42c54582-dcb6-4460-b211-b6536636ec5b)

### ข้อที่ 2 ตรวจสอบ Singularity

เราก็ได้นำ ค่า matrix jacobina ที่ได้มาจาก robotic toolbox นั้นมาคำณวนเพื่อเช็คว่า ค่า q ที่ใส่มานั้น ทำให้หุ่นอยู่ตำแหน่งใกล้สภาวะ Singularity หรือไม่ 

![image](https://github.com/user-attachments/assets/bf52e5d6-833a-4564-b741-13973cc9d231)

จากนั้นทำการสุ่มค่า q เข้าไป เเพื่อ check คำตอบของทั้ง 2 ฟังก์ชัน

![image](https://github.com/user-attachments/assets/c2ae9c89-47b0-4dcf-a10a-6a46ee789e73)

โดยค่า q ต่างๆ ดังนี้ 

![image](https://github.com/user-attachments/assets/edbfc476-e57b-459e-87d8-579e5f4596d6)

ผลลัพธ์

![image](https://github.com/user-attachments/assets/e75b81ec-2a39-4d7e-a8a1-85b208f20bf3)

จากนั้นนำค่า q ต่างๆ ที่นำไป check singualrity มา plot เพื่อ visualize pose ของหุ่นว่า จากค่า q นั้นๆ หุ่นเข้าใกล้ หรือ ไม่เข้าใกล้ค่า singualrity จริงหรือ ไม่

#### <ins>ค่า q = [0.0, pi/2, 0]</ins>


![image](https://github.com/user-attachments/assets/d85cc7d2-1900-4338-9917-cff97f301a5a)

ดังภาพ หุ่นเกิด หรือ เข้าใกล้สภาวะ singualrity จริง ที่ joint 1 และ 2 

#### <ins>ค่า q = [0.0, 0.0, pi/2] </ins>

![image](https://github.com/user-attachments/assets/7ce5e780-8e04-47f4-b5a2-794b1597ba06)

ดังภาพ หุ่นไม่เกิดสภาวะ singualrity

### ข้อที่ 3 การคำนวณแรงบิด (Torque) โดยใช้ Jacobian

เราได้สร้างฟังก์ชัน computeEffortHW3_robotic โดยใช้ robotic toolbox ในการคำนวณ

![image](https://github.com/user-attachments/assets/f76944f8-65b4-4447-a76c-4dae89135329)

และ จากนั้นนำไปตรวจคำตอบดังนี้ 

![image](https://github.com/user-attachments/assets/d0420803-6e53-4260-a40b-10cc1da4fa6b)

ผลลัพธ์

![image](https://github.com/user-attachments/assets/d42def25-510b-4c5d-8768-76ee5b204f1f)

โดยค่าที่ออกมาจาก robotic toolbox กับค่าพวกเราคิดมานั้นจะมีค่าตรงกันข้ามกัน แต่มี amplitute ตรงกัน (ค่าเท่ากัน แต่ สัญลักษณ์ตรงกันข้าม)
เนื่องจาก function robot.pay ของ robotic toolbox จะมี output เป็น มุมมองแรงที่หุ่นจะเกิดเป็นแรงต้านแรงที่เข้ามา เพื่อให้หุ่นอยู่ในสภาวะ static หรือ แรงกระทำย้อนออกไป แต่ function เราคคิดออกมามาจะเป็นมุมมองของแรงที่เกิดขึ้นที่แต่ละข้อต่อเมื่อแรงนั้นๆ เข้ามากระทำ (มุมมองตรงกันข่ามกัน)


