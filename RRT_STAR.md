# RRT* จาก Occupancy Grid Map

ส่วน offline ใช้ Python + NumPy + SciPy + Matplotlib และอ่าน probability จาก OCGM จริง
ไม่ใช้ geometry ของ SDF/ฉากเพื่อสร้างเส้นทางแทนข้อมูลที่วัด
รันได้โดยไม่เปิด Gazebo, ROS หรือ CoppeliaSim

## เริ่มวางแผน

```bash
cd "/home/d-dream/autonomous robot"
python3 rrt_star_plan.py
```

โปรแกรมอ่าน `ocgm_660610823.csv` กับ `ocgm_660610823.meta.json` ในโฟลเดอร์โปรเจกต์
สุ่ม seed ใหม่ทุกครั้ง และแสดง seed พร้อม start/goal ใน Terminal
ทดลองซ้ำแบบเดิม: `python3 rrt_star_plan.py --seed 271411`
ดูภาพหลังคำนวณ: `python3 rrt_star_plan.py --seed 271411 --show`

หากใช้แมพที่เพิ่งเก็บรอบใหม่ ให้ระบุไฟล์นั้น:

```bash
python3 rrt_star_plan.py --map "maps/array_<วันเวลา>/ocgm_660610823.csv" --seed 271411
```

ค่าเริ่มต้น: รัศมีครอบตัวรถ 0.50 m, เผื่อการติดตามเส้นทาง 0.10 m,
5,000 iterations, ขยายต้นไม้ครั้งละไม่เกิน 0.65 m, goal bias 0.12,
สุ่มให้ start/goal ห่างกันอย่างน้อย 5 m และอยู่ในพื้นที่ว่างที่เชื่อมถึงกัน
ค่าเหล่านี้แก้ผ่าน `--robot-radius`, `--margin`, `--iterations`, `--step-size`,
`--goal-bias`, `--min-distance` ตามลำดับ ดูตัวเลือกทั้งหมดด้วย `--help`

กรณีอยากกำหนด pose เองเพื่อทดลอง:

```bash
python3 rrt_star_plan.py --seed 42 --start -1 0 0 --goal -5 -3 1.57
```

x/y เป็นเมตรใน map world และ theta เป็นเรเดียน ต้องอยู่ในพื้นที่ที่ผ่านการขยายสิ่งกีดขวางแล้ว
หาก pose อยู่ในผนัง พื้นที่ unknown ขอบแมพ หรือใกล้สิ่งกีดขวางเกินไป โปรแกรมจะปฏิเสธ

## ขั้นตอนอัลกอริทึม

1. ตรวจ CSV/metadata ให้ตรงกัน: shape, resolution, extent, probability ช่วง [0,1]
   และ orientation แบบแถวเพิ่มตาม world Y คอลัมน์เพิ่มตาม world X
2. ใช้เฉพาะเซลล์ probability < 0.35 เป็น free; unknown ที่ 0.5 ถือว่าผ่านไม่ได้
   ขยายสิ่งกีดขวางด้วยขนาดวงกลมครอบรถ + margin + ค่าชดเชยขนาดเซลล์
3. สุ่ม `[x,y,theta]` เริ่มต้น/เป้าหมายใน connected component เดียวกัน
   รัศมีวงกลมทำให้การหมุนอยู่กับที่ไม่เปลี่ยนพื้นที่ครอบตัวรถ
4. RRT*: Sample → Nearest → Steer → ตรวจชนตลอด segment → ChooseParent
   จากต้นทุนระยะทางสะสมต่ำสุด → Rewire พร้อมปรับต้นทุนลูกหลาน
   รัศมีเพื่อนบ้านลดตาม `sqrt(log(n)/n)` ใน 2 มิติและมีเพดานตามตัวเลือก
5. ทำต่อจนครบ iterations แม้พบเส้นทางแรกแล้ว จากนั้นเลือกการต่อ goal ที่มีต้นทุนต่ำสุด
   ย้อน parent กลับ start และกลับลำดับให้เป็น start → goal
6. ส่งออกโหนดจริงจากต้นไม้ พร้อม interpolate ตำแหน่งตามเส้นตรงระหว่างโหนด
   ทุกช่วงห่างไม่เกิน 0.10 m และมุม interpolate ผ่านส่วนต่างที่สั้นกว่า

เป็น RRT* ในพื้นที่ XY สำหรับรถที่หมุนอยู่กับที่ได้ ต้นทุนคือความยาวเส้นทาง
theta ของ start/goal ถูกเก็บไว้ใช้ในการควบคุม ส่วนมุมโหนดกลางใช้แนวเส้นทาง
จึงไม่ใช่การค้นหาแบบ kinodynamic สำหรับรถที่มีรัศมีเลี้ยวขั้นต่ำ
ผล finite iterations เป็นเส้นทางที่ดีที่สุดที่พบในการรันนั้น ไม่รับรอง global optimum

อ้างอิง: [Karaman & Frazzoli, Sampling-based Algorithms for Optimal Motion Planning](https://arxiv.org/abs/1105.1186)

## ไฟล์ผลลัพธ์

สร้างโฟลเดอร์ใหม่ `planning/rrtstar_<วันเวลา>_seed<seed>/` ทุกครั้ง:

| ไฟล์ | เนื้อหา |
| --- | --- |
| `rrt_star.png` | ภาพ OCGM, พื้นที่เผื่อชน, ต้นไม้ และเส้นทางตามลำดับ |
| `plan.json` | start/goal, node poses, พารามิเตอร์, metrics, path/hash ของแมพ |
| `path_nodes.npy`, `path_nodes.csv` | อาร์เรย์ N×3 `[x,y,theta]` เรียงจาก start ไป goal |
| `path_interpolated.npy`, `.csv` | อาร์เรย์หลัง interpolate ให้ช่วงสั้นลง |
| `tree.csv` | node, x, y, parent, cost_m ของต้นไม้สุดท้าย |
| `cost_history.csv` | iterations ที่ต้นทุนเส้นทางดีที่สุดลดลง |

```python
import numpy as np
poses = np.load("planning/rrtstar_<วันเวลา>_seed<seed>/path_nodes.npy")
t = len(poses)
for i in range(t):
    x, y, theta = poses[i]
    print(i, x, y, theta)
```

## ต่อ CoppeliaSim / ZeroMQ Remote API

ระบบ mapping เดิมในโปรเจกต์ใช้ Gazebo ส่วนโจทย์ระบุ CoppeliaSim `.ttt`
ไฟล์ `.ttt` เดิมที่พบในเครื่องยังไม่ได้เปิดตรวจว่าเป็นฉาก 5 ห้องเดียวกับ OCGM นี้
จึงยังนับว่าการรันจริงผ่าน CoppeliaSim ไม่ได้ทดสอบ
ต้องใช้ scene ที่ตรงกับแมพและกำหนดกรอบพิกัดให้ตรงกันก่อนขับ
แมพรอบก่อนเพิ่มเฟอร์นิเจอร์ควรเก็บใหม่ก่อนนำไปใช้กับฉากที่เพิ่มของแล้ว

เตรียมตัวคุมล้อไว้ที่ `follow_rrt_coppelia.py` และตัวคุมคณิตศาสตร์ที่ `rrt_path_control.py`
ติดตั้ง Python client ตาม [คู่มือ ZeroMQ](https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm):

```bash
python3 -m pip install coppeliasim-zmqremoteapi-client
```

ตัวอย่างเป็น client ปัจจุบันและ API ของ CoppeliaSim ปัจจุบัน
รองรับชื่อโมดูล `zmqRemoteApi` ในกรณีมีโมดูลนั้นอยู่แล้ว แต่ไม่ได้ยืนยัน API ของ server รุ่นเก่า

คัดลอก `coppelia_robot.example.json` เป็นไฟล์ config ของตัวเองแล้วกรอก:

- `robot`: absolute path ของตัว robot model ที่ใช้อ้างอิง pose
- `left_joints`/`right_joints`: รายการ absolute path ของ joint ล้อแต่ละด้าน (2 หรือ 4 ล้อได้)
- `wheel_radius`/`wheel_separation`: รัศมีล้อ/ระยะระหว่างล้อซ้าย–ขวาที่วัดจริง หน่วยเมตร
- `footprint_radius`: รัศมีวงกลมครอบตัวรถรวมล้อ ต้องไม่ใหญ่กว่าที่ใช้วางแผน
- `left_sign`/`right_sign`: เครื่องหมาย +1/-1 ตามแกน joint ของฉาก
- `map_to_scene`: `[tx,ty,yaw]` แปลงจาก map world ไป scene world; ใช้ `[0,0,0]` เมื่อยืนยันว่าตรงกันแล้ว
- `robot_forward_yaw`: มุมแกนหน้าของหุ่นเทียบกับแกน X ของ model (เรเดียน)

ค่าขนาดใน template เป็น `null` เพื่อให้กรอกขนาดรถจริงก่อนรัน ไม่มีการเดาชื่อ joint/ขนาดรถจากไฟล์ binary
ตั้ง joint มอเตอร์เป็น dynamic velocity control ใน scene พร้อม torque ที่เหมาะสม
ปิด script อื่นที่สั่งล้อเดียวกัน และเปิด scene ที่ตรงกับแมพใน CoppeliaSim โดยยังหยุด simulation ไว้

ตรวจไฟล์โดยไม่เชื่อมต่อ simulator:

```bash
python3 follow_rrt_coppelia.py --plan "planning/<รอบที่เลือก>/plan.json" --config coppelia_robot.json --check
```

รันจริงเมื่อกำหนดฉากและ config แล้ว:

```bash
python3 follow_rrt_coppelia.py --plan "planning/<รอบที่เลือก>/plan.json" --config coppelia_robot.json
```

โปรแกรมตรวจ hash ของแมพ/เส้นทาง วางรถที่ random start หนึ่งครั้งขณะ simulator หยุด
เริ่ม stepping simulation อ่าน pose จริง แล้วติดตามจุดที่ interpolate ไว้
แปลงความเร็วเชิงเส้น v และความเร็วหมุน omega เป็นความเร็วล้อ:

```text
left_rad_s  = (v - omega * wheel_separation / 2) / wheel_radius
right_rad_s = (v + omega * wheel_separation / 2) / wheel_radius
```

สั่งทุกมอเตอร์ผ่าน `sim.setJointTargetVelocity(joint_handle, target_rad_s)`
เมื่อถึง goal จะหมุนให้ตรง theta เป้าหมายและสั่งล้อหยุด; Ctrl+C ก็พยายามหยุดทุกมอเตอร์
ไม่ขยับรถด้วยการตั้ง pose ระหว่างการเดินทาง
บันทึก pose ที่วัดจริงใน `coppelia_run_<เวลา>/actual_poses.csv`
การทดสอบตัวคุมด้วยโมเดล differential drive เป็นการตรวจคณิตศาสตร์ ไม่ใช่หลักฐานการทำงานของ scene จริง

อ้างอิง: [setJointTargetVelocity](https://manual.coppeliarobotics.com/en/sim/simSetJointTargetVelocity.htm)

## ตรวจโปรแกรม

```bash
python3 -m unittest -v test_rrt_star.py
```

ครอบคลุม unknown/ขนาดรถ, ชนมุมเซลล์ด้วย geometric oracle อิสระ, random poses ที่เชื่อมถึงกัน,
RRT* อ้อมกำแพงผ่านช่องเปิด, cost/parent หลัง rewire, interpolation, หน่วย/ทิศความเร็วล้อ,
การแปลงกรอบพิกัด และการติดตามเส้นทางด้วยแบบจำลอง differential drive
