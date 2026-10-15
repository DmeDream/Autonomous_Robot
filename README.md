# Custom Python OCGM 660610823

## RRT* วางแผนจากแมพที่เก็บแล้ว

รัน `python3 rrt_star_plan.py` เพื่อโหลด OCGM สุ่ม start/goal ที่ไม่ชน และวางแผน RRT* แบบ offline
ได้ภาพต้นไม้/เส้นทางและอาร์เรย์ `[x,y,theta]` เรียงจาก start ไป goal ใน `planning/rrtstar_<วันเวลา>_seed<seed>/`
ใช้ `--seed 271411` เพื่อทำซ้ำ หรือ `--map maps/array_<วันเวลา>/ocgm_660610823.csv` เพื่อเลือกแมพรอบใหม่
อ่านรายละเอียดการใช้และการต่อ joint ของ CoppeliaSim ผ่าน ZeroMQ ใน [RRT_STAR.md](RRT_STAR.md)
ส่วน CoppeliaSim ยังต้องยืนยัน scene `.ttt`, กรอบพิกัด และชื่อ/ขนาด joint ก่อนทดสอบรันจริง

## Python เก็บ Map ทุกห้องจากอาร์เรย์ [x, y, r]

เปิด Gazebo ด้วย ROS bringup เดิม แล้วปิด Teleop (Ctrl+C ใน Terminal Teleop)
เพื่อให้ Python เป็นผู้สั่งขับเพียงตัวเดียว จากนั้นรันในอีก Terminal:

```bash
python3 "/home/d-dream/autonomous robot/map_all_rooms.py"
```

Python โหลด ROS environment ให้เอง เริ่มบันทึก LiDAR และวิ่งตาม `WAYPOINTS`
ใน `map_all_rooms.py` โดย `t = len(WAYPOINTS)` และวนจุด `0` ถึง `t-1`
แต่ละแถวคือ `[x, y, r]`: x/y เป็นเมตรในกรอบ world และ r เป็น yaw หน่วยเรเดียน
เส้นทางเริ่มต้นมี 32 จุดผ่านห้อง 1, 2, 3, 5, 4 แล้วกลับ (-1, 0, 0)
นับว่าถึงจุดเมื่อระยะไม่เกินประมาณ 0.20 m และ yaw ต่างไม่เกิน 0.08 rad
โปรแกรมวางเส้นทางผ่านประตูโดยใช้ collision ของ world ปัจจุบันเพื่อหลบเฟอร์นิเจอร์
แผนที่สร้างจาก LiDAR ที่วัดจริงเท่านั้น จึงยังมี unknown ในส่วนที่ถูกบัง
รถหยุดเมื่อจบเส้นทาง; กด Ctrl+C เพื่อหยุดก่อนและสร้างแมพจากข้อมูลที่เก็บแล้ว

ตรวจอาร์เรย์โดยไม่ขับ: `python3 map_all_rooms.py --check-route`
ลองแค่ 3 จุด: `python3 map_all_rooms.py --limit 3`
ดูแผนที่อัปเดตสดด้วย: `python3 map_all_rooms.py --live`
ความเร็วเริ่มต้น 0.30 m/s; ปรับด้วย `--speed 0.25`
ค่า resolution เริ่มต้น 0.05 m/cell และประมวลผลทุก 5 scans (`--scan-stride`)

ผลแต่ละรอบอยู่ใน `maps/array_<วันเวลา>/`:

- `waypoints_xyr.json`: อาร์เรย์จุดหมายที่สั่ง
- `poses_t.npy`: อาร์เรย์ตำแหน่งรถจริงขนาด N×3, เรียกแถวที่ t ด้วย `poses[t]`
- `poses_t.csv`: คอลัมน์ `t,time_s,x,y,r,dx,dy,dr`; delta เทียบกับแถว t-1 และมุม dr ใช้ช่วง [-π, π]
- `poses_t.meta.json`: หน่วย กรอบพิกัด และจำนวนตัวอย่าง; เวลาเป็นวินาทีของ simulation
- `data.csv` / `data.meta.json`: LiDAR ที่จับคู่กับ pose ณ timestamp ของ scan; x/y/yaw สามคอลัมน์แรกเป็นพิกัดตัว LiDAR ตาม recorder เดิม ส่วน `poses_t.npy` เป็นพิกัดตัวรถ
- `ocgm_660610823.csv` / `.meta.json` / `.png`: แผนที่ probability และภาพ
- `pointcloud_660610823.png` / `survey.json`: ภาพจุดและสรุปจุดหมาย/ห้องที่ไปถึง

ตัวอย่างอ่านตำแหน่งจริงหลังจบรอบ:

```python
import numpy as np
poses = np.load("maps/array_<วันเวลา>/poses_t.npy")
t = 1  # ใช้ t >= 1 เมื่อต้องการเปรียบเทียบกับแถวก่อนหน้า
print("pose[t-1] =", poses[t-1])
print("pose[t]   =", poses[t])
```

## Teleop ใน Terminal: แก้อาการรถเลี้ยวช้า

ปรับแรงเสียดทานด้านข้างของล้อให้รถ 4 ล้อหมุนแบบ skid steering ได้
ใช้ `ros2 launch robot_bringup simulation.launch.py rviz:=false` และ Teleop เดิมได้เลย
ต้องปิด Gazebo แล้วเปิดใหม่เพื่อโหลดค่าล้อที่แก้แล้ว ไม่ต้อง build ใหม่เพราะ install ใช้ symlink
`j/l` หมุนซ้าย/ขวาอยู่กับที่; `u/o` เดินหน้าพร้อมเลี้ยว; `k` หยุด
ถ้าต้องการเดินหน้าช้าลงขณะเก็บ Map หลัง source ROS ให้รัน:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.25 -p turn:=1.0
```

## เก็บแมพอัตโนมัติในฉาก 5 ห้อง

เปิด Gazebo ด้วย `bash bringup.sh` แล้วปิดหน้าต่าง Robot Teleop ก่อนรัน:

```bash
bash auto_map.sh
```

ดู Gridmap อัปเดตสดขณะรถวิ่งคู่กับ Gazebo: `bash auto_map.sh --live`
หน้าต่างแสดงแมพจากข้อมูล LiDAR ที่บันทึกจริง สีเทาคือพื้นที่ยังไม่ทราบ
เส้นแดงคือเส้นทาง LiDAR และจุดเขียวคือตำแหน่งล่าสุด หน้าต่างยังเปิดอยู่หลังรถหยุด

รถต้องเริ่มในทางเดินกลาง (|y| <= 0.35 m และ |x| <= 9 m) สคริปต์ตรวจ world 5 ห้อง
และชื่อ research_robot ก่อนใช้ตำแหน่งจริงจาก Gazebo ขับทางเดินกลางไป
(-8, 0), (8, 0) แล้วกลับ (-1, 0) โดยเดินหน้า/ถอยหลังแทนหมุนกลับ
เส้นทางนี้สำรวจเฉพาะทางเดินกลาง จึงยังไม่พาเข้าห้อง
ความเร็วเริ่มต้น 0.35 m/s จำกัดเวลาขับ 900 วินาที
สคริปต์นี้ใช้เส้นทางของฉากปัจจุบัน ไม่ใช่ระบบสำรวจอาคารใหม่โดยอิสระ
อย่าใช้ teleop หรือสคริปต์ขับอื่นพร้อมกัน ระบบปฏิเสธเมื่อพบ publisher อื่นที่ /cmd_vel

ทดลองช่วงสั้น: `bash auto_map.sh --duration 20`
ปรับความเร็ว: `bash auto_map.sh --speed 0.25`
กด Ctrl+C เพื่อหยุดรถและสร้างแมพจากข้อมูลที่เก็บแล้ว
รถหยุดเมื่อพบสิ่งกีดขวางในทิศที่ขับใกล้กว่า 0.65 m, LiDAR ผิดปกติ,
ข้อมูล scan/pose หายเกิน 1 วินาที, ไม่ขยับเกิน 15 วินาที หรือจุดหมายใช้เวลาเกิน 150 วินาที
กรณีถูกบัง สคริปต์หยุดและเก็บแมพบางส่วน ไม่มีการขับอ้อมสิ่งกีดขวางเอง

ผลอยู่ใน `maps/auto_<วันเวลา>/`: `data.csv`, `data.meta.json`,
`ocgm_660610823.csv`, `ocgm_660610823.meta.json`, `ocgm_660610823.png`,
`pointcloud_660610823.png` และ `survey.json` บอกเหตุผลที่หยุดและจำนวนจุดที่ไปถึง
ไฟล์แมพล่าสุดในโฟลเดอร์หลักจะอัปเดตเหมือน collect_map.py
แมพสร้างจาก LiDAR เท่านั้น พื้นที่ที่ถูกบังยังคง unknown แม้รถไปถึงครบทุกจุด

## วิธีใช้งานปัจจุบัน: Terminal เปิดรถ / VS Code เก็บแมพ

1. Terminal แรก source ROS และ robot_ws แล้วรัน `ros2 launch robot_bringup simulation.launch.py rviz:=false`
2. Terminal ที่สอง source ROS และ robot_ws แล้วรัน `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
3. เปิดโฟลเดอร์ `/home/d-dream/autonomous robot` ใน VS Code แล้วเปิด `collect_map.py`
4. กด **Run Python File in Terminal** (ปุ่มสามเหลี่ยมของ Python extension)
   หรือเลือก **Collect Gazebo Map** ใน Run and Debug แล้วกด F5

Python โหลด ROS environment และเปิด pose bridge ให้อัตโนมัติ ไม่ต้อง source ใน VS Code เอง
เมื่อเก็บพอ กด `k` ใน teleop ให้รถหยุด แล้วคลิก Terminal ใน VS Code ที่กำลังบันทึกและกด **Ctrl+C**
รอข้อความ `Saved map` จะได้ CSV และภาพใน `maps/<วันเวลา>/`
ใช้ Ctrl+C ใน Terminal เพื่อให้โปรแกรมสร้างแมพต่อ อย่าใช้ปุ่ม Stop/Kill Terminal เพื่อจบการเก็บ
ต้องเปิดทั้งโฟลเดอร์เพื่อให้ VS Code อ่าน `.vscode/launch.json`; ใช้ Python extension ของ VS Code

## World ปัจจุบัน: 5 ห้อง

ปรับหน้าตาฉากและเพิ่มเฟอร์นิเจอร์เมื่อ 2026-10-15: แล็บมีโต๊ะเครื่องมือและจอคอมพิวเตอร์,
คลังมีชั้นสินค้าและเคาน์เตอร์จัดส่ง, ห้องประชุมมีจอนำเสนอและตู้ข้างผนัง,
ออฟฟิศมีโต๊ะขาโปร่งพร้อมคอมพิวเตอร์/เก้าอี้/หนังสือ, ห้องช่างมีเครื่องมือและตู้ลิ้นชัก
เพิ่มพื้นไม้ พรม แผงผนัง ต้นไม้ และสีประจำห้อง โดยคงขนาดอาคาร ทางเดิน ประตู และผัง 5 ห้องเดิม
เฟอร์นิเจอร์ที่เป็นสิ่งกีดขวางมี collision; ลายพื้น/พรมเป็นผิวตกแต่งบาง ไม่ใช่ขอบนูน
ภาพจาก Gazebo: `world_design/five_rooms_perspective.png` และ `world_design/five_rooms_overview.png`

ปิด Gazebo/launch เดิมด้วย Ctrl+C แล้วเปิดด้วยคำสั่งเดิมเพื่อโหลดฉากใหม่:

```bash
source /opt/ros/jazzy/setup.bash
source /home/d-dream/robot_ws/install/setup.bash
ros2 launch robot_bringup simulation.launch.py rviz:=false
```

ไม่ต้อง build ใหม่ เพราะ world ที่ install เป็น symlink ไปยัง source ที่อัปเดตแล้ว
`map_all_rooms.py` ยังใช้อาร์เรย์ 32 จุดเดิมได้; เก็บแมพรอบใหม่ให้ตรงกับเฟอร์นิเจอร์ที่เพิ่ม
ไฟล์แมพที่เก็บก่อนปรับฉากยังเป็นข้อมูลของฉากเดิม
สำรองฉากก่อนปรับหน้าตาครั้งนี้ที่ `world_design/five_rooms_before_refresh.sdf`
หากแก้ generator ให้รัน `python3 world_design/build_five_rooms.py` แล้วคัดลอก
`world_design/five_rooms.sdf` ไปยัง `/home/d-dream/robot_ws/src/robot_gazebo/worlds/obstacle_course.sdf`
ก่อนเปิด Gazebo รอบใหม่

ห้อง 1, 3 และ 5 มีเฟอร์นิเจอร์สำหรับเปรียบเทียบ 3D กับ 2D LiDAR:
ห้องแล็บมีโต๊ะขาโปร่งและเก้าอี้ 2 ตัว, ห้องประชุมมีโต๊ะขาโปร่งและเก้าอี้พนักพิง 4 ตัว,
ห้องช่างมีโต๊ะทำงาน เก้าอี้ 1 ตัว และชั้นโปร่ง 3 ระดับพร้อมกล่องต่ำ/สูง
ระนาบ LiDAR ประมาณ z=0.35 m เห็นขาโต๊ะ ขาเก้าอี้ เสาชั้น และกล่องล่าง
แต่ไม่เห็นหน้าโต๊ะ เบาะ พนักพิง และกล่องชั้นสูงที่ไม่ตัดระนาบสแกน
ทุกชิ้นมี collision ตามรูปทรงจริง ไม่ใช่กล่องทึบครอบทั้งเฟอร์นิเจอร์
พื้นที่ free ในแผนที่ 2D หมายถึงว่างที่ระดับสแกน ไม่รับรองว่าทั้งตัวรถลอดใต้สิ่งของได้

World เริ่มต้นปรับเป็น 5 ห้องแล้ว: ด้านเหนือ 1=แล็บ, 2=คลัง, 3=ประชุม;
ด้านใต้ 4=ออฟฟิศ, 5=ห้องช่าง มีทางเดินกลางกว้าง 2.8 m และประตูประมาณ 1.8 m
ขอบเขตยังเป็น 20×16 m ชื่อ world/topic และคำสั่ง launch/collect_map เดิมใช้ได้
ตัว generator และ SDF ใหม่อยู่ใน `world_design/`; world เก่าสำรองที่
`world_design/obstacle_course_original.sdf`

CSV/ภาพตัวอย่างก่อนการเปลี่ยน world เป็นข้อมูลของฉากเก่า ต้องเก็บแมพใหม่สำหรับฉาก 5 ห้อง
อย่าเทียบข้อมูลฉากเก่ากับ geometry ของ world ใหม่ผ่าน verify_world_map.py
หากใช้ plot_ocgm_comparison.py ต้องใช้ screenshot ใหม่ของฉากนี้และ calibrate crop ให้ตรงภาพ

## ใช้ 2 คำสั่ง

Terminal แรกเปิด Gazebo พร้อมหน้าต่าง keyboard teleop (ไม่เปิด RViz):

```bash
bash "$HOME/autonomous robot/bringup.sh"
```

Terminal ที่สองเริ่มเก็บแมพ:

```bash
bash "$HOME/autonomous robot/collect_map.sh"
```

คลิกหน้าต่าง **Robot Teleop** เพื่อขับ: `i` หน้า, `,` ถอย, `j/l` หมุน, `k` หยุด
เมื่อเก็บพอ กด `k` แล้ว Ctrl+C ใน Terminal เก็บแมพ ระบบสร้าง CSV/ภาพใน `maps/<วันเวลา>/`
คำสั่งทั้งสอง source ROS ให้เอง ไม่ต้องเปิด pose bridge หรือ teleop เพิ่มอีก
ถ้าจะปิด Gazebo ให้ Ctrl+C ใน Terminal bringup หลังสร้างแมพเสร็จ
ถ้ามี bringup เดิมเปิดอยู่ ให้ปิดก่อนใช้ bringup.sh เพื่อไม่เปิด simulation ซ้ำ

## เก็บแมพเอง — ใช้ 3 Terminal

Terminal 1: เปิด Gazebo เดิม ไม่เปิด RViz

```bash
source /opt/ros/jazzy/setup.bash
source /home/d-dream/robot_ws/install/setup.bash
ros2 launch robot_bringup simulation.launch.py rviz:=false
```

Terminal 2: เริ่มเก็บข้อมูล (เปิด pose bridge ให้อัตโนมัติ)

```bash
cd '/home/d-dream/autonomous robot'
bash collect_map.sh
```

Terminal 3: ขับรถด้วย keyboard

```bash
source /opt/ros/jazzy/setup.bash
source /home/d-dream/robot_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

ใช้ `i` เดินหน้า, `,` ถอยหลัง, `j/l` หมุน, `k` หยุด และคลิกโฟกัส Terminal teleop ก่อนกด
ไม่ต้องเปิด `survey_gazebo.py` หรือ command publisher อื่นขณะขับเอง
รถเดิมอาจเลี้ยวไม่ตรง odom ให้ดูรถใน Gazebo; mapping อ่าน pose จริงแทน odom

เมื่อเก็บพอแล้ว กด `k` ให้รถหยุดใน Terminal 3 แล้วกด **Ctrl+C ใน Terminal 2**
ระบบจะสร้าง OCGM เองและบอก path ผลลัพธ์ แต่ละรอบอยู่ใน `maps/<วันเวลา>/`:

- `data.csv` และ `data.meta.json`
- `ocgm_660610823.csv` และ metadata
- `ocgm_660610823.png`
- `pointcloud_660610823.png`

ข้อมูลเก่าไม่ถูกทับ; ขั้นสร้างแมพหลังหยุดบันทึกอาจใช้เวลาตามจำนวน scans
โปรแกรมนี้ไม่ส่งคำสั่งขับรถและไม่ปิด Gazebo/teleop ของคุณ
ใช้กับ `obstacle_course` เดิม โดย pose แรกใน Gazebo PoseArray ต้องเป็น `research_robot`

ตัวเลือก: `bash collect_map.sh --duration 60` เก็บ 60 วินาที;
`bash collect_map.sh --resolution 0.05 --scan-stride 5` กำหนดรายละเอียด mapper
สร้างแมพจากข้อมูลเก่าได้ด้วย `bash collect_map.sh --from-data data.csv`

## งานส่งล่าสุด: Gazebo แทน CoppeliaSim

ใช้ `obstacle_course.sdf` เดิมแทน scene `.ttt` ตามที่อาจารย์อนุญาต
ไฟล์ `.ttt` และตัวอย่าง CoppeliaSim ที่ `/home/d-dream/robot_ws/src` ใช้ศึกษา format เท่านั้น
Gazebo เปิด `.ttt` โดยตรงไม่ได้ และไม่ต้องติดตั้ง CoppeliaSim client สำหรับ pipeline นี้
ไม่ใช้ RViz หรือ SLAM Toolbox ในการสร้างแผนที่

ผลล่าสุดคือข้อมูลขณะรถเดินสำรวจแนวทางเดินกลางฉากไป–กลับ โดยใช้ **pose จริงใน Gazebo**
เพราะการทดสอบเคลื่อนที่พบว่า wheel odom ไม่ตรงกับ pose จริง จึงไม่ใช้ odom สร้างงานส่งรอบนี้
Robot/world/controller ไม่ถูกแก้ ใช้ pose bridge เพิ่มแยกสำหรับ application

`data.csv` ล่าสุดไม่มี header เก็บ pose ของฐาน LiDAR ใน world ตามด้วย range/angle
มี 848 rows × 1,443 columns (720 rays ตาม LiDAR เดิม ไม่บังคับให้เหลือ 684)
`X,Y,TH` จาก loader มี shape `(848,1)`; `sensor_length,sensor_angle` มี shape `(848,720)`
map ใช้ทุก 5 scans เพื่อลด evidence ที่ซ้ำกัน โดยเก็บข้อมูลเต็มทุกแถวไว้ใน CSV
`data_stationary.csv` และ metadata เป็นสำเนาข้อมูลรถจอดเดิม
`data_survey.csv` เป็นชุด odom ที่ไม่ผ่านตรวจ pose ไม่ใช้สร้าง final map

เปิด Gazebo เดิมโดยไม่เปิด RViz:

```bash
source /opt/ros/jazzy/setup.bash
source /home/d-dream/robot_ws/install/setup.bash
ros2 launch robot_bringup simulation.launch.py rviz:=false
```

เปิด pose bridge อีก terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/world/obstacle_course/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V' \
  --ros-args -r /world/obstacle_course/dynamic_pose/info:=/ocgm/world_pose_array
```

ตรวจแล้วว่า pose แรกใน topic นี้คือ `research_robot` และ model frame ตรงกับฐานรถ
PoseArray ไม่มีชื่อ model ดังนั้นถ้าเปลี่ยน scene/จำนวน dynamic models ต้องตรวจลำดับใหม่ก่อนใช้
recorder interpolation ตาม timestamp ของ scan และปฏิเสธช่วง pose ห่างกันเกิน 0.1 วินาที

เก็บข้อมูลขณะขับด้วย teleop เดิม:

```bash
cd '/home/d-dream/autonomous robot'
source /opt/ros/jazzy/setup.bash
python3 save_laser_show_pointcloud.py --output another.csv \
  --world-x 0 --world-y 0 --world-yaw 0 \
  --world-pose-topic /ocgm/world_pose_array
```

ใช้ชื่อใหม่เพื่อไม่ทับ `data.csv` ปัจจุบัน แล้ว Ctrl-C หยุดบันทึก
สำหรับ survey อัตโนมัติของฉากนี้ ใช้ `python3 survey_gazebo.py` ใน terminal ที่ source ROS แล้ว
ต้องไม่มี teleop หรือ command publisher อื่นทำงานพร้อมกัน เส้นทางนี้จำกัดอยู่ corridor กลาง

สร้างงานส่งจากข้อมูลล่าสุด:

```bash
python3 load_laser_show_pointcloud.py
python3 myworld_ocgm_660610823.py --scan-stride 5
python3 verify_world_map.py
python3 plot_ocgm_comparison.py
```

`load_laser_pointcloud.py` เป็น alias ของ loader ตามชื่อที่ระบุในโจทย์
`gazebo_top_view.png` เป็นภาพที่จับจาก Gazebo จริง ไม่ใช่ภาพสร้างจาก occupancy map
`ocgm_vs_gazebo_660610823.png` เทียบ OCGM กับภาพนั้นใน world axes และ equal aspect
ภาพ camera ถูกเทียบสเกลพื้นจากกำแพงขอบ world; ผิว obstacle ที่สูงและเงาอาจต่างจาก LiDAR slice
crop ใน comparison script ใช้กับภาพ 581×797 และ camera (0,0,17), quaternion (−0.5,0.5,0.5,0.5)
ถ้าจับภาพใหม่ด้วยมุมหรือขนาดต่างกัน ต้อง calibrate crop ใหม่

การสำรวจ corridor ไม่เท่ากับสำรวจทุกห้อง/ทุกด้าน พื้นที่ถูกบังยังคง unknown ตามจริง

## รายละเอียด implementation และผลรอบแรก (ข้อมูลรถจอด)

ใช้รถ, controller, LiDAR และ world เดิมจาก `/home/d-dream/robot_ws` โดยไม่แก้ไฟล์เดิม
ไฟล์ในโฟลเดอร์นี้เป็น application เพิ่มเติม รันโดยตรงได้ ไม่ต้อง build ROS package ใหม่

## ข้อมูลที่ตรวจจากโปรเจกต์และ runtime

- World เริ่มต้นจริง: `obstacle_course.sdf` พื้น 20 × 16 m กำแพงอยู่ที่ x=±10, y=±8
- Grid ใช้ขอบเขต x=[−10.5,10.5), y=[−8.5,8.5), resolution 0.05 m: 340 rows × 420 columns
- `/scan`: LaserScan, `lidar_link`, 720 rays, 10 Hz ตาม config, range 0.15–12 m
- มุมจริงเริ่ม −3.141592741 rad, increment 0.008738783188 rad; คำนวณทุกมุมจาก LaserScan
- `/odom` และ TF `odom → base_footprint` ใช้งานได้; recorder ใช้ TF ณ timestamp ของ scan
- TF sensor mount มี x=0.12 m และ yaw=0; อ่านจาก TF แล้วเก็บใน metadata
- ตรวจ Gazebo world pose แล้วรถเริ่ม x=−1, y≈0, yaw≈0 แต่ odom เริ่ม x≈0
  จึงต้องกำหนด world-from-odom เป็น (−1,0,0) สำหรับ launch เริ่มต้นนี้
  ถ้าเปลี่ยน spawn หรือแหล่ง pose ต้องตรวจ alignment แล้วเปลี่ยน arguments ให้ตรง

## รันกับ simulation เดิม

เปิด simulation ถ้ายังไม่ได้เปิด:

```bash
source /opt/ros/jazzy/setup.bash
source /home/d-dream/robot_ws/install/setup.bash
ros2 launch robot_bringup simulation.launch.py rviz:=false
```

อีก terminal:

```bash
cd '/home/d-dream/autonomous robot'
source /opt/ros/jazzy/setup.bash
python3 scan_to_csv.py --world-x -1 --world-y 0 --world-yaw 0
```

ขับรถด้วยระบบควบคุมเดิมเพื่อเก็บบริเวณที่ต้องการ แล้ว Ctrl-C หยุด recorder
ใช้ `--duration 20` เพื่อบันทึก 20 วินาที หรือ `--output another.csv` เพื่อเปลี่ยนชื่อ
recorder เปิดแบบ exclusive จึงไม่ทับข้อมูลเก่า ต้องใช้ชื่อใหม่หรือย้ายข้อมูลเก่าก่อน
ต้องมี `rclpy`, `sensor_msgs`, `tf2_ros` จาก ROS Jazzy และ `numpy`, `matplotlib` สำหรับ mapper

```bash
python3 myworld_ocgm_660610823.py
python3 -m unittest -v test_ocgm.py
python3 verify_world_map.py
```

Mapper รองรับ `--input another.csv`, `--output another_map.csv`, `--resolution 0.05`,
`--bounds XMIN XMAX YMIN YMAX` และ `--show` สำหรับเปิด plot
`verify_world_map.py` ใช้ตรวจ world obstacle_course นี้โดยเฉพาะ ไม่ใช่ validator ทุก SDF

## ไฟล์และความหมาย

`data.csv` มี header และแต่ละแถวเป็น `x,y,yaw,range_0,angle_0,...`:
x,y,yaw เป็น vehicle pose ใน world, angle เป็นมุม sensor-relative หน่วย radians
720 rays ทำให้มี 1,443 columns; ค่า inf/NaN ของ sensor ถูกเก็บตามจริง
`data.meta.json` เก็บ range limits, sensor mount และ world alignment ต้องเก็บคู่กับ CSV

Mapper เริ่ม log odds=0, อัปเดต free ด้วย log(0.3/0.7) และ hit ด้วย log(0.7/0.3)
ใช้ Bresenham จาก sensor origin, จำกัด log odds ±10 และแปลงเป็น probability ก่อน save
แต่ละ ray/scan เพิ่ม evidence แยกกัน ค่า probability จึงเป็น inverse sensor model
ไม่ใช่การรับรองความมั่นใจทางสถิติของ scan ที่มีความสัมพันธ์กัน
inf/max-range ไม่สร้าง obstacle ที่ endpoint; NaN/−inf/below-minimum ถูกข้าม
endpoint นอก grid ไม่สร้างกำแพงปลอมที่ขอบ

`ocgm_660610823.csv` เป็น array 2D ล้วน ไม่มี header: ใกล้ 0=free, 0.5=unknown,
ใกล้ 1=occupied แถวเพิ่มตาม world Y, column เพิ่มตาม world X
`ocgm_660610823.meta.json` เก็บ extent, resolution และ shape
`ocgm_660610823.png` แสดงพิกัด Gazebo, equal aspect และ trajectory สีแดง
`verification_report.json` เก็บผลตรวจ map เทียบผิว collision ของ world
Ground truth ใช้ตรวจเท่านั้น ไม่ได้ใช้สร้าง final map

การทดสอบเริ่มต้นเก็บรถจอดนิ่ง จึงตรวจสิ่งกีดขวางที่มองเห็นจากจุดเริ่มต้นได้
บริเวณที่ถูกบังและภายใน obstacle อาจยังเป็น unknown; ต้องขับสำรวจเพื่อเพิ่ม coverage
