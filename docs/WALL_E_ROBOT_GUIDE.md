# Wall-E Robot — Hướng Dẫn Toàn Diện cho Newbie ROS2

> **Môi trường:** Ubuntu 22.04 · ROS2 Humble · Gazebo Sim (Ignition Fortress 6)
> **Build status:** ✅ Biên dịch thành công · `robot_state_publisher` hoạt động đúng

---

## Mục lục

1. [Kiến trúc file — Sơ đồ cha con](#1-kiến-trúc-file--sơ-đồ-cha-con)
2. [Luồng hoạt động căn bản](#2-luồng-hoạt-động-căn-bản)
3. [Khái niệm cho Newbie](#3-khái-niệm-cho-newbie)
4. [Cấu hình Gazebo Sim & Bridge — Lỗi thường gặp](#4-cấu-hình-gazebo-sim--bridge--lỗi-thường-gặp)
5. [Bảng lệnh "Sống còn"](#5-bảng-lệnh-sống-còn)

---

## 1. Kiến trúc file — Sơ đồ cha con

```
wall-e-robot/                          ← Gốc của ROS2 package
│
├── package.xml                        ← Khai báo package (tên, phụ thuộc, maintainer)
├── CMakeLists.txt                     ← Hướng dẫn build: copy file vào install/
│
├── description/                       ← MÔ TẢ VẬT LÝ CỦA ROBOT (URDF/Xacro)
│   ├── robot.urdf.xacro               ← "File gốc" — điểm vào duy nhất, include hai file dưới
│   │   ├── robot_core.xacro           ← Định nghĩa toàn bộ link + joint (thân, bánh xe...)
│   │   │   └── inertial_macros.xacro  ← Thư viện macro tính moment quán tính (inertia)
│   │   └── gazebo_control.xacro       ← Plugin Gazebo Sim: DiffDrive + JointStatePublisher
│
├── launch/                            ← CÁC FILE KHỞI ĐỘNG HỆ THỐNG
│   ├── rsp.launch.py                  ← Chỉ chạy robot_state_publisher (dùng cho RViz thuần)
│   └── launch_sim.launch.py           ← Chạy đầy đủ: RSP + Gazebo Sim + spawn + bridge
│
├── config/                            ← CẤU HÌNH THAM SỐ
│   ├── gz_bridge.yml                  ← Quy tắc chuyển đổi topic GZ ↔ ROS2
│   └── empty.yaml                     ← File placeholder (để git commit được folder)
│
└── worlds/
    └── empty.world                    ← Thế giới Gazebo (SDF): chỉ có mặt đất + mặt trời
```

### Quan hệ include giữa các file Xacro

```
robot.urdf.xacro  (file entry point)
        │
        ├──► robot_core.xacro
        │           │
        │           └──► inertial_macros.xacro
        │
        └──► gazebo_control.xacro
```

**Tại sao tách thành nhiều file?**
- `robot_core.xacro` → Mô tả hình dạng thuần túy (không phụ thuộc simulator)
- `gazebo_control.xacro` → Phần phụ thuộc Gazebo (dễ swap sang simulator khác)
- `inertial_macros.xacro` → Tái sử dụng công thức tính inertia cho mọi shape

---

## 2. Luồng hoạt động căn bản

### 2.1 Khi chạy `ros2 launch wall-e-robot rsp.launch.py` (chỉ RViz)

```
[Terminal]
    │
    ▼
rsp.launch.py
    │  1. Đọc robot.urdf.xacro
    │  2. Gọi xacro.process_file() → xuất ra chuỗi XML (URDF thuần)
    │  3. Truyền chuỗi đó vào parameter robot_description
    ▼
robot_state_publisher (node)
    │  - Publish /robot_description  (latched topic)
    │  - Publish /tf_static           (transform cố định: chassis, caster)
    │  - Subscribe /joint_states      (transform động: bánh xe)
    ▼
RViz2 (mở thủ công)
    │  - Nhận /robot_description → vẽ hình robot
    │  - Nhận /tf, /tf_static → định vị từng khớp
```

### 2.2 Khi chạy `ros2 launch wall-e-robot launch_sim.launch.py` (đầy đủ với Gazebo)

```
launch_sim.launch.py
        │
        ├──[1] include rsp.launch.py (use_sim_time=true)
        │           └─► robot_state_publisher node khởi động
        │
        ├──[2] include gz_sim.launch.py (từ ros_gz_sim package)
        │           └─► Gazebo Sim mở, nạp world empty.sdf
        │
        ├──[3] Node ros_gz_sim/create
        │           └─► Đọc /robot_description → spawn robot vào Gazebo
        │               (tên model: my_bot, z=0.1m)
        │
        └──[4] Node ros_gz_bridge/parameter_bridge
                    └─► Đọc config/gz_bridge.yml → tạo bridge hai chiều

Sau khi khởi động xong, luồng dữ liệu runtime:

Gazebo Sim                           ROS2
─────────────                        ─────────────────────
Plugin DiffDrive ──────────────────► /odom      (nav_msgs/Odometry)
Plugin DiffDrive ──────────────────► /tf        (tf2_msgs/TFMessage)
Plugin JointState ─────────────────► /joint_states (sensor_msgs/JointState)
gz clock ──────────────────────────► /clock     (rosgraph_msgs/Clock)

/cmd_vel (geometry_msgs/TwistStamped) ─────────► Plugin DiffDrive → quay bánh

robot_state_publisher:
  /joint_states ─────────────────────────────► /tf (transform động bánh xe)
  /robot_description (static) ───────────────► /tf_static
```

### 2.3 Bảng node & nhiệm vụ

| Node | Package | Nhiệm vụ |
|---|---|---|
| `robot_state_publisher` | robot_state_publisher | Parse URDF, publish TF cho toàn robot |
| `gz_sim` (Gazebo) | ros_gz_sim | Chạy simulation vật lý |
| `create` (spawner) | ros_gz_sim | Spawn model robot vào Gazebo |
| `parameter_bridge` | ros_gz_bridge | Bridge topic giữa Gazebo ↔ ROS2 |

---

## 3. Khái niệm cho Newbie

### 🔧 URDF (Unified Robot Description Format)
**Là gì?** Một file XML mô tả robot: hình dạng, kích thước, vật liệu, các khớp nối.
**Ví von:** Như bản vẽ kỹ thuật của robot. Bạn mô tả "thân dài 30cm, bánh trái đường kính 10cm, gắn tại vị trí xyz...".
**Hạn chế:** Viết tay rất dài dòng và không hỗ trợ biến số hay vòng lặp.

### ✂️ Xacro (XML Macros)
**Là gì?** Một preprocessor (bộ tiền xử lý) cho URDF. Xacro file `.urdf.xacro` → xacro xử lý → xuất ra URDF thuần.
**Xacro cho phép bạn:**
- Dùng biến: `${pi/2}`, `${mass}`
- Định nghĩa macro tái sử dụng: `<xacro:macro name="inertial_box" params="mass x y z">`
- Include file khác: `<xacro:include filename="robot_core.xacro"/>`
**Ví von:** Như hàm trong lập trình. Thay vì copy-paste công thức inertia 3 lần, bạn định nghĩa 1 lần và gọi lại.

### 🧱 Link
**Là gì?** Một bộ phận cứng (rigid body) của robot.
**Trong dự án này:**
- `base_link` — điểm tham chiếu gốc (invisible, không có hình dạng)
- `chassis` — thân hộp trắng 30×30×15cm
- `left_wheel`, `right_wheel` — bánh trụ màu xanh
- `caster_wheel` — bánh tròn màu đen phía trước
**Ví von:** Như các xương trong cơ thể.

### 🔗 Joint
**Là gì?** Kết nối giữa hai link, định nghĩa loại chuyển động cho phép.
**Các loại joint trong dự án:**
- `fixed` — không chuyển động (chassis_joint, caster_wheel_joint)
- `continuous` — quay không giới hạn (left_wheel_joint, right_wheel_joint)
**Ví von:** Như các khớp xương. Khớp cứng (fixed) = hàn chết. Khớp liên tục (continuous) = bánh xe quay mãi.

### 🗺️ TF (Transform Frame)
**Là gì?** Hệ thống theo dõi vị trí và góc xoay của MỌI link so với nhau theo thời gian thực.
**Hoạt động:** `robot_state_publisher` nhận `/joint_states` (góc bánh xe hiện tại) → tính toán → publish `/tf` cho toàn bộ cây link.
**Tại sao cần?** RViz cần biết "link left_wheel hiện đang ở đâu trong không gian?" để vẽ đúng.
**Ví von:** Như GPS theo dõi vị trí từng bộ phận của robot trong thế giới thực.

### 📡 Topic
**Là gì?** Kênh truyền thông tin kiểu publish/subscribe trong ROS2.
**Hoạt động:** Node A publish lên topic → Node B subscribe topic đó để nhận.
**Các topic quan trọng trong dự án:**

| Topic | Kiểu dữ liệu | Ý nghĩa |
|---|---|---|
| `/robot_description` | `String` | Chuỗi URDF của robot |
| `/joint_states` | `sensor_msgs/JointState` | Góc/vận tốc bánh xe hiện tại |
| `/tf` | `tf2_msgs/TFMessage` | Transform động (bánh xe) |
| `/tf_static` | `tf2_msgs/TFMessage` | Transform tĩnh (chassis) |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Lệnh di chuyển (vận tốc) |
| `/odom` | `nav_msgs/Odometry` | Vị trí ước tính của robot |
| `/clock` | `rosgraph_msgs/Clock` | Đồng hồ simulation từ Gazebo |

---

## 4. Cấu hình Gazebo Sim & Bridge — Lỗi thường gặp

### 4.1 Cấu trúc Bridge (`config/gz_bridge.yml`)

Bridge là "phiên dịch viên" giữa ROS2 và Gazebo Sim. Mỗi entry trong file có cấu trúc:

```yaml
- ros_topic_name: "tên_topic_ros2"
  gz_topic_name:  "tên_topic_gazebo"
  ros_type_name:  "package/msg/MessageType"
  gz_type_name:   "gz.msgs.MessageType"
  direction: GZ_TO_ROS | ROS_TO_GZ | BIDIRECTIONAL
```

**Bridge hiện tại:**

| Topic | Hướng | Mục đích |
|---|---|---|
| `clock` | GZ → ROS | Đồng bộ thời gian simulation |
| `cmd_vel` | ROS → GZ | Gửi lệnh di chuyển vào Gazebo |
| `odom` | GZ → ROS | Gazebo gửi odometry ra ROS |
| `tf` | GZ → ROS | Gazebo gửi transform ra ROS |
| `joint_states` | GZ → ROS | Trạng thái bánh xe từ Gazebo |

### 4.2 Plugin trong `gazebo_control.xacro`

```xml
<!-- Plugin điều khiển xe 2 bánh vi sai (Differential Drive) -->
<plugin name="gz::sim::systems::DiffDrive" filename="gz-sim-diff-drive-system">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.35</wheel_separation>   <!-- khoảng cách 2 bánh: 35cm -->
    <wheel_radius>0.05</wheel_radius>            <!-- bán kính bánh: 5cm -->
    <topic>cmd_vel</topic>                       <!-- nhận lệnh từ topic này -->
    <odom_topic>odom</odom_topic>
    <tf_topic>tf</tf_topic>
</plugin>

<!-- Plugin publish trạng thái bánh xe -->
<plugin name="gz::sim::systems::JointStatePublisher" filename="gz-sim-joint-state-publisher-system">
    <topic>joint_states</topic>
    <joint_name>left_wheel_joint</joint_name>
    <joint_name>right_wheel_joint</joint_name>
</plugin>
```

### 4.3 ⚠️ Lỗi đã xác nhận & cách sửa

#### BUG 1 — Tên file bridge sai extension (CRITICAL)

**File:** `launch/launch_sim.launch.py`, dòng 52

```python
# ❌ SAI — file không tồn tại
bridge_params = os.path.join(..., 'config', 'gz_bridge.yaml')

# ✅ ĐÚNG — đúng tên file thực tế
bridge_params = os.path.join(..., 'config', 'gz_bridge.yml')
```

**Triệu chứng:** Bridge khởi động nhưng không load config, không có topic `/odom`, `/tf` từ Gazebo.

**Cách sửa:** Mở `launch/launch_sim.launch.py` và thay `.yaml` thành `.yml` ở dòng 52.

---

#### BUG 2 — Collision geometry bánh xe có 2 shapes (cảnh báo)

**File:** `description/robot_core.xacro`, dòng 69–73 và 94–100

```xml
<!-- ❌ SAI — chỉ được có 1 shape trong <geometry> -->
<collision>
    <geometry>
        <cylinder length="0.04" radius="0.05" />
        <sphere radius="0.05" />    <!-- ← phần này bị bỏ qua hoặc gây lỗi parser -->
    </geometry>
</collision>

<!-- ✅ ĐÚNG — giữ lại chỉ cylinder -->
<collision>
    <geometry>
        <cylinder length="0.04" radius="0.05" />
    </geometry>
</collision>
```

**Triệu chứng:** Cảnh báo trong log, hoặc hành vi va chạm không như kỳ vọng trong Gazebo.

---

#### BUG 3 — `cmd_vel` type mismatch với teleop tools

**Vấn đề:** Bridge config dùng `geometry_msgs/msg/TwistStamped` nhưng hầu hết teleop tools (ví dụ `teleop_twist_keyboard`) publish `geometry_msgs/msg/Twist` (không có header).

**Cách kiểm tra:**
```bash
ros2 topic info /cmd_vel --verbose
```

**Cách sửa nếu dùng teleop_twist_keyboard:**
```yaml
# Trong gz_bridge.yml, đổi thành:
- ros_topic_name: "cmd_vel"
  gz_topic_name: "cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

---

#### LỖI THƯỜNG GẶP KHÁC

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `[ERROR] [gz_sim]: Unable to find world 'empty.sdf'` | World mặc định của Gazebo Sim không có `empty.sdf` trong path | Chạy với `world:=/path/to/empty.sdf` hoặc không truyền arg world |
| `[robot_state_publisher]: No urdf file found` | Chưa build hoặc chưa source | `colcon build` rồi `source install/setup.bash` |
| `[gz_bridge]: No route to topic '/tf'` | Chưa start bridge hoặc file config không tìm thấy | Kiểm tra bug #1 ở trên |
| `ResourceNotFound: wall-e-robot` | Package chưa được source | `source ~/dev_ws/install/setup.bash` |
| Robot spawn nhưng ngã xuống | Inertia quá nhỏ hoặc quá lớn | Kiểm tra lại giá trị mass trong `inertial_macros.xacro` |
| `/odom` không có data | Bridge không chạy hoặc config sai | Kiểm tra `ros2 node list` xem có `parameter_bridge` không |

---

### 4.4 Kiểm tra bridge đang hoạt động

```bash
# Liệt kê tất cả topic đang active
ros2 topic list

# Xem data từ Gazebo có qua bridge không
ros2 topic echo /odom --once

# Xem TF tree có đầy đủ không
ros2 run tf2_tools view_frames
```

---

## 5. Bảng lệnh "Sống còn"

### 🔨 Build & Source

| Mục đích | Lệnh |
|---|---|
| Build toàn bộ workspace | `cd ~/dev_ws && colcon build` |
| Build chỉ package này | `cd ~/dev_ws && colcon build --packages-select wall-e-robot` |
| Build nhanh (bỏ qua test) | `cd ~/dev_ws && colcon build --packages-select wall-e-robot --cmake-args -DBUILD_TESTING=OFF` |
| Source ROS2 system | `source /opt/ros/humble/setup.bash` |
| Source workspace | `source ~/dev_ws/install/setup.bash` |
| Source cả hai (dán vào `~/.bashrc`) | `source /opt/ros/humble/setup.bash && source ~/dev_ws/install/setup.bash` |
| Xem log build | `cat ~/dev_ws/log/latest_build/wall-e-robot/stdout_stderr.log` |

### 👁️ Chạy RViz (không Gazebo)

| Mục đích | Lệnh |
|---|---|
| Chỉ khởi động robot state publisher | `ros2 launch wall-e-robot rsp.launch.py` |
| Mở RViz2 riêng | `rviz2` |
| Mở RViz2 với config mặc định | `ros2 run rviz2 rviz2` |
| Xem robot description dạng text | `ros2 param get /robot_state_publisher robot_description` |
| Xem TF tree | `ros2 run tf2_tools view_frames` |
| Echo TF realtime | `ros2 topic echo /tf` |

### 🤖 Chạy Simulation (Gazebo Sim)

| Mục đích | Lệnh |
|---|---|
| Chạy simulation đầy đủ | `ros2 launch wall-e-robot launch_sim.launch.py` |
| Chạy với world cụ thể | `ros2 launch wall-e-robot launch_sim.launch.py world:=empty.sdf` |
| Điều khiển robot bằng bàn phím | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |
| Xem tất cả topic | `ros2 topic list` |
| Xem tốc độ publish topic | `ros2 topic hz /odom` |
| Echo odometry | `ros2 topic echo /odom` |
| Xem node đang chạy | `ros2 node list` |
| Xem info node | `ros2 node info /robot_state_publisher` |

### 🔍 Debug & Kiểm tra

| Mục đích | Lệnh |
|---|---|
| Kiểm tra package đã install | `ros2 pkg list \| grep wall-e` |
| Xem file trong package | `ros2 pkg prefix wall-e-robot` |
| Validate URDF/Xacro | `check_urdf <(xacro ~/dev_ws/src/wall-e-robot/description/robot.urdf.xacro)` |
| Chuyển xacro sang URDF | `xacro ~/dev_ws/src/wall-e-robot/description/robot.urdf.xacro > /tmp/robot.urdf` |
| Kiểm tra xacro parse được không | `xacro ~/dev_ws/src/wall-e-robot/description/robot.urdf.xacro` |
| Xem graph kết nối node/topic | `ros2 run rqt_graph rqt_graph` |
| Xem joint state realtime | `ros2 topic echo /joint_states` |

### 🗑️ Dọn dẹp

| Mục đích | Lệnh |
|---|---|
| Xóa toàn bộ build cache | `cd ~/dev_ws && rm -rf build/ install/ log/` |
| Xóa build của 1 package | `cd ~/dev_ws && rm -rf build/wall-e-robot install/wall-e-robot` |
| Kill tất cả Gazebo process | `pkill -f gz_sim; pkill -f gzserver; pkill -f gzclient` |

---

## Phụ lục: Cấu trúc tổng quan hệ thống khi simulation chạy

```
┌─────────────────────────────────────────────────────┐
│                   GAZEBO SIM                         │
│                                                     │
│  ┌──────────────────────────────────────────────┐   │
│  │  my_bot (URDF model)                         │   │
│  │  ┌──────────────┐  ┌─────────────────────┐  │   │
│  │  │  DiffDrive   │  │ JointStatePublisher │  │   │
│  │  │  Plugin      │  │ Plugin              │  │   │
│  │  └──────┬───────┘  └──────────┬──────────┘  │   │
│  └─────────┼────────────────────┼─────────────┘   │
│            │ gz topics          │ gz topics         │
└────────────┼────────────────────┼───────────────────┘
             │ bridge             │ bridge
┌────────────┼────────────────────┼───────────────────┐
│            ▼                    ▼    ROS2 GRAPH      │
│       /cmd_vel              /joint_states            │
│       /odom                 /tf                      │
│       /clock                                         │
│                                                     │
│  ┌──────────────────────────┐                        │
│  │  robot_state_publisher   │                        │
│  │  /joint_states ─────────►  /tf (dynamic)          │
│  │  /robot_description ────►  /tf_static             │
│  └──────────────────────────┘                        │
│                                                     │
│  ┌───────────────┐   ┌─────────────────────────┐    │
│  │  RViz2        │   │  teleop_twist_keyboard   │    │
│  │  (visualize)  │   │  publish /cmd_vel        │    │
│  └───────────────┘   └─────────────────────────┘    │
└─────────────────────────────────────────────────────┘
```

---

*File này được tạo tự động sau khi xác nhận build thành công và RSP hoạt động đúng.*
*Cập nhật lần cuối: 2026-04-12*
