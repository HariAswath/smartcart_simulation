# SmartCart Simulation — Implementation Plan

> **Source of truth:** `SMARTCART_STANDALONE_SIMULATION_SPEC.md`
> **Target:** ROS 2 Jazzy + Gazebo Harmonic 8.x + Ubuntu 24.04
> **Workspace:** `~/smartcart_simulation/smartcart_ws`

---

## Environment (Already Verified ✅)

| Tool | Status |
|---|---|
| ROS 2 Jazzy | ✅ `/opt/ros/jazzy` |
| Gazebo Harmonic 8.15.0 | ✅ |
| `ros_gz_sim` | ✅ |
| `ros_gz_bridge` | ✅ |
| `ros_gz_interfaces` | ✅ |
| `robot_state_publisher` | ✅ |
| `xacro` | ✅ |
| `colcon` | ✅ `/usr/bin/colcon` |

---

## Final Source Tree (Target)

```
smartcart_simulation/
└── smartcart_ws/
    └── src/
        ├── smartcart_description/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   └── urdf/
        │       └── smartcart.urdf.xacro
        │
        ├── smartcart_gazebo/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── launch/
        │   │   └── smartcart.launch.py
        │   ├── worlds/
        │   │   └── smartcart_world.sdf
        │   └── models/
        │       └── human/
        │           ├── model.config
        │           └── model.sdf
        │
        └── smartcart_human/
            ├── package.xml
            ├── setup.py
            ├── resource/
            │   └── smartcart_human
            └── smartcart_human/
                ├── __init__.py
                ├── human_controller.py
                ├── follow_controller.py
                └── rfid_simulator.py
```

---

## ROS Interface Contract (Fixed — Do Not Rename)

| Topic | Direction | Type |
|---|---|---|
| `/cmd_vel` | controller → robot | `geometry_msgs/msg/Twist` |
| `/scan` | LiDAR → controller | `sensor_msgs/msg/LaserScan` |
| `/human/pose` | human node → controller | `geometry_msgs/msg/Pose` |
| `/rfid/item` | RFID → consumer | `std_msgs/msg/String` |
| `/wheel/odometry` | Gazebo → ROS | odometry |
| `/camera/image_raw` | camera → ROS | `sensor_msgs/msg/Image` (optional) |

---

## Phase-by-Phase Plan

---

### PHASE 0 — Environment Verification

**Goal:** Confirm the machine has everything before writing any code.

**Steps:**
1. `lsb_release -a` → confirm Ubuntu 24.04.
2. Source `/opt/ros/jazzy/setup.bash`.
3. `gz sim --version` → Gazebo Harmonic 8.x.
4. `ros2 pkg prefix` for all required packages.
5. `which colcon`.

**Verification pass criteria:** All commands succeed with no errors.

> ✅ Already verified — all checks pass.

---

### PHASE 1 — Workspace & Package Scaffolding

**Goal:** Create the three empty-but-valid ROS 2 packages; confirm `colcon build` succeeds.

**Files to create:**

```
smartcart_ws/src/smartcart_description/
  CMakeLists.txt          (ament_cmake; install urdf/ only)
  package.xml             (ament_cmake build type)
  urdf/                   (directory placeholder for Phase 2)

smartcart_ws/src/smartcart_gazebo/
  CMakeLists.txt          (ament_cmake; installs launch/ worlds/ models/)
  package.xml             (depends: ros_gz_sim, ros_gz_bridge,
                           robot_state_publisher, xacro)
  launch/                 (placeholder)
  worlds/                 (placeholder)
  models/human/           (placeholder)

smartcart_ws/src/smartcart_human/
  package.xml             (ament_python build type)
  setup.py                (entry_points for 3 executables)
  resource/smartcart_human (marker file, required by ament_python)
  smartcart_human/__init__.py
  smartcart_human/human_controller.py   (minimal stub)
  smartcart_human/follow_controller.py  (minimal stub)
  smartcart_human/rfid_simulator.py     (minimal stub)
```

**CMake rule:** Only `install(DIRECTORY ...)` directories that actually exist on disk.

**Build & verify:**
```bash
cd ~/smartcart_simulation/smartcart_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

**Pass criteria:**
```
Summary: 3 packages finished
```

---

### PHASE 2 — Robot Description (URDF/Xacro)

**Goal:** Full SmartCart geometry with sensors and plugins so `robot_state_publisher` can broadcast TF.

**File:** `smartcart_description/urdf/smartcart.urdf.xacro`

**Robot physical dimensions:**

| Link | Geometry | Dimensions | Mass |
|---|---|---|---|
| `base_link` | box | 0.8 × 0.6 × 0.3 m | 8.0 kg |
| `left_wheel_link` | cylinder | r=0.15 m, l=0.08 m | 1.0 kg |
| `right_wheel_link` | cylinder | r=0.15 m, l=0.08 m | 1.0 kg |
| `caster_link` | sphere | r=0.05 m | 0.5 kg |
| `lidar_link` | cylinder | r=0.05 m, l=0.04 m | 0.2 kg |
| `camera_link` | box | 0.05 × 0.05 × 0.05 m | 0.1 kg |

**Joints:**

| Joint | Type | Parent → Child | Notes |
|---|---|---|---|
| `left_wheel_joint` | continuous | `base_link` → `left_wheel_link` | Y offset −0.35 m |
| `right_wheel_joint` | continuous | `base_link` → `right_wheel_link` | Y offset +0.35 m |
| `caster_joint` | fixed | `base_link` → `caster_link` | rear of base |
| `lidar_joint` | fixed | `base_link` → `lidar_link` | top center |
| `camera_joint` | fixed | `base_link` → `camera_link` | front top |

**Wheel axis orientation:** X axis — both wheels must share the same axis to produce straight forward motion in +X.

**Gazebo plugins (inside `<gazebo>` tags or Gazebo-specific URDF extension):**

1. `gz::sim::systems::DiffDrive`
   - `left_joint`: `left_wheel_joint`
   - `right_joint`: `right_wheel_joint`
   - `wheel_separation`: 0.7
   - `wheel_radius`: 0.15
   - `topic`: `/cmd_vel`
   - `odom_topic`: `/wheel/odometry`
   - `frame_id`: `odom`
   - `child_frame_id`: `base_link`
   - `odom_publisher_frequency`: 30

2. `gz::sim::systems::JointStatePublisher`

3. `gz::sim::systems::Sensors` (required for LiDAR to work)

**LiDAR sensor (on `lidar_link`):**
- type: `lidar`
- horizontal samples: 360
- angle_min: −π, angle_max: +π
- vertical samples: 1
- range_min: 0.12, range_max: 10.0
- update_rate: 10 Hz
- Gazebo topic bridged to ROS `/scan`

**Camera sensor (on `camera_link`, optional):**
- type: `camera`
- resolution: 320 × 240
- update_rate: 10 Hz
- Bridged to `/camera/image_raw`

**Inertia formula for box (mass m, x×y×z dimensions):**
```
Ixx = m/12 * (y² + z²)
Iyy = m/12 * (x² + z²)
Izz = m/12 * (x² + y²)
```

**Build & verify:**
```bash
colcon build --packages-select smartcart_description
source install/setup.bash
ros2 run xacro xacro src/smartcart_description/urdf/smartcart.urdf.xacro
# Must produce valid XML with no errors
```

**Pass criteria:** `xacro` outputs valid URDF, no errors.

---

### PHASE 3 — Gazebo World + Human Model + SmartCart Spawn

**Goal:** Supermarket world loads in Gazebo; SmartCart is visible; human model exists.

#### 3a — World SDF

**File:** `smartcart_gazebo/worlds/smartcart_world.sdf`

**Layout (top-down view, 12 m × 8 m):**

```
+Y = 4.0
         +--North wall-------------------------------+
         |                                           |
Shelf 4  |  [S4]                        [S1]  Shelf 1|
Shelf 5  |  [S5]    CENTRAL AISLE       [S2]  Shelf 2|
Shelf 6  |  [S6]         O (human x=2)  [S3]  Shelf 3|
         |               C (cart  x=0)              |
         |                                   [CHK]  |
ENT→     |  <entrance gap>                          |
         +--South wall-------------------------------+
-Y = -4.0
```

**World objects:**

| Name | Geometry | Size (x × y × z m) | Position | Static |
|---|---|---|---|---|
| Floor | plane | 12 × 8 | z=0 | ✅ |
| Wall North | box | 12.0 × 0.15 × 2.5 | x=0, y=4.0, z=1.25 | ✅ |
| Wall South | box | 12.0 × 0.15 × 2.5 | x=0, y=−4.0, z=1.25 | ✅ |
| Wall East | box | 0.15 × 8.0 × 2.5 | x=6.0, y=0, z=1.25 | ✅ |
| Wall West (left seg) | box | 0.15 × 3.5 × 2.5 | x=−6.0, y=2.25, z=1.25 | ✅ |
| Wall West (right seg) | box | 0.15 × 3.5 × 2.5 | x=−6.0, y=−2.25, z=1.25 | ✅ |
| Checkout counter | box | 1.5 × 0.6 × 1.0 | x=4.5, y=3.2, z=0.5 | ✅ |
| Shelf 1 (right-back) | box | 2.0 × 0.6 × 1.5 | x=3.0, y=−2.5, z=0.75 | ✅ |
| Shelf 2 (right-mid) | box | 2.0 × 0.6 × 1.5 | x=0.0, y=−2.5, z=0.75 | ✅ |
| Shelf 3 (right-front) | box | 2.0 × 0.6 × 1.5 | x=−3.0, y=−2.5, z=0.75 | ✅ |
| Shelf 4 (left-back) | box | 2.0 × 0.6 × 1.5 | x=3.0, y=2.5, z=0.75 | ✅ |
| Shelf 5 (left-mid) | box | 2.0 × 0.6 × 1.5 | x=0.0, y=2.5, z=0.75 | ✅ |
| Shelf 6 (left-front) | box | 2.0 × 0.6 × 1.5 | x=−3.0, y=2.5, z=0.75 | ✅ |
| Obstacle demo box | box | 0.4 × 0.4 × 0.4 | x=4.0, y=0.0, z=0.2 | ✅ |

**Physics:**
```xml
<max_step_size>0.001</max_step_size>
<real_time_update_rate>1000</real_time_update_rate>
<real_time_factor>1</real_time_factor>
```

#### 3b — Human Model

**Files:**
- `smartcart_gazebo/models/human/model.config`
- `smartcart_gazebo/models/human/model.sdf`

Human geometry (all links with collision):

| Link | Shape | Size | Mass |
|---|---|---|---|
| torso | box | 0.4 × 0.3 × 0.8 m | 50 kg |
| head | sphere | r=0.15 m | 5 kg |
| l_arm | box | 0.1 × 0.08 × 0.6 m | 3 kg |
| r_arm | box | 0.1 × 0.08 × 0.6 m | 3 kg |
| l_leg | box | 0.15 × 0.1 × 0.8 m | 5 kg |
| r_leg | box | 0.15 × 0.1 × 0.8 m | 5 kg |

**Initial pose:** x=2.0, y=0.0, z=0.0

#### 3c — Launch File

**File:** `smartcart_gazebo/launch/smartcart.launch.py`

Starts in this order:
1. Gazebo Harmonic → `gz sim -r smartcart_world.sdf`
2. `robot_state_publisher` → publishes URDF from xacro
3. `ros_gz_sim create` → spawns SmartCart at x=0, y=0, z=0.3, yaw=0

**Build & verify:**
```bash
colcon build
source install/setup.bash
ros2 launch smartcart_gazebo smartcart.launch.py
```

**Pass criteria:**
- Gazebo opens showing supermarket environment.
- SmartCart visible at origin.
- Human visible at x=2.0.
- No errors in terminal.

---

### PHASE 4 — Differential Drive Verification

**Goal:** Confirm diff drive plugin is working — robot moves on `/cmd_vel`.

**No new files.** Plugin was defined in Phase 2 URDF.

**Test commands:**
```bash
# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Rotate
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Verify odometry
ros2 topic echo /wheel/odometry --once
```

**Pass criteria:**
- Robot moves forward, backward (via negative x), and rotates in Gazebo.
- `/wheel/odometry` publishes pose/twist data.

---

### PHASE 5 — LiDAR Bridge

**Goal:** `/scan` publishes `sensor_msgs/msg/LaserScan` in ROS at ~10 Hz.

**Addition to launch file:** ROS-Gazebo bridge process:
```python
ros_gz_bridge  --ros-args -p config_file:=<bridge_config_yaml>
```

Bridge config YAML:
```yaml
- ros_topic_name: /scan
  gz_topic_name: /scan
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS

- ros_topic_name: /wheel/odometry
  gz_topic_name: /model/smartcart/odometry
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS
```

**Verify:**
```bash
ros2 topic type /scan
# sensor_msgs/msg/LaserScan

ros2 topic hz /scan
# ~10.0 Hz

ros2 topic echo /scan --once
# ranges array with finite values near objects
```

**Pass criteria:**
- `/scan` at ~10 Hz with `range_min=0.12`, `range_max=10.0`.
- Ranges change when an object is placed in front.

---

### PHASE 6 — Simulated Human + `/human/pose`

**Goal:** `human_controller` node moves the human model in Gazebo and publishes position.

**File:** `smartcart_human/smartcart_human/human_controller.py`

**Algorithm:**
- Node name: `human_controller`
- Wait for `SetEntityPose` Gazebo service.
- Oscillate human: x moves 2.0 m → 6.0 m → 2.0 m at 0.2 m/s, y=0.
- Timer period: 0.1 s.
- At each step: call SetEntityPose service → publish `geometry_msgs/msg/Pose` to `/human/pose`.

**Key implementation note:**
- Verify actual service name at runtime: `ros2 service list | grep pose`
- Use `ros_gz_interfaces/srv/SetEntityPose` (check field names with `ros2 interface show`)

**Added to launch file:** `human_controller` node.

**Verify:**
```bash
ros2 topic echo /human/pose
# position.x must change between 2.0 and 6.0 over time
```

**Pass criteria:**
- `/human/pose` publishes continuously.
- `position.x` oscillates between 2.0 and 6.0.
- Human model moves visibly in Gazebo.

---

### PHASE 7 — Human-Follow Controller

**Goal:** SmartCart follows the human, maintains ~1.2 m distance, turns toward human.

**File:** `smartcart_human/smartcart_human/follow_controller.py`

**Controller constants (from spec section I):**
```python
TARGET_DISTANCE        = 1.2   # m
K_DISTANCE             = 0.5
K_ANGLE                = 1.0
MAX_LINEAR_SPEED       = 0.5   # m/s
MAX_ANGULAR_SPEED      = 1.0   # rad/s
OBSTACLE_STOP_DISTANCE = 0.6   # m
HUMAN_TIMEOUT          = 1.0   # s
```

**Subscriptions:**
- `/human/pose` → `geometry_msgs/msg/Pose`
- `/scan` → `sensor_msgs/msg/LaserScan`
- `/wheel/odometry` → `nav_msgs/msg/Odometry` (provides `rx`, `ry`, `yaw`)

**Publisher:** `/cmd_vel` → `geometry_msgs/msg/Twist`

**Control loop (10 Hz timer):**

```python
# Step 1: Human timeout check
age = now - last_human_time
if age > HUMAN_TIMEOUT:
    publish_stop(); return

# Step 2: Obstacle check (details in Phase 8)
if front_min < OBSTACLE_STOP_DISTANCE:
    publish_stop(); return

# Step 3: Coordinate transform to robot frame
dx = human_world_x - robot_x
dy = human_world_y - robot_y
human_x_local =  cos(yaw)*dx + sin(yaw)*dy
human_y_local = -sin(yaw)*dx + cos(yaw)*dy

# Step 4: Distance
d = sqrt(human_x_local**2 + human_y_local**2)

# Step 5: Velocity commands
if d <= TARGET_DISTANCE:
    linear_x = 0.0
else:
    linear_x = min(K_DISTANCE * (d - TARGET_DISTANCE), MAX_LINEAR_SPEED)

angular_z = max(-MAX_ANGULAR_SPEED,
                min(K_ANGLE * human_y_local, MAX_ANGULAR_SPEED))

# Step 6: Publish
```

**Throttled status log (~1 Hz):**
```
Human: DETECTED | Dist: 1.37m | Obstacle: CLEAR | State: FOLLOW | v=0.08 w=0.12
```

**Shutdown safety:** Publish zero `Twist` before exit.

**Added to launch file:** `follow_controller` node.

**Verify:**
```bash
ros2 topic echo /cmd_vel
# Non-zero linear.x when human is >1.2m away
# Positive angular.z when human is to the left
```

**Pass criteria:**
- Cart follows human forward.
- Cart turns correctly left/right.
- Cart stops forward motion when within 1.2 m.

---

### PHASE 8 — Obstacle Emergency Stop

**Goal:** Cart stops when any obstacle enters the front 60° LiDAR sector within 0.6 m.

**Implementation inside `follow_controller.py` (from Phase 7).**

**LiDAR processing (from spec section G):**
```python
def get_front_min_range(scan):
    front_min = float('inf')
    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        if -math.pi/6 <= angle <= math.pi/6:   # ±30°
            if math.isnan(r) or math.isinf(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            front_min = min(front_min, r)
    return front_min  # returns inf if no valid readings
```

**Safety priority (strict order from spec §31):**
1. `human_lost` → STOP
2. `front_min < 0.6` → STOP
3. `distance <= TARGET_DISTANCE` → stop forward, may turn
4. Otherwise → FOLLOW

**Verify:**
```bash
# Move obstacle box to x=0.3, y=0 (in front of cart) via Gazebo GUI
ros2 topic echo /cmd_vel
# linear.x must be 0.0 while obstacle is within 0.6m
```

**Pass criteria:**
- Cart stops for obstacle < 0.6 m in front.
- NaN/inf/out-of-range LiDAR values silently ignored.
- Zero velocity published on node shutdown.

---

### PHASE 9 — RFID Simulator

**Goal:** Interactive RFID node resolves tags → products, prevents duplicates, prints running total.

**File:** `smartcart_human/smartcart_human/rfid_simulator.py`

**Product table (exact, from spec section K):**
```python
PRODUCTS = {
    "RFID001": ("Milk",    40.0),
    "RFID002": ("Bread",   35.0),
    "RFID003": ("Apple",   20.0),
    "RFID004": ("Biscuit", 30.0),
}
```

**Behavior:**
- Print available tags on startup.
- Read RFID tag ID from `stdin` in background thread.
- Known tag, not in cart → add to cart, publish to `/rfid/item`, print cart + total.
- Known tag, already in cart → print "Already in cart: Milk".
- Unknown tag → print "Unknown RFID tag: RFIDXXX", do not crash.
- Input `clear` or `c` → empty cart, print "Cart cleared. Total: ₹0".

**Publisher:** `/rfid/item` → `std_msgs/msg/String`

**Run command:**
```bash
ros2 run smartcart_human rfid_simulator
```

**Pass criteria (from spec Test U):**

| Input | Expected Output |
|---|---|
| `RFID001` | Milk, ₹40, Total ₹40 |
| `RFID002` | Bread, ₹35, Total ₹75 |
| `RFID003` | Apple, ₹20, Total ₹95 |
| `RFID001` (again) | Already in cart |
| `RFID999` | Unknown RFID tag: RFID999 |
| `clear` | Cart cleared. Total: ₹0 |

---

### PHASE 10 — Integrated Launch + Final Validation

**Goal:** One-command launch starts complete simulation; all acceptance criteria met.

**Final `smartcart.launch.py` node list:**

| Process | Package | Executable/Command |
|---|---|---|
| Gazebo | `ros_gz_sim` | `gz_server` with `smartcart_world.sdf` |
| robot_state_publisher | `robot_state_publisher` | reads URDF from xacro |
| SmartCart spawn | `ros_gz_sim` | `create` action |
| ROS-Gz bridge | `ros_gz_bridge` | `/scan`, `/wheel/odometry` |
| human_controller | `smartcart_human` | `human_controller` |
| follow_controller | `smartcart_human` | `follow_controller` |

**Final demo commands:**
```bash
# Terminal 1 — main simulation
source /opt/ros/jazzy/setup.bash
source ~/smartcart_simulation/smartcart_ws/install/setup.bash
ros2 launch smartcart_gazebo smartcart.launch.py

# Terminal 2 — RFID (interactive)
source /opt/ros/jazzy/setup.bash
source ~/smartcart_simulation/smartcart_ws/install/setup.bash
ros2 run smartcart_human rfid_simulator
```

**Full acceptance test matrix:**

| # | Test | Input | Expected |
|---|---|---|---|
| 1 | Build | `colcon build` | 3 packages finished |
| 2 | Spawn | launch | cart visible in Gazebo |
| 3 | Human | launch | human visible |
| 4 | Drive | `/cmd_vel 0.2` | cart moves forward |
| 5 | LiDAR | obstacle in front | `/scan` ranges < 10 m |
| 6 | Human pose | human moving | `/human/pose` x changes |
| 7 | Follow | human moves away | cart follows |
| 8 | Turn | human lateral offset | cart turns toward human |
| 9 | Target dist | human ~1.2 m | cart stops forward motion |
| 10 | Obstacle | box < 0.6 m | cart stops |
| 11 | Human lost | stop pose publisher | cart stops within ~1 s |
| 12 | RFID known | RFID001 | Milk + ₹40 |
| 13 | RFID duplicate | RFID001 twice | total unchanged |
| 14 | RFID unknown | RFID999 | warning, no crash |
| 15 | RFID total | 001+002+003 | ₹95 |
| 16 | Shutdown | kill controller | zero velocity published |

**Completion definition (from spec section Z):**
All 16 tests pass on a clean machine with a clean `colcon build`.

---

## Key Engineering Rules

- ❌ No Nav2, SLAM, YOLO, OpenCV, ML, lifecycle nodes, action servers.
- ❌ No custom message packages, no frontend/backend/database.
- ✅ Only `install(DIRECTORY ...)` directories that actually exist on disk.
- ✅ Fix one phase fully before proceeding to the next.
- ✅ Named constants only — no magic numbers in controller logic.
- ✅ All stop conditions → `linear.x = 0.0, angular.z = 0.0`.
- ✅ Throttle logs — do not flood terminal.
- ✅ `follow_controller` publishes zero `Twist` on shutdown.
- ✅ Human must have collision geometry (for LiDAR detection).
- ✅ Shelves, walls, floor must have collision geometry.
