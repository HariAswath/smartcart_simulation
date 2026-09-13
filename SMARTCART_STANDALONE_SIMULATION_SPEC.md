# SmartCart Standalone Simulation — AI Engineering Contract

> **IMPORTANT:** This document is the implementation contract for the SmartCart simulation.
> It is written for an AI coding agent such as Gemini CLI.
> The agent must treat the requirements below as authoritative and must not invent additional architecture.

## 0. Agent Operating Rules

### 0.1 Primary objective

Implement a **working standalone ROS 2 Jazzy + Gazebo Harmonic simulation** from an empty project directory.

The implementation must be:

- deterministic
- buildable
- runnable
- testable from the terminal
- modular
- easy to debug
- minimal in dependencies
- compatible with Ubuntu 24.04
- understandable to a human engineer

### 0.2 Do not over-engineer

Do not introduce:

- Nav2
- SLAM
- localization
- machine learning
- YOLO
- OpenCV
- custom message packages
- custom service packages
- lifecycle nodes
- behavior trees
- action servers
- complex planners
- database systems
- web servers
- frontend applications
- backend applications
- external APIs

unless this document explicitly requires them.

### 0.3 Implementation sequence

The agent must implement the project in this order:

```text
PHASE 0  -> environment verification
PHASE 1  -> workspace/packages
PHASE 2  -> robot description
PHASE 3  -> Gazebo world + spawning
PHASE 4  -> differential drive
PHASE 5  -> LiDAR
PHASE 6  -> simulated human + /human/pose
PHASE 7  -> human-follow controller
PHASE 8  -> obstacle emergency stop
PHASE 9  -> RFID simulator
PHASE 10 -> integrated launch + validation
```

After every phase:

1. build
2. run the relevant test
3. inspect errors
4. fix the current phase
5. only then continue

Do not implement all phases at once.

### 0.4 Existing environment verification

Before installing anything, inspect the machine:

```bash
lsb_release -a
source /opt/ros/jazzy/setup.bash
ros2 --version
gz sim --version
which colcon
```

Verify required packages:

```bash
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix ros_gz_interfaces
ros2 pkg prefix robot_state_publisher
ros2 pkg prefix xacro
```

If a package is already installed, do not reinstall it unnecessarily.

### 0.5 File-system discipline

Before creating files:

```bash
pwd
find . -maxdepth 3 -type f | sort
```

Never assume a directory exists.

Every directory referenced by CMake installation rules must actually exist.

Do not create stale references such as:

```cmake
install(DIRECTORY launch meshes config ...)
```

when those directories are absent.

### 0.6 No speculative changes

When a build/runtime error occurs:

- identify the exact failing package/file
- inspect the relevant file
- make the smallest correction
- rebuild
- retest

Do not rewrite unrelated packages.

---

# SmartCart — Standalone Minimum Viable Simulation
## Complete AI-Implementation Technical Specification

**Version:** 2.0  
**Status:** Implementation Source of Truth  
**Target:** ROS 2 Jazzy + Gazebo Harmonic + Ubuntu 24.04  
**Primary goal:** Build a simple, stable, demonstrable SmartCart simulation that an AI coding agent can implement phase-by-phase with minimal ambiguity.

---

# 1. Executive Summary

SmartCart is a smart shopping cart with two independent parts:

1. **Shopping/RFID system**
   - Simulates RFID item detection.
   - Maintains a simple list of scanned products.
   - Calculates a running bill.
   - This is only a ROS simulation mock and does not replace the existing backend/frontend.

2. **Robot system**
   - Simulates a mobile shopping cart in Gazebo.
   - Uses differential drive.
   - Has a simulated LiDAR.
   - Has an optional simulated camera.
   - Receives the simulated human's position.
   - Follows the human.
   - Stops when an obstacle is too close.
   - Stops when the human is lost.

The design intentionally avoids advanced robotics technologies.

---

# 2. Core Design Principle

## Build the smallest reliable system first.

The final demonstration should prove:

```text
RFID item detection
        +
running shopping bill
        +
mobile SmartCart
        +
human following
        +
LiDAR safety stop
        =
working SmartCart prototype
```

The simulation is **not** intended to be a production autonomous robot.

---

# 3. Explicit Non-Goals

Version 2.0 must NOT implement:

- Nav2
- SLAM
- mapping
- localization
- AMCL
- autonomous navigation
- path planning
- A*
- Dijkstra
- RRT
- complex obstacle avoidance
- machine learning
- neural networks
- YOLO
- OpenCV-based human detection
- camera-based human recognition
- LiDAR-based human detection
- Kalman filtering
- sensor fusion
- multi-person tracking
- multi-robot systems
- realistic RFID electromagnetic simulation
- custom Gazebo RFID physics
- database connection from ROS
- React frontend modification
- FastAPI backend modification
- payment processing
- inventory management

If a feature is not necessary for the demonstration, do not add it.

---

# 4. Project Boundary

This project is a **simulation-only ROS 2 project**.

There is no frontend, backend, database, web application, payment system, or external application dependency.

Everything required for the demonstration must live inside this simulation project.

The complete system consists of:

```text
ROS 2
  +
Gazebo Harmonic
  +
SmartCart robot
  +
Simulated human
  +
LiDAR
  +
Optional camera
  +
RFID simulator
  +
Human-following controller
```

The project must be independently cloneable, buildable, and runnable.


# 5. Elaborated Simulation Description

## 5.1 What We Are Simulating

The SmartCart simulation is a **virtual prototype of an intelligent shopping cart** operating inside a supermarket environment.

The purpose of the simulation is to demonstrate two important capabilities of the SmartCart concept:

1. **Assisted shopping**
   - The cart can identify simulated products using RFID tags.
   - The system maintains the products currently placed in the cart.
   - A running shopping total is displayed.

2. **Autonomous cart movement**
   - The cart can move through a simulated supermarket.
   - The cart can identify the location of a designated shopper.
   - The cart follows that shopper automatically.
   - A LiDAR sensor continuously checks the area in front of the cart.
   - The cart stops when an obstacle becomes dangerously close.
   - The cart also stops when the shopper's position is no longer available.

The simulation therefore demonstrates the main idea of a **SmartCart that assists the shopper while also providing autonomous following behavior**.

---

## 5.2 Why the Simulation Is Being Built

Building and testing the complete SmartCart system directly on physical hardware would require:

- a mobile robot platform
- motors and motor drivers
- wheel encoders
- LiDAR hardware
- a camera
- an RFID reader
- RFID tags
- batteries
- motor controllers
- mechanical construction
- safety equipment
- a physical testing environment

Developing the behavior directly on hardware would also make debugging slower and potentially unsafe.

Gazebo provides a controlled virtual environment where the robot can be tested before physical hardware is introduced.

The simulation allows the development team to verify:

```text
Robot model
     ↓
Robot movement
     ↓
Sensors
     ↓
Human position
     ↓
Following algorithm
     ↓
Obstacle safety
     ↓
RFID behavior
```

without requiring physical hardware.

---

## 5.3 What the Final Demonstration Represents

The final simulation represents a shopper walking through a supermarket while a SmartCart follows them.

A simplified demonstration is:

```text
                    SUPERMARKET

       +-----------------------------------+
       |                                   |
       |    SHELF              SHELF       |
       |                                   |
       |              HUMAN                |
       |                O                  |
       |                |                  |
       |                |                  |
       |             SmartCart             |
       |                C                  |
       |                                   |
       |    SHELF              SHELF       |
       |                                   |
       +-----------------------------------+
```

The human acts as the **target**.

The SmartCart continuously determines:

```text
Where is the human?
How far away is the human?
Is the human to the left or right?
Is there an obstacle in front?
Should the cart move or stop?
```

The controller then produces a velocity command.

---

## 5.4 Simulation Components

The simulation contains the following major components:

```text
+---------------------------------------------------+
|                 Gazebo Supermarket                |
|                                                   |
|       Human                         Shelves       |
|         |                              |          |
|         |                              |          |
|         v                              v          |
|   Human Position                  Obstacles       |
|         |                              |          |
|         +---------------+--------------+          |
|                         |                         |
|                         v                         |
|                 SmartCart Robot                   |
|                         |                         |
|             +-----------+-----------+              |
|             |                       |              |
|           LiDAR                  Camera            |
|             |                       |              |
|             v                       |              |
|       Safety Detection              |              |
|             |                       |              |
|             +-----------+-----------+              |
|                         |                         |
|                         v                         |
|                 Follow Controller                 |
|                         |                         |
|                         v                         |
|                      /cmd_vel                     |
|                         |                         |
|                         v                         |
|                  Differential Drive               |
+---------------------------------------------------+

                     RFID System
                         |
                         v
                  RFID Simulator
                         |
                         v
                    Product List
                         |
                         v
                    Total Price
```

---

## 5.5 SmartCart Robot

The SmartCart is represented by a simple mobile robot model.

The physical model contains:

```text
Rectangular cart body
        +
Left drive wheel
        +
Right drive wheel
        +
Passive caster
        +
LiDAR
        +
Optional camera
```

The cart uses **differential drive**.

This means the robot moves by independently controlling:

```text
Left wheel velocity
Right wheel velocity
```

For example:

```text
Both wheels forward
        ↓
Robot moves forward

Left wheel faster
Right wheel slower
        ↓
Robot turns right

Right wheel faster
Left wheel slower
        ↓
Robot turns left

Both wheels stopped
        ↓
Robot stops
```

The application does not directly control individual wheels.

Instead, the follow controller publishes:

```text
/cmd_vel
```

and Gazebo's differential-drive system converts that command into wheel motion.

---

## 5.6 Simulated Shopper

The shopper is represented by a simple human-shaped Gazebo model.

The model does not need realistic graphics.

It only needs:

- visible geometry
- collision geometry
- a position in the Gazebo world

The human is controlled programmatically.

For the demonstration, the shopper walks along a predefined path.

Example:

```text
Start
  |
  v
x = 2 m
  |
  |  walk forward
  v
x = 6 m
  |
  |  turn around
  v
x = 2 m
  |
  +---- repeat
```

This deterministic movement makes the simulation easy to reproduce and debug.

---

## 5.7 Human Detection Abstraction

A real SmartCart would need to detect a human using sensors.

That could eventually look like:

```text
Camera
   ↓
Person Detection
   ↓
Person Tracking
   ↓
Target Position
   ↓
Follow Controller
```

However, implementing real computer vision is outside the scope of this simulation.

Instead, Gazebo already knows the exact position of the simulated human.

Therefore:

```text
Gazebo Human Position
        ↓
Human Controller
        ↓
 /human/pose
        ↓
Follow Controller
```

The `/human/pose` topic acts as the **human-detection interface**.

This provides a clean separation:

```text
                 Human Detection Interface
                           |
                    /human/pose
                           |
             +-------------+-------------+
             |                           |
     Current simulation            Future real system
       implementation              camera detector
             |                           |
      Gazebo position              vision algorithm
```

A future real detector can replace the simulated source without redesigning the following controller.

---

## 5.8 How Human Following Works

The controller receives the human's relative position.

The simplified coordinate convention is:

```text
              +X
               ^
               |
               |
        Human  |
               |
               |
             Robot
               |
        +Y = left
```

More precisely:

```text
human_x = forward distance
human_y = lateral offset
```

The distance is:

```text
distance = sqrt(human_x² + human_y²)
```

The desired following distance is approximately:

```text
1.2 metres
```

If:

```text
distance > 1.2 m
```

the cart moves toward the human.

If:

```text
distance <= 1.2 m
```

the cart stops moving forward.

The angular velocity is based on the lateral error:

```text
angular_z = K_ANGLE × human_y
```

Therefore:

```text
Human on left
      ↓
positive angular command
      ↓
Cart turns left
```

and:

```text
Human on right
      ↓
negative angular command
      ↓
Cart turns right
```

---

## 5.9 LiDAR Safety System

The LiDAR is not being used to perform mapping.

Its only purpose is:

> **Prevent the cart from driving into an obstacle.**

The controller examines a forward sector:

```text
             +30°
               \
                \
                 |
                 |
             CART
                 |
                /
               /
             -30°
```

If the closest valid object is less than:

```text
0.6 m
```

the controller immediately commands:

```text
linear.x  = 0
angular.z = 0
```

The cart therefore stops.

This is intentionally an **emergency stop**, not a full obstacle-avoidance system.

---

## 5.10 Why We Do Not Use Complex Obstacle Avoidance

A real autonomous robot may need to:

```text
detect obstacle
        ↓
create obstacle map
        ↓
plan alternate path
        ↓
avoid obstacle
        ↓
rejoin target
```

That requires significantly more software.

For this project, the important demonstration is simply:

```text
Obstacle detected
        ↓
Cart stops
```

This is safer, easier to implement, and easier to explain during a project demonstration.

---

## 5.11 RFID Simulation

The RFID portion represents the shopping-assistance side of SmartCart.

A real RFID system would contain:

```text
RFID reader
     ↓
RFID tag
     ↓
RFID identifier
     ↓
Product database
     ↓
Shopping cart
     ↓
Total bill
```

The simulation replaces the physical RFID reader with a Python program.

Therefore:

```text
Simulated RFID input
        ↓
RFID ID
        ↓
Product lookup
        ↓
Product added to cart
        ↓
Total recalculated
```

No electromagnetic RFID physics is required.

---

## 5.12 Example RFID Products

The simulation uses a small predefined product list:

```text
RFID001 -> Milk     -> ₹40
RFID002 -> Bread    -> ₹35
RFID003 -> Apple    -> ₹20
RFID004 -> Biscuit  -> ₹30
```

When the user enters:

```text
RFID001
```

the system produces:

```text
RFID detected: RFID001
Product: Milk
Price: ₹40

Current Total: ₹40
```

After:

```text
RFID002
```

the result becomes:

```text
Milk       ₹40
Bread      ₹35
----------------
Total      ₹75
```

---

## 5.13 RFID Duplicate Protection

A product should not accidentally be added multiple times because the same simulated RFID tag was entered repeatedly.

Therefore:

```text
RFID001
RFID001
RFID001
```

should result in:

```text
Milk
₹40
```

not:

```text
Milk
Milk
Milk
₹120
```

unless quantity handling is explicitly added later.

This keeps the demonstration predictable.

---

## 5.14 RFID and Robot Independence

The RFID system and robot-following system are separate subsystems.

```text
                 SmartCart Simulation
                         |
             +-----------+-----------+
             |                       |
             v                       v
        RFID System              Robot System
             |                       |
       Product data              Human pose
             |                       |
       Running total              LiDAR
                                     |
                                     v
                              Follow controller
```

This separation is intentional.

The robot should continue following the human even if the RFID simulator is not running.

Likewise, RFID scanning should work even if Gazebo is stopped.

---

## 5.15 Optional Camera

The camera is included as a sensor demonstration rather than as a required part of the control loop.

It can provide:

```text
/camera/image_raw
```

The camera allows future development toward:

```text
Camera
  ↓
Human detection
  ↓
Human tracking
  ↓
Follow controller
```

but Version 2.0 does not depend on this pipeline.

---

## 5.16 ROS 2 Communication

The simulation uses ROS 2 topics to connect its components.

Main robot topics:

```text
/cmd_vel
/scan
```

Human topic:

```text
/human/pose
```

RFID topic:

```text
/rfid/item
```

Optional camera:

```text
/camera/image_raw
```

Conceptually:

```text
/human/pose
      |
      v
Follow Controller
      |
      +-------> /cmd_vel
      |
/scan-+
```

and:

```text
RFID Simulator
      |
      v
/rfid/item
      |
      v
Shopping Cart List
      |
      v
Total
```

---

## 5.17 Why ROS 2 Is Used

ROS 2 provides a modular communication framework.

Each part of the system can be developed independently.

For example:

```text
Human Controller
       |
       | /human/pose
       v
Follow Controller
       |
       | /cmd_vel
       v
Gazebo Robot
```

This makes the project easier to:

- test
- debug
- replace components
- expand later
- migrate toward physical hardware

---

## 5.18 Why Gazebo Is Used

Gazebo provides:

- physics simulation
- collision detection
- robot movement
- sensor simulation
- visualization
- repeatable test scenarios

The robot can therefore be tested without physical hardware.

---

## 5.19 Simulation-to-Hardware Concept

The long-term architecture is:

```text
             SIMULATION
                  |
     +------------+------------+
     |                         |
 Gazebo Robot              Simulated Human
     |
 ROS 2
```

Later:

```text
             REAL ROBOT
                  |
     +------------+------------+
     |            |            |
 Motors         LiDAR        Camera
                  |
                ROS 2
```

The controller should remain mostly independent of whether its input originates from simulation or physical sensors.

This is one of the main engineering benefits of the project.

---

## 5.20 Complete Data Flow

### Robot following

```text
Gazebo Human
     |
     v
human_controller.py
     |
     | /human/pose
     v
follow_controller.py
     |
     +------ /scan
     |
     v
Safety + Following Logic
     |
     v
geometry_msgs/Twist
     |
     | /cmd_vel
     v
Gazebo Diff Drive
     |
     v
SmartCart Motion
```

### RFID

```text
User / automatic RFID event
             |
             v
     rfid_simulator.py
             |
             | RFID ID
             v
      Product Lookup
             |
             v
       Shopping List
             |
             v
       Total Price
```

### Complete system

```text
                       SMARTCART
                           |
             +-------------+-------------+
             |                           |
             v                           v
        RFID Subsystem             Robot Subsystem
             |                           |
       RFID Simulator                Gazebo
             |                           |
       Product Lookup           +-------+-------+
             |                  |               |
       Shopping List          Human           Cart
             |                  |               |
       Running Total       /human/pose       LiDAR
                                    \          /
                                     \        /
                                      v      v
                                  Follow Controller
                                          |
                                          v
                                       /cmd_vel
                                          |
                                          v
                                   Differential Drive
```

---

## 5.21 What Makes This a Prototype

This project is a **functional robotics prototype**, not a production product.

It demonstrates the core behaviors with controlled assumptions.

The simulation intentionally replaces difficult real-world components with deterministic abstractions:

| Real system | Simulation |
|---|---|
| Real shopper | Gazebo human |
| Human vision detection | `/human/pose` |
| Real LiDAR | Gazebo LiDAR |
| Real motor controller | Gazebo differential drive |
| Real RFID reader | RFID simulator |
| Real RFID tags | RFID IDs |
| Real supermarket | Simple Gazebo world |

This allows the main system behavior to be tested without implementing every physical detail.

---

## 5.22 Expected Final Behavior

When everything is running:

1. Gazebo opens the supermarket.
2. SmartCart appears.
3. Human appears ahead of the cart.
4. Human begins moving.
5. Human position is published.
6. Follow controller calculates relative position.
7. Cart moves toward the human.
8. Cart turns when the human moves sideways.
9. Cart maintains approximately 1.2 m distance.
10. LiDAR continuously checks the front.
11. Cart stops if an obstacle is closer than 0.6 m.
12. Cart stops if human information is lost.
13. RFID simulator can independently scan products.
14. Product names and prices are displayed.
15. Running total is updated.

The result is a complete, understandable SmartCart prototype demonstration.

---

# 6. Technology Stack

Required:

```text
OS:          Ubuntu 24.04
ROS:         ROS 2 Jazzy
Simulator:   Gazebo Harmonic
Language:    Python 3
Build:       colcon
Robot model: URDF/Xacro
Bridge:      ros_gz_bridge
```

ROS packages:

```text
rclpy
geometry_msgs
sensor_msgs
std_msgs
nav_msgs
ros_gz_sim
ros_gz_bridge
ros_gz_interfaces
robot_state_publisher
xacro
```

Avoid adding packages unless they are genuinely necessary.

---

# 7. Workspace

Recommended location:

```text
~/smartcart_simulation/smartcart_ws
```

Structure:

```text
smartcart_ws/
├── src/
│   ├── smartcart_description/
│   ├── smartcart_gazebo/
│   └── smartcart_human/
├── build/
├── install/
└── log/
```

`build`, `install`, and `log` are generated directories and should not be committed.

---

# 8. ROS Package Architecture

Only three packages are required.

```text
smartcart_description
    |
    +-- Robot model

smartcart_gazebo
    |
    +-- World
    +-- Human model
    +-- Launch file
    +-- ROS/Gazebo bridges

smartcart_human
    |
    +-- Human movement
    +-- Human pose publishing
    +-- Follow controller
    +-- RFID simulation
```

---

# 9. Recommended Package Files

## 8.1 smartcart_description

```text
smartcart_description/
├── CMakeLists.txt
├── package.xml
└── urdf/
    └── smartcart.urdf.xacro
```

Only install directories that actually exist.

Do NOT put nonexistent directories such as `launch`, `meshes`, or `config` into `install(DIRECTORY ...)`.

---

## 8.2 smartcart_gazebo

```text
smartcart_gazebo/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── smartcart.launch.py
├── worlds/
│   └── smartcart_world.sdf
└── models/
    └── human/
        ├── model.config
        └── model.sdf
```

---

## 8.3 smartcart_human

```text
smartcart_human/
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

# 10. Robot Physical Model

Use primitive geometry.

## Base

```text
Length: 0.8 m
Width:  0.6 m
Height: 0.3 m
Mass:   approximately 8 kg
```

Base visual:

```text
box
size = 0.8 0.6 0.3
```

Place its center approximately:

```text
z = 0.15
```

---

# 11. Wheels

Use two drive wheels.

```text
Radius:       0.15 m
Width:        approximately 0.08 m
Separation:   0.7 m
```

Frames:

```text
left_wheel
right_wheel
```

Joints:

```text
left_wheel_joint
right_wheel_joint
```

Use continuous/revolute wheel joints appropriate for Gazebo.

---

# 12. Caster

Add one passive caster near the front or rear.

The caster only exists to stabilize the cart.

Do not implement steering.

---

# 13. Robot Frames

Keep TF minimal:

```text
odom
  |
  v
base_link
  |
  +---- lidar_link
  |
  +---- camera_link
```

The robot should publish:

```text
odom -> base_link
```

through the Gazebo differential drive system.

Static transforms:

```text
base_link -> lidar_link
base_link -> camera_link
```

can be generated from the robot model.

---

# 14. Differential Drive

Use Gazebo's standard differential drive system.

Required parameters:

```text
left_joint       = left_wheel_joint
right_joint      = right_wheel_joint
wheel_separation = 0.7
wheel_radius     = 0.15
cmd topic        = /cmd_vel
odom topic       = /wheel/odometry
```

ROS controller output:

```text
geometry_msgs/msg/Twist
```

Only use:

```text
linear.x
angular.z
```

All other velocity fields should be zero.

---

# 15. Velocity Limits

Recommended:

```text
maximum linear velocity  = 0.5 m/s
maximum angular velocity = 1.0 rad/s
```

Normal following speed should generally be:

```text
0.1–0.4 m/s
```

Do not allow the controller to command excessive speeds.

---

# 16. LiDAR

Use a simple 2D LiDAR.

Recommended:

```text
Type:       lidar
Samples:    360
Horizontal FOV: 360 degrees
Update rate: 10 Hz
Minimum:    0.12 m
Maximum:    10.0 m
Resolution: 0.01 m
```

ROS topic:

```text
/scan
```

Message:

```text
sensor_msgs/msg/LaserScan
```

Frame:

```text
lidar_link
```

The LiDAR is used only for obstacle safety.

---

# 17. LiDAR Compatibility Rule

For Gazebo Harmonic, verify the actual sensor configuration against the installed Gazebo version.

If GPU LiDAR produces only `inf` values while a visible collision object is directly ahead, switch to the CPU LiDAR sensor type.

Do not assume that a ROS bridge problem is the cause.

Debug in this order:

```text
Gazebo sensor topic
        ↓
Gazebo values
        ↓
ROS bridge
        ↓
ROS /scan
```

If Gazebo itself publishes `inf`, fix the Gazebo sensor/model before debugging ROS.

---

# 18. Camera

The camera is optional.

If implemented:

```text
Topic: /camera/image_raw
Resolution: 320 x 240
Rate: 10–15 Hz
```

The camera is for:

- visualization
- future development
- demonstrating that the robot has a vision sensor

It is NOT required for human detection.

The simulation must work if the camera is disabled.

---

# 19. Human Model

Use simple primitive geometry.

Example:

```text
Torso: box
Head:  sphere
Arms:  boxes
Legs:  boxes
```

Approximate mass:

```text
70 kg
```

The human must have collision geometry so LiDAR can detect it.

Do not use external human meshes.

---

# 20. Human Movement

Create:

```text
human_controller.py
```

Responsibilities:

1. connect to Gazebo's entity pose service
2. move the simulated human
3. publish the human's current position

Recommended path:

```text
x = 2 m -> 6 m -> 2 m
y = 0 m
```

Recommended speed:

```text
0.1–0.3 m/s
```

The movement should be deterministic.

---

# 21. Human Pose Topic

Publish:

```text
/human/pose
```

Message:

```text
geometry_msgs/msg/Pose
```

Minimum useful information:

```text
pose.position.x
pose.position.y
```

The controller interprets this as the target position.

---

# 22. Why We Use Direct Human Pose

In a real robot:

```text
Camera
  ↓
Person detector
  ↓
Person tracker
  ↓
Human position
```

For this simulation:

```text
Gazebo human
  ↓
Known simulated position
  ↓
/human/pose
```

This is deliberate.

It allows the project to demonstrate the following behavior without spending most of the project on computer vision.

The interface can later be replaced by a real detector.

---

# 23. Human Position Convention

For easiest controller implementation, transform the human position into the robot's local frame.

Use:

```text
+X = robot forward
+Y = robot left
```

Therefore:

```text
human_x > 0
```

means the human is in front.

```text
human_y > 0
```

means the human is to the left.

```text
human_y < 0
```

means the human is to the right.

If the first implementation keeps everything on the same world axis, it may initially use:

```text
distance_x = human_world_x - robot_world_x
distance_y = human_world_y - robot_world_y
```

However, for a rotating robot, the preferred implementation is to calculate the relative position in the robot frame.

---

# 24. Human Distance

Calculate:

```text
distance = sqrt(human_x² + human_y²)
```

Target following distance:

```text
TARGET_DISTANCE = 1.2 m
```

Recommended acceptable band:

```text
1.0–1.5 m
```

---

# 25. Human Following Controller

Create:

```text
follow_controller.py
```

Subscriptions:

```text
/human/pose
/scan
```

Publisher:

```text
/cmd_vel
```

The controller runs at approximately:

```text
10 Hz
```

A faster loop is unnecessary for the demonstration.

---

# 26. Controller Algorithm

Use proportional control.

Forward error:

```text
distance_error = distance - TARGET_DISTANCE
```

Forward velocity:

```text
linear_x = K_DISTANCE * distance_error
```

Starting value:

```text
K_DISTANCE = 0.5
```

Angular velocity:

```text
angular_z = K_ANGLE * human_y
```

Starting value:

```text
K_ANGLE = 1.0
```

Clamp:

```text
linear_x  ∈ [0.0, 0.5]
angular_z ∈ [-1.0, 1.0]
```

Do not command negative linear velocity in the first version.

If the human is too close, stop instead of reversing.

---

# 27. Simplest Stable Follow Logic

Recommended implementation:

```python
if human_lost:
    stop()

elif obstacle_too_close:
    stop()

elif distance <= TARGET_DISTANCE:
    linear_x = 0.0
    angular_z = K_ANGLE * human_y

else:
    linear_x = K_DISTANCE * (distance - TARGET_DISTANCE)
    angular_z = K_ANGLE * human_y
```

This is enough.

---

# 28. Human Lost Timeout

Track the time of the latest `/human/pose`.

Recommended:

```text
HUMAN_TIMEOUT = 1.0 second
```

If:

```text
current_time - last_human_time > 1.0
```

then:

```text
linear.x = 0
angular.z = 0
```

The robot must stop.

Do not make it search for the person.

---

# 29. Obstacle Safety

Read `/scan`.

Only inspect the front sector.

Recommended:

```text
-30 degrees to +30 degrees
```

Convert these angles into indices using:

```text
index = (angle - angle_min) / angle_increment
```

Ignore:

```text
NaN
inf
ranges outside range_min/range_max
```

Find the minimum valid range in the front sector.

---

# 30. Emergency Stop Threshold

Use:

```text
OBSTACLE_STOP_DISTANCE = 0.6 m
```

If:

```text
front_min_distance < 0.6
```

then:

```text
linear.x = 0
angular.z = 0
```

Do not attempt sophisticated avoidance.

---

# 31. Safety Priority

The controller must evaluate conditions in this order:

```text
1. Human lost
2. Obstacle too close
3. Human too close
4. Human following
```

Both human loss and obstacle detection result in a complete stop.

A safer explicit ordering is:

```text
if human_lost:
    STOP

elif obstacle_detected:
    STOP

elif human_too_close:
    STOP

else:
    FOLLOW
```

---

# 32. Controller State Machine

Use three states:

```text
STOP
FOLLOW
TURN
```

### STOP

```text
linear.x = 0
angular.z = 0
```

### FOLLOW

```text
linear.x > 0
angular.z based on human_y
```

### TURN

Used when the human is significantly to the side.

```text
linear.x = 0
angular.z = turn toward human
```

The TURN state is optional if proportional control already works.

Do not create more states unless needed.

---

# 33. RFID Simulation

RFID is part of the demonstration, but it must remain simple.

There is no need to simulate electromagnetic fields.

Create:

```text
rfid_simulator.py
```

The node simulates an RFID reader.

---

# 34. RFID Product Data

Use a small hard-coded product database.

Example:

```python
PRODUCTS = {
    "RFID001": {
        "name": "Milk",
        "price": 40.0
    },
    "RFID002": {
        "name": "Bread",
        "price": 35.0
    },
    "RFID003": {
        "name": "Apple",
        "price": 20.0
    },
    "RFID004": {
        "name": "Biscuit",
        "price": 30.0
    }
}
```

Use Indian Rupees.

Prices are demonstration values only.

---

# 35. RFID Topics

Publish:

```text
/rfid/item
```

Message:

```text
std_msgs/msg/String
```

The string should contain the RFID ID.

Example:

```text
RFID001
```

Optionally publish a human-readable event:

```text
/rfid/status
```

using:

```text
std_msgs/msg/String
```

Example:

```text
Detected: Milk | RFID001 | ₹40
```

No custom ROS message is required.

---

# 36. RFID Simulation Method

Use one of the following simple methods.

## Preferred method: keyboard/manual trigger

The node accepts RFID IDs from the terminal.

Example:

```text
Enter RFID:
RFID001

Detected:
Milk
Price:
₹40
```

This is easiest to demonstrate.

---

## Alternative: timed automatic scan

The node can automatically generate an item every few seconds.

Example:

```text
RFID001
RFID002
RFID003
```

This is useful for unattended demos.

---

# 37. RFID Duplicate Handling

The simulator should maintain a cart list.

Example:

```text
Milk       ₹40
Bread      ₹35
Apple      ₹20
```

Total:

```text
₹95
```

For version 1, duplicate RFID scans should be ignored unless the implementation explicitly wants quantities.

Simplest rule:

```text
If RFID ID already exists:
    ignore duplicate
```

This prevents accidental repeated billing.

---

# 38. RFID Billing Logic

Maintain:

```text
items = {}
```

When an unknown RFID is received:

```text
look up product
add product
recalculate total
print cart
```

Example output:

```text
================================
       SMARTCART RFID
================================
Detected Item: Milk
RFID: RFID001
Price: ₹40

Current Cart:
1. Milk       ₹40

TOTAL: ₹40
================================
```

After another scan:

```text
Current Cart:
1. Milk       ₹40
2. Bread      ₹35

TOTAL: ₹75
```

---

# 39. RFID Unknown Tag

If:

```text
RFID999
```

is received:

```text
Unknown RFID tag: RFID999
```

Do not crash.

---

# 40. RFID Reset

The node should support a simple reset mechanism.

Simplest implementation:

```text
type:
clear
```

or a keyboard command:

```text
c
```

Expected:

```text
Cart cleared.
Total: ₹0
```

This is optional but useful for repeated demonstrations.

---

# 41. RFID Simulation Boundary

The RFID simulator does NOT connect to:

```text
FastAPI
database
React
payment
inventory
```

unless a future integration phase is explicitly requested.

Version 1 only demonstrates the concept.

---

# 42. Optional Status Monitor

A small status node or status output can make the demo clearer.

The controller can print:

```text
========================================
          SMARTCART STATUS
========================================
Human:       DETECTED
Distance:    1.37 m
Obstacle:    CLEAR
State:       FOLLOW
Linear:      0.08 m/s
Angular:     0.12 rad/s
========================================
```

Do not create a GUI.

Terminal output is enough.

---

# 43. Launch Architecture

One launch file should start:

```text
Gazebo
robot_state_publisher
SmartCart spawn
LiDAR bridge
camera bridge (if enabled)
human pose bridge if required
```

The Python nodes may be:

- launched separately during development
- or included in the final launch file

For the final demonstration, preferably provide one launch file that starts everything.

---

# 44. Final One-Command Demo

Target:

```bash
ros2 launch smartcart_gazebo smartcart.launch.py
```

This should start the main simulation.

If RFID is intentionally kept interactive in a separate terminal, document:

```bash
ros2 run smartcart_human rfid_simulator
```

The final project should make the two-terminal setup clear.

---

# 45. Recommended Final Demo Terminals

## Terminal 1

```bash
source /opt/ros/jazzy/setup.bash
source ~/smartcart_simulation/smartcart_ws/install/setup.bash
ros2 launch smartcart_gazebo smartcart.launch.py
```

## Terminal 2

```bash
source /opt/ros/jazzy/setup.bash
source ~/smartcart_simulation/smartcart_ws/install/setup.bash
ros2 run smartcart_human rfid_simulator
```

This is acceptable.

---

# 46. Build Rules

Always build from:

```bash
cd ~/smartcart_simulation/smartcart_ws
```

Then:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
```

After success:

```bash
source install/setup.bash
```

For a clean rebuild:

```bash
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build
```

Only use a clean rebuild when necessary.

---

# 47. Package Dependency Rules

## smartcart_description

Needs:

```text
ament_cmake
```

plus whatever is required for installation.

## smartcart_gazebo

Needs:

```text
ament_cmake
ros_gz_sim
ros_gz_bridge
robot_state_publisher
xacro
```

## smartcart_human

Needs:

```text
ament_python
rclpy
geometry_msgs
sensor_msgs
std_msgs
ros_gz_interfaces
```

Do not add unused dependencies.

---

# 48. Python Coding Rules

All Python nodes should:

- use `rclpy`
- subclass `Node` when appropriate
- create publishers/subscribers explicitly
- use timers instead of busy loops where possible
- log important state changes
- stop the robot when shutting down
- avoid global mutable state unless necessary
- keep functions short
- use named constants for thresholds
- avoid hard-coded magic numbers inside controller logic

Example constants:

```python
TARGET_DISTANCE = 1.2
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 1.0
OBSTACLE_STOP_DISTANCE = 0.6
HUMAN_TIMEOUT = 1.0
K_DISTANCE = 0.5
K_ANGLE = 1.0
```

---

# 49. Shutdown Safety

When `follow_controller.py` exits:

```text
publish zero Twist
```

so the robot stops.

At minimum:

```python
stop_msg = Twist()
publisher.publish(stop_msg)
```

before shutdown.

---

# 50. Error Handling

The implementation must gracefully handle:

## No human pose

```text
STOP
```

## No LiDAR message yet

```text
STOP
```

## Invalid LiDAR ranges

Ignore invalid values.

If no valid front range exists:

```text
STOP
```

## Unknown RFID

Print warning.

Do not crash.

## Gazebo service unavailable

Wait and log:

```text
Waiting for Gazebo set_pose service...
```

---

# 51. Logging

Important logs:

```text
Human controller started
Gazebo service connected
Human movement started
Follow controller started
RFID simulator started
Obstacle detected
Emergency stop
Human lost
Human detected
RFID item detected
Cart total updated
```

Do not print hundreds of messages per second.

Use throttled/status logs where appropriate.

---

# 52. Gazebo World

Use:

```text
12 m x 8 m
```

Create:

- ground
- outer walls
- 4–6 shelf boxes
- human
- SmartCart

Shelves can simply be:

```text
box collision
box visual
```

No textures are required.

---

# 53. World Layout

Recommended:

```text
+---------------------------------------+
|                                       |
|    [SHELF]             [SHELF]       |
|                                       |
|                                       |
|                HUMAN                  |
|                  O                    |
|                                       |
|                CART                   |
|                 C                     |
|                                       |
|    [SHELF]             [SHELF]       |
|                                       |
+---------------------------------------+
```

Keep a clear central aisle.

The human should initially move through this clear aisle.

---

# 54. Simulation Initial Conditions

Recommended:

```text
Robot:
x = 0
y = 0
yaw = 0

Human:
x = 2
y = 0
yaw = 0
```

Robot faces:

```text
+X direction
```

Human is initially:

```text
2 m ahead
```

This makes the first demo deterministic.

---

# 55. Simulation Timing

Use normal real-time simulation:

```text
real_time_factor = 1
```

Physics:

```text
max_step_size = 0.001
real_time_update_rate = 1000
```

If laptop performance is poor, do not immediately increase complexity.

Reduce sensor rates/resolution first.

---

# 56. Camera Performance Rule

If camera causes low simulation performance:

First reduce:

```text
640x480 -> 320x240
```

Then:

```text
30 Hz -> 10 Hz
```

The camera is optional.

Do not sacrifice robot stability for camera quality.

---

# 57. Testing Strategy

Test one feature at a time.

---

## Test 1 — Build

```bash
colcon build
```

Expected:

```text
Summary: 3 packages finished
```

---

## Test 2 — Gazebo

Launch world.

Expected:

```text
Gazebo starts
```

---

## Test 3 — Robot Spawn

Expected:

```text
SmartCart visible
```

---

## Test 4 — Manual Movement

Publish:

```text
/cmd_vel
```

Verify:

```text
forward
backward
rotation
stop
```

---

## Test 5 — LiDAR

Run:

```bash
ros2 topic echo /scan
```

Place a box in front.

Expected:

```text
range becomes smaller
```

---

## Test 6 — Human

Verify:

```bash
ros2 topic echo /human/pose
```

Expected:

```text
x changes over time
```

---

## Test 7 — Following

Expected:

```text
Human moves
Cart follows
```

---

## Test 8 — Obstacle

Place obstacle ahead.

Expected:

```text
Cart stops
```

---

## Test 9 — Human Lost

Stop `/human/pose`.

Expected:

```text
Cart stops within approximately 1 second
```

---

## Test 10 — RFID

Run:

```bash
ros2 run smartcart_human rfid_simulator
```

Enter:

```text
RFID001
RFID002
```

Expected:

```text
Milk
Bread
Total ₹75
```

---

# 58. Acceptance Criteria

Version 2.0 is complete when:

### Robot

- [ ] SmartCart appears in Gazebo.
- [ ] Differential drive works.
- [ ] Robot moves using `/cmd_vel`.
- [ ] Robot publishes odometry.
- [ ] TF is valid.
- [ ] LiDAR publishes `/scan`.

### Human

- [ ] Human appears.
- [ ] Human has collision geometry.
- [ ] Human moves deterministically.
- [ ] `/human/pose` publishes.

### Following

- [ ] Cart follows human.
- [ ] Cart turns toward human.
- [ ] Cart maintains approximately 1.2 m distance.
- [ ] Cart stops when human information is lost.

### Safety

- [ ] Cart stops for an obstacle below 0.6 m.
- [ ] Invalid LiDAR values do not crash controller.
- [ ] Robot stops on shutdown.

### RFID

- [ ] RFID simulator starts.
- [ ] Known tags resolve to products.
- [ ] Duplicate tags do not double-count.
- [ ] Total is calculated.
- [ ] Unknown tags are handled safely.
- [ ] Cart can be reset.

### Demonstration

- [ ] Supermarket-like environment exists.
- [ ] Simulation is understandable visually.
- [ ] No advanced feature is required for basic operation.

---

# 59. Expected Demo

The evaluator should be able to see:

```text
             SMARTCART
                 |
       +---------+---------+
       |                   |
   RFID System        Robot System
       |                   |
   Scan item           Human moves
       |                   |
   Product name        Cart follows
       |                   |
   Running total       LiDAR detects
                           |
                        Obstacle
                           |
                           ↓
                          STOP
```

This clearly demonstrates the project's two main capabilities.

---

# 60. Example RFID Demo

Start:

```text
SMARTCART RFID SIMULATOR

Available Tags:
RFID001 = Milk = ₹40
RFID002 = Bread = ₹35
RFID003 = Apple = ₹20
RFID004 = Biscuit = ₹30

Enter RFID:
```

Input:

```text
RFID001
```

Output:

```text
Item detected: Milk
Price: ₹40

TOTAL: ₹40
```

Input:

```text
RFID002
```

Output:

```text
Item detected: Bread
Price: ₹35

TOTAL: ₹75
```

---

# 61. Recommended Demo Script

### Step 1

Start Gazebo.

Say:

> "This is our simulated SmartCart environment."

### Step 2

Show the cart.

Say:

> "The cart uses differential drive and a simulated LiDAR."

### Step 3

Start human movement.

Say:

> "The simulated person position is provided to the following controller."

### Step 4

Human moves.

Say:

> "The cart calculates the relative position and follows the person."

### Step 5

Place obstacle.

Say:

> "LiDAR continuously checks the front safety region."

Cart stops.

### Step 6

Run RFID simulator.

Scan:

```text
RFID001
RFID002
RFID003
```

Say:

> "The RFID subsystem identifies products and maintains the running shopping total."

### Step 7

Explain architecture.

```text
RFID -> shopping system
ROS 2 -> robot behavior
Gazebo -> simulation
```

---

# 62. AI Agent Implementation Strategy

An AI coding agent must NOT implement the entire repository in one step.

Use milestones.

---

## Milestone 1

Create packages and verify build.

Deliverables:

```text
smartcart_description
smartcart_gazebo
smartcart_human
```

Stop and test.

---

## Milestone 2

Implement robot URDF/Xacro.

Test:

```text
robot appears
```

Stop.

---

## Milestone 3

Implement differential drive.

Test:

```text
/cmd_vel
```

Stop.

---

## Milestone 4

Implement LiDAR.

Test:

```text
/scan
```

Stop.

---

## Milestone 5

Implement human model.

Test:

```text
human appears
```

Stop.

---

## Milestone 6

Implement human movement and:

```text
/human/pose
```

Stop.

---

## Milestone 7

Implement follow controller.

Test:

```text
human movement -> robot movement
```

Stop.

---

## Milestone 8

Implement obstacle emergency stop.

Test:

```text
obstacle -> STOP
```

Stop.

---

## Milestone 9

Implement RFID simulator.

Test:

```text
RFID -> product -> total
```

Stop.

---

## Milestone 10

Integrate and clean the final launch.

Test complete demonstration.

---

# 63. AI Debugging Protocol

Whenever an implementation fails:

1. Read the complete error.
2. Identify the package causing it.
3. Check whether the failure is build-time or runtime.
4. Test the smallest affected component.
5. Fix only that component.
6. Rebuild.
7. Re-run the test.
8. Only then continue.

Never respond to a runtime error by rewriting the entire project.

---

# 64. AI Must Verify Files Before Editing

Before modifying a file, the AI should inspect:

```bash
pwd
find . -maxdepth 3 -type f | sort
```

Do not assume a directory exists.

For example, if CMake contains:

```cmake
install(DIRECTORY launch ...)
```

but:

```text
launch/
```

does not exist, fix the package structure before building.

---

# 65. AI Must Not Invent ROS APIs

Use installed package APIs.

For ROS/Gazebo interfaces:

```bash
ros2 interface show <interface>
```

For example:

```bash
ros2 interface show ros_gz_interfaces/srv/SetEntityPose
```

before writing code that calls it.

This avoids incorrect field assumptions.

---

# 66. AI Must Verify Topics

Before writing subscribers:

```bash
ros2 topic list
ros2 topic type /scan
ros2 topic type /human/pose
```

Before writing publishers, verify expected message types.

---

# 67. AI Must Verify Gazebo Topics Separately

Use:

```bash
gz topic -l
```

If necessary:

```bash
gz topic -e -t /scan
```

Do not assume that a ROS topic means Gazebo itself is working.

---

# 68. Performance Rules

The simulation should prioritize:

```text
stable physics
>
working sensors
>
correct controller
>
visual quality
```

If performance is poor:

1. reduce camera resolution
2. reduce camera rate
3. reduce LiDAR samples
4. reduce world complexity
5. keep controller rate around 10 Hz

Do not add hardware-heavy features.

---

# 69. Troubleshooting Guide

## Problem: robot does not appear

Check:

```bash
ros2 topic echo /robot_description
```

and launch output.

Verify:

```text
xacro file exists
robot_state_publisher starts
ros_gz_sim create runs
```

---

## Problem: robot appears but does not move

Check:

```bash
ros2 topic echo /cmd_vel
```

If commands exist, inspect:

```text
wheel joint names
wheel radius
wheel separation
diff drive plugin
```

---

## Problem: `/scan` exists but all ranges are `inf`

Test Gazebo directly:

```bash
gz topic -e -t /scan
```

Place a collision object directly in front.

If Gazebo also shows `inf`, fix the sensor/model.

Possible causes:

- incorrect sensor type
- incorrect sensor configuration
- missing Sensors system plugin
- missing collision geometry
- sensor orientation/location
- object outside field/range

---

## Problem: human cannot be detected by LiDAR

Verify human has collision elements.

A visual model without collision geometry is not enough.

---

## Problem: robot oscillates

Reduce:

```text
K_ANGLE
```

or reduce maximum angular velocity.

Start with:

```text
K_ANGLE = 0.5
```

---

## Problem: robot gets too close

Increase:

```text
TARGET_DISTANCE
```

For example:

```text
1.2 -> 1.4
```

or reduce:

```text
K_DISTANCE
```

---

## Problem: robot moves too aggressively

Reduce:

```text
K_DISTANCE
```

or:

```text
MAX_LINEAR_SPEED
```

---

## Problem: robot stops unexpectedly

Check:

```text
/scan
/human/pose
```

The likely causes are:

- false obstacle detection
- human timeout
- invalid LiDAR values
- incorrect coordinate calculation

---

# 70. Coordinate Debugging

If the cart turns the wrong direction:

Print:

```text
human_x
human_y
distance
angular_z
```

Expected:

```text
human_y > 0 -> turn left
human_y < 0 -> turn right
```

If this is reversed, fix the coordinate transformation rather than changing random controller signs.

---

# 71. Minimal ROS Graph

Expected:

```text
human_controller
       |
       +---- /human/pose
                    |
                    v
             follow_controller
                    |
/scan --------------+
                    |
                    v
                 /cmd_vel
                    |
                    v
              Gazebo Robot
```

RFID is independent:

```text
rfid_simulator
      |
      v
/rfid/item
```

---

# 72. Final File Responsibility Table

| File | Responsibility |
|---|---|
| `smartcart.urdf.xacro` | Robot geometry, joints, sensors, diff drive |
| `smartcart_world.sdf` | Ground, walls, shelves, human |
| `human/model.sdf` | Human geometry and collisions |
| `smartcart.launch.py` | Start simulation and bridges |
| `human_controller.py` | Move human and publish pose |
| `follow_controller.py` | Follow human and safety stop |
| `rfid_simulator.py` | RFID products and running bill |

Keep each file focused.

---

# 73. Final Architecture

```text
                           SMARTCART
                               |
             +-----------------+-----------------+
             |                                   |
       SHOPPING SYSTEM                       ROBOT SYSTEM
             |                                   |
       RFID Simulator                         Gazebo
             |                                   |
       /rfid/item                         +------+------+
             |                             |             |
      Product Database                   Human        SmartCart
             |                             |             |
       Running Total                  Human Pose      LiDAR
                                           |             |
                                           +------+------+
                                                  |
                                           Follow Controller
                                                  |
                                               /cmd_vel
                                                  |
                                           Differential Drive
```

---

# 74. Version 2.0 Definition

The project should be considered successful if an evaluator can see:

```text
1. A shopping cart robot in Gazebo.
2. A simulated person.
3. The robot following that person.
4. The robot turning toward that person.
5. The robot stopping for an obstacle.
6. RFID products being scanned.
7. A running shopping total.
```

No additional complexity is required.

---

# 75. Final Engineering Rule

The project should always prefer:

```text
deterministic
+
observable
+
testable
+
simple
```

over:

```text
realistic
+
complex
+
hard to debug
```

The objective is not to simulate every part of a real SmartCart.

The objective is to create a technically understandable prototype that clearly demonstrates the core SmartCart concept.

**END OF SPECIFICATION**



# Supermarket / Shopping Mart Environment — Detailed Specification

## Environment Objective

The Gazebo world must visually and physically represent a **small supermarket/shopping mart** rather than an empty test room.

The environment exists to demonstrate that SmartCart can operate in a shopping environment while following the shopper and reacting to obstacles.

The environment must remain simple enough for reliable simulation performance.

---

## Environment Layout

Use a rectangular supermarket approximately:

```text
Width  = 12 m
Length = 8 m
```

Coordinate convention:

```text
                 +Y
                  ^
                  |
        -X <------+------> +X
                  |
                  v
                 -Y
```

The SmartCart starts near:

```text
x = 0
y = 0
yaw = 0
```

The main shopping aisle should extend approximately along the X axis.

---

## Required Supermarket Components

The world must contain:

```text
1. Floor
2. Four outer walls
3. Entrance area
4. Checkout/counter area
5. Multiple product shelves
6. Central walking aisle
7. Side aisles
8. Product/display boxes
9. Simulated shopper
10. SmartCart
```

The objects should use simple primitive Gazebo geometry.

No downloaded supermarket assets are required.

---

## Top-Down Layout

Use a layout similar to:

```text
+------------------------------------------------+
|                                                |
| ENTRANCE                         CHECKOUT      |
|   ↓                                [====]      |
|                                                |
|  +--------+       CENTRAL AISLE      +--------+|
|  | SHELF  |                          | SHELF  ||
|  |        |                          |        ||
|  +--------+                          +--------+|
|                                                |
|                                                |
|                 HUMAN                         |
|                   O                            |
|                                                |
|                 SMARTCART                     |
|                    C                           |
|                                                |
|  +--------+                          +--------+|
|  | SHELF  |                          | SHELF  ||
|  |        |                          |        ||
|  +--------+                          +--------+|
|                                                |
|  +--------+                          +--------+|
|  | SHELF  |                          | SHELF  ||
|  +--------+                          +--------+|
|                                                |
+------------------------------------------------+
```

The exact visual arrangement may differ slightly, but the central aisle must remain sufficiently wide for the SmartCart and human to move safely.

---

## Floor

Create one large ground plane.

Recommended:

```text
size = approximately 12 m x 8 m
```

The floor must have collision geometry.

Use a simple material/color suitable for a supermarket floor.

Do not use complex textures.

---

## Outer Walls

Create four walls.

Approximate:

```text
wall thickness = 0.1–0.2 m
wall height    = 2.5–3.0 m
```

Walls must have collision geometry.

The walls prevent the human and SmartCart from leaving the supermarket.

---

## Entrance

Create a clearly recognizable entrance on one side.

The entrance can simply be:

```text
wide opening
```

between two wall sections.

No automatic doors are required.

The entrance is primarily a visual element.

---

## Checkout Counter

Add a simple checkout counter near one side of the supermarket.

Use:

```text
box
```

geometry.

Example:

```text
length = 1.5 m
width  = 0.6 m
height = 1.0 m
```

The checkout counter must have collision geometry.

It does not need to perform any checkout logic.

The RFID simulator remains responsible for demonstrating the shopping/billing concept.

---

## Shelves

Create approximately **6 shelves**.

Each shelf can be constructed from:

```text
main rectangular body
+
optional smaller product boxes
```

The shelf must have collision geometry.

Recommended approximate shelf size:

```text
length = 2.0 m
width  = 0.6 m
height = 1.5 m
```

The exact dimensions can be adjusted to avoid collisions with the cart.

---

## Shelf Placement

Recommended arrangement:

```text
Left side:

[Shelf 1]
[Shelf 2]
[Shelf 3]

Right side:

[Shelf 4]
[Shelf 5]
[Shelf 6]
```

Leave a clear aisle between them.

For example:

```text
+------------------------------------------------+
|                                                |
|  [S1]                     [S4]                |
|                                                |
|  [S2]       CLEAR AISLE      [S5]             |
|                                                |
|  [S3]                     [S6]                |
|                                                |
+------------------------------------------------+
```

The human should initially walk through the central clear area.

---

## Side Aisles

The shelves should create simple side aisles.

The robot does not need to autonomously navigate these aisles in Version 2.0.

They are primarily present to make the environment look like a shopping mart and provide realistic obstacles.

---

## Product Displays

Add small colored/simple boxes on or near shelves to represent products.

Example:

```text
[ Milk ]
[ Bread ]
[ Apple ]
[ Biscuit ]
```

These are visual objects only.

They do not need to be individually connected to RFID.

The RFID simulator handles RFID events independently.

---

## Product/RFID Relationship

The environment may visually label product areas, but **RFID detection is not based on physical proximity to these Gazebo objects**.

Version 2.0 uses:

```text
RFID ID
   ↓
Product dictionary
   ↓
Shopping cart
   ↓
Total
```

rather than:

```text
Robot physically approaches shelf
   ↓
RFID electromagnetic simulation
   ↓
Tag detected
```

This keeps the simulation deterministic.

---

## SmartCart Starting Position

Start the cart in the central aisle.

Recommended:

```text
x = 0.0
y = 0.0
z = 0.3
yaw = 0.0
```

The cart should face toward the positive X direction.

---

## Human Starting Position

Start the human approximately:

```text
x = 2.0
y = 0.0
z = 0.0
yaw = 0.0
```

This places the shopper in front of the cart.

The human then moves through the central aisle.

---

## Human Shopping Behavior

For the basic demonstration, the human can move:

```text
+X direction
```

and then return:

```text
-X direction
```

A more visually interesting but still simple deterministic path may be:

```text
Start
  |
  v
Central aisle
  |
  v
Move forward
  |
  v
Move slightly left
  |
  v
Move forward
  |
  v
Return toward center
  |
  v
Move backward
  |
  v
Repeat
```

The path must not require autonomous path planning.

The human controller directly sets the simulated pose.

---

## Obstacle Demonstration Area

Reserve one section of the supermarket where an obstacle can be placed directly in front of the SmartCart.

Example:

```text
Human
  O
  |
  |
 Cart ---> [BOX]
```

The box represents:

- another shopping cart
- product box
- temporary obstacle
- supermarket object

The exact interpretation is not important.

When the box is closer than:

```text
0.6 m
```

the cart must stop.

---

## Environment Collision Requirements

Every object that should affect robot safety must have collision geometry.

Required collision objects:

```text
floor
walls
shelves
checkout counter
obstacle demonstration box
human
```

Visual geometry alone is insufficient for LiDAR/collision behavior.

---

## Environment Visual Requirements

The world should be recognizable as a supermarket at a glance.

Use simple visual differentiation:

```text
Floor       -> simple floor material
Walls       -> simple wall material
Shelves     -> contrasting shelf material
Products    -> small colored boxes
Checkout    -> counter-like box
Human       -> human model
SmartCart   -> cart body/wheels
```

Do not spend implementation time on photorealistic rendering.

The environment's purpose is:

```text
recognizable
+
functional
+
fast
+
stable
```

---

## Environment Performance Requirements

The supermarket must not contain:

- high-polygon meshes
- complex textures
- unnecessary lighting
- hundreds of objects
- complex physics
- dynamic shelves
- unnecessary moving objects

Use primitive geometry wherever possible.

The world should run smoothly on a normal development laptop.

---

## Lighting

Use simple Gazebo lighting.

Recommended:

```text
one directional light
```

or another minimal light configuration supported by Gazebo Harmonic.

Do not add complicated lighting systems.

---

## Static vs Dynamic Objects

The following should normally be static:

```text
floor
walls
shelves
checkout counter
product displays
```

The following are dynamic:

```text
SmartCart
human
```

The obstacle demonstration box may be static or dynamic.

For the simplest implementation, make the obstacle static.

---

## Supermarket World Success Criteria

The supermarket environment is complete when:

```text
[ ] Gazebo starts without world errors.
[ ] Floor is visible.
[ ] Outer walls exist.
[ ] Entrance is recognizable.
[ ] Checkout counter exists.
[ ] Six shelves exist.
[ ] Central aisle is clear.
[ ] Product/display boxes are visible.
[ ] Human can move through the aisle.
[ ] SmartCart can move through the aisle.
[ ] Shelves have collision geometry.
[ ] Human has collision geometry.
[ ] An obstacle can be placed in front of the cart.
[ ] LiDAR can detect the obstacle.
[ ] Cart stops when obstacle is too close.
[ ] World runs without unnecessary performance problems.
```

---

## Final Supermarket Demonstration

The preferred visual demonstration is:

```text
                SMARTCART SUPERMARKET

        +------------------------------------+
        | ENTRANCE             CHECKOUT      |
        |   ↓                    [====]      |
        |                                    |
        | [SHELF]              [SHELF]       |
        |                                    |
        |             HUMAN                  |
        |               O                    |
        |               ↓                    |
        |             CART                   |
        |              C                     |
        |                                    |
        | [SHELF]              [SHELF]       |
        |                                    |
        | [SHELF]              [SHELF]       |
        |                                    |
        +------------------------------------+
```

The final demo should communicate immediately that:

> This is an intelligent shopping cart operating inside a supermarket, following its shopper, using LiDAR for safety, and supporting RFID-based shopping.


# AI Implementation Contract

## A. Exact Source Tree

The final source tree must be:

```text
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

No additional package is required.

---

# B. ROS Interface Contract

The following interfaces are fixed.

| Name | Direction | Type | Required |
|---|---|---|---|
| `/cmd_vel` | controller → robot | `geometry_msgs/msg/Twist` | YES |
| `/scan` | LiDAR → controller | `sensor_msgs/msg/LaserScan` | YES |
| `/human/pose` | human node → controller | `geometry_msgs/msg/Pose` | YES |
| `/rfid/item` | RFID simulator → RFID consumer | `std_msgs/msg/String` | YES |
| `/camera/image_raw` | camera → ROS | `sensor_msgs/msg/Image` | OPTIONAL |
| `/wheel/odometry` | Gazebo → ROS/Gazebo interface | odometry | YES for robot verification |

Do not rename these topics.

Do not create custom message definitions.

---

# C. Node Contract

## C.1 `human_controller`

Executable:

```text
human_controller
```

Node name:

```text
human_controller
```

Responsibilities:

1. Wait for Gazebo entity-pose functionality if required.
2. Maintain deterministic human position.
3. Move the simulated human between x=2.0 m and x=6.0 m.
4. Keep y approximately 0.
5. Publish `/human/pose`.
6. Stop cleanly on shutdown.

Suggested update period:

```text
0.1 s
```

Suggested human speed:

```text
0.1–0.3 m/s
```

The node must not publish robot velocity.

---

## C.2 `follow_controller`

Executable:

```text
follow_controller
```

Node name:

```text
follow_controller
```

Subscriptions:

```text
/human/pose
/scan
```

Publisher:

```text
/cmd_vel
```

Control loop:

```text
10 Hz
```

Responsibilities:

1. validate human pose freshness
2. calculate human relative position
3. calculate human distance
4. inspect LiDAR front sector
5. apply emergency stop
6. apply following controller
7. clamp velocity
8. publish `Twist`
9. publish zero velocity on shutdown

The controller must not directly manipulate Gazebo model state.

---

## C.3 `rfid_simulator`

Executable:

```text
rfid_simulator
```

Node name:

```text
rfid_simulator
```

Publisher:

```text
/rfid/item
```

Type:

```text
std_msgs/msg/String
```

Responsibilities:

1. provide known RFID IDs
2. resolve IDs to products
3. reject unknown IDs safely
4. prevent accidental duplicate additions
5. maintain a running total
6. print the cart contents
7. support cart reset
8. remain independent from Gazebo

---

# D. Robot Description Contract

## D.1 Links

Required links:

```text
base_link
left_wheel_link
right_wheel_link
caster_link
lidar_link
camera_link
```

Required joints:

```text
left_wheel_joint
right_wheel_joint
caster_joint
lidar_joint
camera_joint
```

Use fixed joints for sensors.

---

## D.2 Base

```text
size:
  x = 0.8 m
  y = 0.6 m
  z = 0.3 m

mass = 8.0 kg
```

Base center:

```text
z = 0.15 m
```

---

## D.3 Wheels

```text
radius = 0.15 m
separation = 0.7 m
```

Wheel collision and visual geometry must agree.

Wheel axes must be oriented correctly for the differential-drive plugin.

---

## D.4 Inertia

Do not use zero inertia for links with non-zero mass.

For simple primitive geometry, calculate physically reasonable inertial values.

For a box:

```text
Ixx = m/12 * (y² + z²)
Iyy = m/12 * (x² + z²)
Izz = m/12 * (x² + y²)
```

Use a small but valid inertia for wheel/caster links.

The root-link KDL warning about inertia is not by itself a runtime failure.

---

# E. Differential Drive Contract

Use Gazebo's standard DiffDrive system.

Required values:

```text
left_joint = left_wheel_joint
right_joint = right_wheel_joint
wheel_separation = 0.7
wheel_radius = 0.15
topic = /cmd_vel
odom_topic = /wheel/odometry
frame_id = odom
child_frame_id = base_link
```

Recommended:

```text
odom_publisher_frequency = 30
```

The plugin must publish odometry and the `odom -> base_link` transform where supported by the selected configuration.

---

# F. LiDAR Contract

Required sensor:

```text
type = lidar
```

Recommended configuration:

```text
horizontal samples = 360
horizontal min angle = -pi
horizontal max angle = +pi
vertical samples = 1
range min = 0.12
range max = 10.0
update rate = 10 Hz
```

ROS topic:

```text
/scan
```

Frame:

```text
lidar_link
```

The Gazebo Sensors system must be loaded.

---

# G. LiDAR Processing Algorithm

Do not use all 360 degrees for emergency stopping.

Use:

```text
-30 degrees <= angle <= +30 degrees
```

For every selected range:

```text
if math.isnan(range):
    ignore

if math.isinf(range):
    ignore

if range < range_min:
    ignore

if range > range_max:
    ignore
```

Compute:

```text
front_min = minimum(valid_ranges)
```

If there are no valid ranges:

```text
SAFE BEHAVIOR = STOP
```

If:

```text
front_min < 0.6
```

then:

```text
linear.x = 0
angular.z = 0
```

---

# H. Human Pose Contract

The human node publishes:

```text
geometry_msgs/msg/Pose
```

The controller must use only:

```text
position.x
position.y
```

Orientation is not required for Version 2.0.

Preferred representation:

```text
human_x = target forward displacement
human_y = target lateral displacement
```

If world coordinates are used internally, transform them into the robot frame before control.

For a planar robot with robot pose `(rx, ry, yaw)` and human world position `(hx, hy)`:

```text
dx = hx - rx
dy = hy - ry

human_x =  cos(yaw) * dx + sin(yaw) * dy
human_y = -sin(yaw) * dx + cos(yaw) * dy
```

This transformation is mandatory if the robot can rotate.

---

# I. Follow Controller Mathematical Contract

Constants:

```python
TARGET_DISTANCE = 1.2
K_DISTANCE = 0.5
K_ANGLE = 1.0
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 1.0
OBSTACLE_STOP_DISTANCE = 0.6
HUMAN_TIMEOUT = 1.0
```

Distance:

```text
d = sqrt(human_x² + human_y²)
```

Distance error:

```text
e_d = d - TARGET_DISTANCE
```

Forward velocity:

```text
v = K_DISTANCE * e_d
```

Clamp:

```text
v = max(0, min(v, MAX_LINEAR_SPEED))
```

Angular velocity:

```text
w = K_ANGLE * human_y
```

Clamp:

```text
w = max(-MAX_ANGULAR_SPEED,
        min(w, MAX_ANGULAR_SPEED))
```

Behavior:

```text
human lost       -> STOP
obstacle close   -> STOP
human <= target  -> no forward motion, may rotate
otherwise        -> FOLLOW
```

The first implementation must not reverse the cart.

---

# J. Human Lost Contract

Store:

```text
last_human_time
```

At every controller cycle:

```text
age = current_time - last_human_time
```

If:

```text
age > 1.0 second
```

publish:

```text
linear.x = 0
angular.z = 0
```

At startup, before receiving the first valid human pose:

```text
STOP
```

---

# K. RFID Data Contract

Use exactly this initial product table unless there is a strong implementation reason to change it:

```python
PRODUCTS = {
    "RFID001": ("Milk", 40.0),
    "RFID002": ("Bread", 35.0),
    "RFID003": ("Apple", 20.0),
    "RFID004": ("Biscuit", 30.0),
}
```

Input:

```text
RFID001
```

Output must clearly identify:

```text
RFID ID
product name
price
current total
```

Duplicate IDs should be ignored.

Unknown IDs should produce a warning and must not crash the node.

---

# L. Launch Contract

`smartcart.launch.py` must start the minimum simulation components:

1. Gazebo Harmonic
2. robot_state_publisher
3. SmartCart spawn
4. LiDAR bridge
5. camera bridge if camera is enabled
6. any required Gazebo/ROS service bridge
7. human simulation node
8. follow controller

RFID may remain a separate terminal process because it is an interactive demonstration.

Final target:

```bash
ros2 launch smartcart_gazebo smartcart.launch.py
```

and separately:

```bash
ros2 run smartcart_human rfid_simulator
```

---

# M. Gazebo World Contract

World dimensions:

```text
12 m x 8 m
```

Required:

```text
ground plane
four walls
4–6 shelf boxes
one human
one SmartCart
```

The central aisle must remain open.

Recommended initial poses:

```text
SmartCart:
x = 0
y = 0
z = 0.3
yaw = 0

Human:
x = 2
y = 0
z = 0
yaw = 0
```

The human must have collision geometry.

The shelves must have collision geometry.

---

# N. ROS-Gazebo Bridge Contract

Bridge:

```text
/scan
```

from:

```text
gz.msgs.LaserScan
```

to:

```text
sensor_msgs/msg/LaserScan
```

Camera, if enabled:

```text
/camera/image_raw
/camera/camera_info
```

Use the installed `ros_gz_bridge` syntax and verify the actual bridge output with:

```bash
ros2 topic type /scan
ros2 topic type /camera/image_raw
```

Do not assume bridge success merely because the bridge process started.

---

# O. Verification Commands

## O.1 Package discovery

```bash
ros2 pkg list | grep smartcart
```

Expected:

```text
smartcart_description
smartcart_gazebo
smartcart_human
```

## O.2 Nodes

```bash
ros2 node list
```

Expected minimum runtime nodes include:

```text
robot_state_publisher
human_controller
follow_controller
```

plus Gazebo-related nodes.

## O.3 Topics

```bash
ros2 topic list
```

Must include:

```text
/cmd_vel
/scan
/human/pose
/rfid/item
```

## O.4 Types

```bash
ros2 topic type /cmd_vel
ros2 topic type /scan
ros2 topic type /human/pose
ros2 topic type /rfid/item
```

Expected:

```text
geometry_msgs/msg/Twist
sensor_msgs/msg/LaserScan
geometry_msgs/msg/Pose
std_msgs/msg/String
```

---

# P. Manual Robot Test

With simulation running, publish a short forward command.

Use a bounded command and then explicitly stop.

Example:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Verify the robot responds.

Do not leave a continuous velocity command running during unrelated tests.

---

# Q. Human Test

Run:

```bash
ros2 topic echo /human/pose
```

The x coordinate must change over time.

The human must remain inside the world.

---

# R. Follow Test

Expected sequence:

```text
Human initially ~2 m ahead
        ↓
Controller receives pose
        ↓
Distance > 1.2 m
        ↓
Cart moves forward
        ↓
Distance approaches 1.2 m
        ↓
Cart stops forward motion
```

When human changes lateral position:

```text
human_y > 0 -> angular_z > 0
human_y < 0 -> angular_z < 0
```

assuming the ROS/Gazebo convention and controller coordinate convention are configured as specified.

---

# S. Safety Test

Put a collision object directly in front of the robot.

Expected:

```text
front_min < 0.6 m
        ↓
STOP
```

Verify:

```bash
ros2 topic echo /cmd_vel
```

The controller should publish zero linear velocity while the emergency condition exists.

---

# T. Human-Loss Test

Stop the human pose publisher.

Expected:

```text
within <= approximately 1 second
        ↓
linear.x = 0
angular.z = 0
```

The robot must not continue indefinitely.

---

# U. RFID Test

Run:

```bash
ros2 run smartcart_human rfid_simulator
```

Enter:

```text
RFID001
RFID002
RFID003
```

Expected total:

```text
₹95
```

Then repeat:

```text
RFID001
```

The total must remain:

```text
₹95
```

Enter:

```text
RFID999
```

Expected:

```text
Unknown RFID tag
```

No crash.

---

# V. Acceptance Test Matrix

| Test | Input | Expected |
|---|---|---|
| Build | `colcon build` | 3 packages build |
| Spawn | launch | cart visible |
| Human | launch | human visible |
| Drive | `/cmd_vel` | cart moves |
| LiDAR | obstacle | `/scan` changes |
| Human pose | human movement | `/human/pose` changes |
| Follow | human moves away | cart follows |
| Turn | human lateral offset | cart turns |
| Target distance | human near 1.2m | cart stops forward motion |
| Obstacle | obstacle <0.6m | cart stops |
| Human lost | pose stops | cart stops |
| RFID known | RFID001 | Milk + ₹40 |
| RFID duplicate | RFID001 twice | only one Milk |
| RFID unknown | RFID999 | warning, no crash |
| RFID total | 001+002+003 | ₹95 |
| Shutdown | terminate controller | zero velocity |

---

# W. Required Engineering Quality

The AI implementation must satisfy:

### Readability

Use:

```text
constants
type hints where useful
clear function names
small functions
comments for non-obvious ROS/Gazebo details
```

### Maintainability

Do not put all logic in one 500-line node.

Suggested controller functions:

```text
get_human_age()
calculate_relative_position()
calculate_distance()
get_front_min_range()
is_obstacle_detected()
calculate_follow_velocity()
publish_stop()
```

### Safety

All stop conditions must produce:

```text
linear.x = 0.0
angular.z = 0.0
```

### Determinism

The same launch should produce approximately the same initial configuration and human path.

---

# X. Final Runtime Architecture

```text
                       GAZEBO HARMONIC
                              |
          +-------------------+-------------------+
          |                                       |
       SmartCart                               Human
          |                                       |
          |                                  human_controller
          |                                       |
          |                                  /human/pose
          |                                       |
       LiDAR                                       |
          |                                        |
        /scan                                      |
          |                                        |
          +-------------------+--------------------+
                              |
                              v
                     follow_controller
                              |
                    +---------+---------+
                    |                   |
              Safety Logic        Follow Logic
                    |                   |
                    +---------+---------+
                              |
                              v
                           /cmd_vel
                              |
                              v
                     Gazebo DiffDrive
                              |
                              v
                         SmartCart


                     RFID SUBSYSTEM
                              |
                    rfid_simulator.py
                              |
                         /rfid/item
                              |
                       Product Lookup
                              |
                       Shopping Cart
                              |
                         Total Price
```

---

# Y. Final AI Completion Checklist

The AI agent must not declare completion until all are true:

```text
[ ] Workspace builds from a clean state.
[ ] All 3 packages build.
[ ] Gazebo launches.
[ ] SmartCart spawns.
[ ] Human spawns.
[ ] Human moves.
[ ] /human/pose works.
[ ] /scan works.
[ ] /cmd_vel works.
[ ] Differential drive works.
[ ] Follow controller works.
[ ] Turning works.
[ ] Target-distance behavior works.
[ ] Human-loss stop works.
[ ] Obstacle stop works.
[ ] RFID simulator works.
[ ] Known RFID tags resolve correctly.
[ ] Duplicate RFID tags are handled.
[ ] Unknown RFID tags are handled.
[ ] RFID total calculation works.
[ ] No frontend/backend/database exists or is required.
[ ] Final launch command is documented.
[ ] No unnecessary packages were introduced.
[ ] No future-enhancement scope was added.
```

---

# Z. Completion Definition

The implementation is complete only when a clean machine with the documented prerequisites can execute:

```bash
cd ~/smartcart_simulation/smartcart_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch smartcart_gazebo smartcart.launch.py
```

and observe:

```text
Gazebo
  ↓
SmartCart
  ↓
Human
  ↓
Human moves
  ↓
Cart follows
  ↓
Obstacle appears
  ↓
Cart stops
```

and, in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/smartcart_simulation/smartcart_ws/install/setup.bash
ros2 run smartcart_human rfid_simulator
```

with:

```text
RFID001 -> Milk -> ₹40
RFID002 -> Bread -> ₹35
RFID003 -> Apple -> ₹20
TOTAL = ₹95
```

The implementation must be considered successful only after these behaviors have been verified, not merely after the code compiles.
