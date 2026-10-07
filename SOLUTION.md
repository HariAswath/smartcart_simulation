# SmartCart — Hybrid Human-Following Solution

## 1. Selected Architecture

SmartCart will use a hybrid human-following system designed for crowded indoor environments:

```text
Camera + Person Detection + Multi-Person Tracking + Re-ID
                         +
                  BLE Customer Tag
                         +
                       LiDAR
                         ↓
                 Target Selection
                         ↓
                  Target Position
                         ↓
             Follow + Safety Controller
                         ↓
                      /cmd_vel
                         ↓
                   Motor Controller
```

The three sensing systems have different responsibilities:

| System | Responsibility |
|---|---|
| Camera | Detect and locate people |
| Person tracker | Maintain persistent IDs for visible people |
| Re-ID | Help recognize the customer again after temporary occlusion/lost tracking |
| BLE | Identify/proximity-assist the customer's wearable/tag |
| LiDAR | Detect obstacles and provide movement safety |
| ROS 2 | Connect the perception, safety, and control components |
| Follow controller | Convert target position + safety constraints into `/cmd_vel` |

---

## 2. Core Idea

The cart must solve four separate questions:

```text
BLE
 ↓
WHO is my customer?

Camera
 ↓
WHERE is my customer?

Tracking/Re-ID
 ↓
IS THIS STILL THE SAME PERSON?

LiDAR
 ↓
IS IT SAFE TO MOVE?
```

The final controller answers:

```text
HOW SHOULD THE CART MOVE?
```

This is preferable to simply following the closest detected person.

---

# 3. Why a Hybrid Approach?

A supermarket may contain many people:

```text
Person A    Person B    Person C

       Person D

Person E             Person F
```

The cart must not switch to whichever person happens to be closest.

A pure person detector can identify all people, but detection alone does not identify the customer.

A tracker can maintain temporary IDs, but tracking can fail when the customer is occluded.

Re-identification helps recover the customer's identity after temporary loss.

BLE provides an additional customer-specific identity/proximity signal.

Therefore:

```text
Camera → visual position
Tracker → temporal identity
Re-ID → recovery after visual loss
BLE → customer identity/proximity
LiDAR → obstacle safety
ROS 2 → integration
```

---

# 4. Camera Pipeline

The camera publishes:

```text
/camera/image_raw
```

with:

```text
sensor_msgs/msg/Image
```

Processing:

```text
Camera
  ↓
Image
  ↓
Person Detection
  ↓
Multiple Person Bounding Boxes
  ↓
Multi-Object Tracking
  ↓
Persistent Person IDs
```

Example:

```text
Person A → ID 17
Person B → ID 21
Person C → ID 35
```

The tracker attempts to keep the same ID assigned as people move.

---

# 5. Person Detection

The detector answers:

> Which regions of the image contain people?

Example:

```text
┌──────────────────────────────┐
│                              │
│ [Person A]       [Person B]  │
│                              │
│       [Person C]             │
│                              │
└──────────────────────────────┘
```

A YOLO-class detector is one possible implementation.

However:

```text
Detection ≠ Customer Identification
```

Detection only tells us that people exist.

---

# 6. Multi-Person Tracking

The tracker associates detections between frames.

Example:

```text
Frame 1:
Person A → ID 17
Person B → ID 21
Person C → ID 35

Frame 2:
Person A → ID 17
Person B → ID 21
Person C → ID 35
```

The system can lock onto:

```text
TARGET_ID = 21
```

and continue following that person.

The cart should not switch targets merely because another person becomes closer.

---

# 7. Customer Re-Identification

Tracking can fail during occlusion.

Example:

```text
Customer
   ↓
Other person crosses camera
   ↓
Customer temporarily hidden
```

Before occlusion:

```text
Customer → ID 21
```

After reappearing:

```text
Detected person → ID 47
```

Re-ID helps determine whether ID 47 is visually consistent with the previous customer.

The result should be treated as a confidence score rather than an absolute identity guarantee.

---

# 8. BLE Customer Identification

The customer carries a small BLE device.

Possible form factors:

- BLE key fob
- BLE wristband
- BLE beacon/tag
- small rechargeable BLE device

The cart detects the customer's BLE advertisement.

Example:

```text
BLE Device:
SMARTCART-CUSTOMER-001
```

BLE is primarily used for:

```text
Customer identity
+
Proximity assistance
```

It should NOT be treated as an accurate standalone positioning system.

---

# 9. BLE + Camera Fusion

Suppose the camera sees:

```text
Person A
Person B
Person C
Person D
```

BLE reports that the customer's tag is nearby.

The fusion layer uses the available information to select the most likely visual target.

Conceptually:

```text
Camera
  ↓
Person A
Person B
Person C
Person D

BLE
  ↓
Customer tag detected

        ↓

Target Selection

        ↓

Customer = Person C
```

Then:

```text
TARGET = Person C
```

The camera/tracker supplies the actual target position.

---

# 10. Why Not Drive Directly Using BLE RSSI?

BLE RSSI is affected by:

- people blocking the signal
- shelves
- walls
- device orientation
- antenna position
- reflections
- multipath effects

Therefore this is NOT the intended architecture:

```text
RSSI
 ↓
Exact distance
 ↓
Drive toward signal
```

Instead:

```text
BLE    → identity/proximity clue
Camera → target position
LiDAR  → safety
ROS 2  → integration
Controller → motion
```

---

# 11. LiDAR Safety Layer

LiDAR publishes:

```text
/scan
```

using:

```text
sensor_msgs/msg/LaserScan
```

The LiDAR is responsible for detecting obstacles and enforcing safety constraints.

Example:

```text
Target controller:
MOVE FORWARD

LiDAR:
OBSTACLE TOO CLOSE

Final:
STOP
```

Safety must have priority over following.

---

# 12. Motion Controller

The controller receives:

```text
Target position
+
Target confidence
+
Obstacle information
+
Robot state
```

and produces:

```text
/cmd_vel
```

using:

```text
geometry_msgs/msg/Twist
```

Architecture:

```text
Camera + Tracking + Re-ID
              │
              ▼
        Target Position
              │
              ├──────────────┐
              │              │
              ▼              ▼
       Follow Logic      LiDAR Safety
              │              │
              └──────┬───────┘
                     ▼
              Motion Controller
                     │
                     ▼
                  /cmd_vel
```

---

# 13. Target State Machine

Recommended states:

```text
SEARCHING
    │
    │ customer found
    ▼
TARGET_LOCKED
    │
    ▼
FOLLOWING
    │
    │ target temporarily lost
    ▼
TARGET_LOST
    │
    ├── recovered → FOLLOWING
    │
    └── timeout → STOP
```

The robot must not continue driving blindly when the target has been lost.

---

# 14. Safety Rules

### Rule 1 — Valid target required

The robot should not follow an arbitrary person.

### Rule 2 — LiDAR overrides following

If an obstacle is dangerously close:

```text
/cmd_vel = STOP
```

### Rule 3 — Lost target means slow/stop

Do not continue moving indefinitely without target confirmation.

### Rule 4 — Avoid unnecessary target switching

Once the target is locked, another nearby person should not automatically become the target.

### Rule 5 — Maintain safe following distance

Initial simulation target:

```text
~1.2 m
```

The physical value must be tuned experimentally.

---

# 15. Recommended Hardware

Initial physical prototype:

```text
Raspberry Pi 5
+
RGB Camera
+
2D LiDAR
+
BLE
+
Motor Controller
+
Wheel Encoders
```

Optional later:

```text
AI Accelerator
Depth Camera
UWB
```

Do not add these advanced components until the basic hybrid system works.

---

# 16. ROS 2 Interfaces

Important interfaces:

| Topic | Type | Role |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Camera input |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR safety |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot movement |
| `/wheel_ticks` | Robot-specific | Encoder feedback |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | Odometry |
| `/tf` | `tf2_msgs/msg/TFMessage` | Transform tree |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transforms |
| `/robot_description` | `std_msgs/msg/String` | Robot model |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU feedback |

The target representation can initially remain internal to the relevant nodes. Avoid custom ROS messages unless they become necessary.

---

# 17. Simulation Development Plan

The existing simulation currently uses:

```text
/human/pose
    ↓
follow_controller
    ↓
/cmd_vel
```

The next simulation stage should introduce multiple people.

Target simulation:

```text
Person 1
Person 2
Person 3
Person 4
```

Each person should have:

- a unique namespace
- an independent pose
- an independent controller
- an independent command topic

Example:

```text
/human_1/cmd_vel
/human_1/pose

/human_2/cmd_vel
/human_2/pose

/human_3/cmd_vel
/human_3/pose
```

This allows realistic crowd testing without changing the robot's control interface.

---

# 18. Simulation Crowd Testing

The simulation should test:

### Test A — Static crowd

Several people are present while the target is stationary.

### Test B — Moving crowd

Several people move independently.

### Test C — Crossing

Another person crosses between the cart and target.

### Test D — Occlusion

The target is temporarily hidden.

### Test E — Reappearance

The target returns after temporary visual loss.

### Test F — Nearest-person trap

Another person becomes closer than the target.

The cart should continue following the selected customer.

### Test G — Target loss

The target disappears for longer than the configured timeout.

The cart should safely stop.

### Test H — Obstacle

An obstacle enters the LiDAR safety zone.

The cart should stop even if the target is still visible.

---

# 19. Simulation-to-Hardware Strategy

We should avoid writing a completely different system for Gazebo and the physical robot.

The desired transition is:

```text
SIMULATION

Simulated people
      ↓
Simulated perception
      ↓
Target tracking
      ↓
Follow controller
      ↓
/cmd_vel
      ↓
Gazebo robot
```

then:

```text
REAL ROBOT

Real people
      ↓
Real camera + BLE
      ↓
Target tracking
      ↓
Same follow controller
      ↓
/cmd_vel
      ↓
Real motor controller
```

The controller interface should remain stable.

---

# 20. Final Architecture

```text
                         SMARTCART
                            │
             ┌──────────────┼──────────────┐
             │              │              │
          CAMERA           BLE           LiDAR
             │              │              │
             ▼              ▼              ▼
       Person Detection  Customer       Obstacle
             │           Identity       Detection
             ▼              │              │
       Multi-Person          │              │
          Tracking            │              │
             │               │              │
             ▼               │              │
            Re-ID             │              │
             │               │              │
             └───────────────┼──────────────┘
                             ▼
                      Target Selection
                             │
                             ▼
                      Target Position
                             │
                             ▼
                  Follow + Safety Controller
                             │
                             ▼
                          /cmd_vel
                             │
                             ▼
                       Motor Controller
                             │
                             ▼
                           Wheels
```

## Design Principle

```text
BLE
 ↓
WHO?

Camera
 ↓
WHERE?

Tracking/Re-ID
 ↓
SAME PERSON?

LiDAR
 ↓
SAFE?

Controller
 ↓
HOW TO MOVE?
```

This hybrid architecture is the selected direction for the physical SmartCart prototype.
