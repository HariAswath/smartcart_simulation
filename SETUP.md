  # Environment & Packages Overview

- ROS 2 Distro: Jazzy Jalisco
- Gazebo Simulator: Gazebo Harmonic (gz-sim v8.15.0) via ros_gz
- Packages:
    - smartcart_description: URDF/Xacro robot model with 4WD skid-steer drive plugin, 2D LiDAR (/scan), and RGB camera (/camera/image_raw).
    - smartcart_gazebo: Supermarket SDF world (smartcart_world.sdf), 3D human model, and master launch file smartcart.launch.py.
    - smartcart_human: Human position controller, 360° skid-steer following controller, interactive keyboard teleoperation, and RFID item simulator.

---

  ## How to Run the Simulation

  Open your terminal in /home/sai/Projects/smartcart_simulation/smartcart_ws:

  #### 1. Build and Source the Workspace

    cd /home/sai/Projects/smartcart_simulation/smartcart_ws
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    source install/setup.bash

  #### 2. Launch Gazebo Simulation & Controllers

    ros2 launch smartcart_gazebo smartcart.launch.py

  (Optional: Run in headless mode without GUI by passing headless:=true)

  This launches:

  • Gazebo Sim Harmonic loaded with the supermarket environment and human model
  • robot_state_publisher and robot spawner for SmartCart
  • ros_gz_bridge for /cmd_vel, /wheel/odometry, /scan, /camera/image_raw, and /clock
  • human_controller (maintains human pose in Gazebo and publishes /human/pose)
  • follow_controller (proportional skid-steer tracking with obstacle safety)
  ──────
  ### Interactive Controls (In New Terminals)

  #### Keyboard Teleop for Human Movement

  To walk the human around the supermarket (the cart will track and follow):

    source /opt/ros/jazzy/setup.bash
    source /home/sai/Projects/smartcart_simulation/smartcart_ws/install/setup.bash
    ros2 run smartcart_human human_teleop

  • W / S / A / D / Arrow Keys: Walk forward, backward, and steer
  • Q / E: Strafe left / right
  • Space: Instant stop
  • + / -: Adjust walking speed

  #### Interactive RFID Item Scanner

  To simulate adding items into the shopping cart:

    source /opt/ros/jazzy/setup.bash
    source /home/sai/Projects/smartcart_simulation/smartcart_ws/install/setup.bash
    ros2 run smartcart_human rfid_simulator

  • Enter product codes: RFID001 (Milk), RFID002 (Bread), RFID003 (Apple), RFID004 (Biscuit)
  • list / total to display current cart total bill
  • clear to empty cart