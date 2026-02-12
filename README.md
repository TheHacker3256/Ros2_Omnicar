**OmniCar: 4 Wheel Drive Mecanum Car**
=====================================

**Table of Contents**
-------------------

1. [Description](#description)
2. [Hardware Requirements](#hardware-requirements)
3. [Installation](#installation)
4. [Hardware Interface](#hardware-interface)
5. [ROS2 Control](#ros2-control)

**Description**
---------------

The OmniCar is a 4 wheel drive mecanum car designed for autonomous navigation. It is built around a Raspberry Pi running Ubuntu 22.04, ROS2 Humble, and ROS2 Control. The car uses an Arduino Uno as a hardware interface, connected to a Duinotech Motor Driver Shield and 4 DC motors.

**Hardware Requirements**
------------------------

* Raspberry Pi (running Ubuntu 22.04)
* Arduino Uno
* Duinotech Motor Driver Shield
* 4 DC motors

**Installation**
---------------

### ROS2 Humble Installation

Follow the official ROS2 installation guide for Ubuntu 22.04: https://index.ros.org/doc/ros2/Installation/Linux-Install-ROS-2/

### Clone and Run the Code

1. Clone the repository:
```
   git clone https://github.com/TheHacker3256/omnicar.git
```
2. Navigate to the repository:
```
   cd omnicar
```
3. Install the required dependencies:
```
   rosdep install -i --from-paths package --rosdistro jazzy
```
4. Run the car (Real Bot):
```
  ros2 launch omnicar launch_bot.launch.py
```
4. Run the car (Simulation):
```
  ros2 launch omnicar launch_sim.launch.py
```

**Docker Container**
-------------------
1. Pull the docker image
```
docker pull thehacker3256/omnicar:latest
```
2. Run the container
```
docker run --cap-add=SYS_PTRACE --security-opt=seccomp=unconfined --ipc=host --network=host --pid=host --privileged -name=omnicar thehacker3256/omnicar:latest
```


**Hardware Interface**
---------------------

The car uses an Arduino Uno as a hardware interface, connected to a Duinotech Motor Driver Shield and 4 DC motors. The Arduino is configured to talk to the Raspberry Pi using the Ros2ArduinoBridge library: https://github.com/TheHacker3256/Ros2ArduinoBridge

**ROS2 Control**
----------------

The car uses ROS2 Control to interface with the hardware. The `launch_bot.launch.py` file is used to launch the car and configure the hardware interface.
