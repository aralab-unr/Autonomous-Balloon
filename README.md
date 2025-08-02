<div align="center">
    <h2>Autonomous Balloon: Autonomous Balloon based Adaptive Sliding Mode Control and Infinite-Horizon POMDP</h2>
</div>


Our paper is available at: [Balloon_POMDP_ASMC.pdf](https://github.com/user-attachments/files/21555560/Balloon_POMDP_ASMC.pdf). If our project supports your projects, please cite our work. Thank you!

# Highlights

This project provides a simulation platform for super-pressure balloons, featuring both a Simulink/MATLAB-based simulation and a quasi-physical model implemented in Gazebo/ROS 2. The experiments are conducted on a real platform weighing 75 grams, which is capable of tracking the desired altitude, estimating wind conditions, and making autonomous decisions.

# Simulations
We present results obtained from both Simulink/MATLAB and Gazebo/ROS 2 simulations, using parameters based on a real balloon to evaluate the effectiveness of the proposed method. The results are divided into two parts: (1) tracking the desired altitude, and (2) an autonomous balloon that estimates wind conditions and makes decisions to maintain a specific position.

# Simulink/Matlab
The diagram of the simulation in MATLAB/Simulink is presented as follows:

<img width="2676" height="1122" alt="matlabballoon" src="https://github.com/user-attachments/assets/eea2ea10-958c-423c-9d37-2d991463decf" />

The simulation files are located in the BalloonMatlab folder. The SPballoonASMC.slx file is used for tracking the desired altitude, while the SPballoonASMCPOMDPfull.slx file is for simulating the autonomous balloon.

## Gazebo/ROS 2
The visual model of the balloon in Gazebo is shown below:
<div align="center">
  <img width="400" height="600" alt="image" src="https://github.com/user-attachments/assets/75b7ff34-a48d-482f-b6ff-6f844e31712b" />
</div>

The simulation on Gazebo/ROS 2 running on the following software setup: 
- Ubuntu 22.04 + ROS2 Humble
- Gazebo 11
- Xarco-ROS-Humble (sudo apt install ros-humble-xacro)
- Gazebo_ros_pkgs (sudo apt install ros-humble-gazebo-ros-pkgs)


To set up the simulation on Gazebo/ROS 2 :

```shell
# Step 1: Create and build a colcon workspace:
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/
$ colcon build
$ echo "source ~/ros2_ws/devel/setup.bash" >> ~/.bashrc

# Step 2: Download the source file on the folder rosballoon into your workspace
$ cd ~/ros2_ws/src
Download the folder rosballoon in the main branch

# Step 3: Build the colcon workspace for this package
$ cd ~/ros2_ws
$ colcon build
```

To run the simulation on Gazebo/ROS 2 :

```shell
# Step 1: Create and build a colcon workspace:
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/
$ colcon build
$ echo "source ~/ros2_ws/devel/setup.bash" >> ~/.bashrc

# Step 2: Download the source file on the folder rosballoon into your workspace
$ cd ~/ros2_ws/src
Download the folder rosballoon in the main branch

# Step 3: Build the colcon workspace for this package
$ cd ~/ros2_ws
$ colcon build
```

# Experiments
## Design and Hardware Setup
The balloon is designed in SolidWorks, and all parts are 3D-printed. A 36-inch latex balloon is used in the project. The visual design in SolidWorks and the hardware setup are shown in the following figure: 

<img width="1517" height="720" alt="balloonsolidworks" src="https://github.com/user-attachments/assets/c2be34ca-33b6-4979-a677-2e40f3d77ab6" />
The SolidWorks assembly and all the parts used can be found in the Balloon folder. <br> 

The control board is powered by an ESP8266 module, which communicates with the ground station via the TCP protocol. For the indoor prototype, the balloon’s position is measured using the MTF-02P Optical & Range Sensor. All components are carried by a 36-inch latex balloon, with a total system weight of 75 grams. <br> 

For the setup of the station-keeping experiment, refer to the following figure: <br>
<img width="1248" height="720" alt="balloonexperimentsetup" src="https://github.com/user-attachments/assets/8ede1cb6-69ea-47f0-8775-51933f8dafaa" />
