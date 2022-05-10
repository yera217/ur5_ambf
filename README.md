# Evaluation of vRCM for MIS. Project#24
 
## Description: 
This project is focusing on implementation and evaluation of Virtual Remote Center of Motion (vRCM) for UR5 robot manipulator with daVinci large
needle driver attached in AMBF environment

Demo video of the implementation:

[![Demo of vRCM](https://img.youtube.com/vi/sJOVtvhsAN0/0.jpg)](https://www.youtube.com/watch?v=sJOVtvhsAN0)

## Usage:
### 1. Install AMBF:
Follow instructions of Section 4.2 on AMBF github page [https://github.com/WPI-AIM/ambf]

### 2. Clone the project repo
```
cd ~
git clone https://github.com/yera217/ur5_ambf.git
```

### 3. Running AMBF with UR5, trocar and target object initialized from launch file
```
roscore
cd ~/ambf/bin/lin-x86_64/
./ambf_simulator --launch_file ~/ur5_ambf/launch.yaml -l 0,1,2
```

### 4. Load UR5 urdf robot description to ROS parameter server
```
cd ~/ur5_ambf/ur5_ws/
catkin build
source devel/setup.bash
roslaunch ur5_rcm ur5_upload.launch
```

### 5. Run scripts with AMBF client and RCM node:

For RCM test:
```
cd ~/ur5_ambf/scripts/
python ur5_rcm.py
```
Press '2' key in AMBF window to turn off gravity in the simulation. It is needed because velocity control of joints in AMBF is open-loop. In future work, it will be converted to position control so that vRCM is working in presence of gravity.

You should see the UR5 following the target frame of tool position while respecting RCM trocar point.
