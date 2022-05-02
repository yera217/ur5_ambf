# Evaluation of vRCM for MIS. Project#24
 
## Description: 
This project is focusing on implementation and evaluation of Virtual Remote Center of Motion (vRCM) for UR5 robot manipulator with daVinci large
needle driver attached in AMBF environment



## Usage:
###1. Install AMBF:
Follow instructions of Section 4.2 on AMBF github page [https://github.com/WPI-AIM/ambf]

###2. Clone the project repo
```
cd ~
git clone https://github.com/yera217/ur5_ambf.git
```

###3. Running AMBF with UR5 initialized from launch file
```
roscore
cd ~/ambf/bin/lin-x86_64/
./ambf_simulator --launch_file ~/ur5_ambf/launch.yaml -l 1
```

###4. Load UR5 urdf robot description to ROS parameter server
```
cd ~/ur5_ambf/ur5_ws/
source devel/setup.bash
roslaunch ur5_rcm ur5_upload.launch
```

###5. Run scripts with AMBF client and RCM node:
```
rosrun ur5_rcm test_pubsub_ambf.py
rosrun ur5_rcm ur5_fk.py
```

You should see the UR5 trying to follow hard-coded (for now) RCM point and input (x,y,z,R,P,Y) position. It is still on debugging/testing stage.
