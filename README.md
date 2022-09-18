
# Tello_formation_and_visual_control

# Working in Progress

By Dr. Yuan Shenghai       syuan003@e.ntu.edu.sg   yuanshenghai@outlook.com

In this repository, we provide a through study and use case for using tello drone.

Tello is a low cost drone with sdk capability. It is controlled by 2.4G wifi and can be used in swarm formation. The only limitation is that the drone is streaming video and controlled by the same ip address 192.168.10.1. When using multiple drone together, it is slightly more complicated to configurate and there is no complete tutorial in teaching how to config. 

There are two different control mode. One is typical AP mode where drone function like a access point and all device can connect to it. The other mode is station mode which all drone connects to the same router. 

Below are some typical use case of tello. 

![Image of Tello UI](https://github.com/snakehaihai/Tello_formation_and_visual_control/blob/master/Images/tello_use_case_part_I.png)




![Image of Tello UI](https://github.com/snakehaihai/Tello_formation_and_visual_control/blob/master/Images/tello_use_case_part_II.png)


# Mode 1: App based control ( system verfication and activation ) 

This is what the tello suppose to do. Run this first if you have a brand new tello or you cant control the tello by its sdk.  If you encounter any problem, make sure in this mode you can control it. If in this mode, you still can not control, then likely you had a hardware issue. 

1. Turn on the tello and wait for the light to turn flashing yellow  

2. Turn on the cellphone wifi and connect to the IP with the name Tello_XXXXXX  where XXXXXX is the tello ID  ( if can not see, w8 for a while or press the power button for more then 5 sec and wait for light to goes off  for a system reset from station mode)

3. After you connect to the drone, turn on the tello app which is available in the app market. For the first time user, you will be asked to activate the drone which is standard process for all dji drones. ( We ignore this step before and goes to mode 2 directly and it doesnt take off, took us a while to figure out that we have to activate it)

4. After activation, you are free to fly and navigate. 

If you can fly by this stage, then you are good to go to the next step.


# Mode 2: Script based control ( SDK testing )

The purpose of this mode is to make sure that the tello sdk works on your PC. 

We only verified this in Ubuntu 18.04.05 in both hardware notebook and virtual machine. For other machine, it is possible to run it. but can not garuntee that it will work. Most of the command you can find from https://github.com/tau-adl/Tello_ROS_ORBSLAM. I added a few key step which stoped many people from getting it running.

Assuming ROS melodic is installed http://wiki.ros.org/melodic/Installation/Ubuntu

The first is to install 

# Usage
## orbslam2
```
roslaunch flock_driver orbslam2_with_cloud_map.launch
```

Once you have added that repository, run these commands to install catkin_tools:
```
sudo apt-get update
sudo apt-get install python-catkin-tools
```
### Eigen3
Required by g2o. Download and install instructions can be found here. Otherwise Eigen can be installed as a binary with:
```
sudo apt install libeigen3-dev
```
### ffmpeg
```
sudo apt install ffmpeg
```
### Python catkin tools (probably already installed)
```
sudo apt-get install python-catkin-tools
```
### Joystick drivers
Tested it only on melodic.
```
sudo apt install ros-melodic-joystick-drivers
```
### Python PIL
```
sudo apt-get install python-imaging-tk
```
## Github based Prerequisites
### Pangolin (used in orbslam2)
Based on https://github.com/stevenlovegrove/Pangolin
```
cd ~/ROS/
git clone https://github.com/stevenlovegrove/Pangolin.git
sudo apt install libgl1-mesa-dev
sudo apt install libglew-dev
sudo apt-get install libxkbcommon-dev
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build
```

### h264decoder
Baed on https://github.com/DaWelter/h264decoder
```
cd ~/ROS/
git clone https://github.com/DaWelter/h264decoder.git
```
Inside h264decoder.cpp replace PIX_FMT_RGB24 with AV_PIX_FMT_RGB24
```
mkdir build
cd build
cmake ..
make
```
now copy it to python path
```
sudo cp ~/ROS/h264decoder/libh264decoder.so /usr/local/lib/python2.7/dist-packages
```
# Installing Our Repository
## Cloning Our repo from github
```
cd ~
mkdir ROS
cd ROS
git clone https://github.com/snakehaihai/Tello_formation_and_visual_control
```
## Installing our version of TelloPy
based on https://github.com/dji-sdk/Tello-Python and https://github.com/hanyazou/TelloPy
```
cd ~/ROS/Tello_ROS_ORBSLAM/TelloPy
sudo python setup.py install
```
## Installing dependencies for ROS
```
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

# Installing orbslam2
based on https://github.com/appliedAI-Initiative/orb_slam_2_ros and https://github.com/rayvburn/ORB-SLAM2_ROS
First - if using Melodic version of ROS, change the ~/ROS/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/src/orb_slam_2_ros/CMakeLists.txt
To the CMakeLists_melodic.txt
## Build the code:
```
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/
catkin init
catkin clean
catkin build
```
If it doesn’t work, make sure you changed the makefile to the wanted version of ROS
## Add the enviroment setup to bashrc
```
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
# Installing ccm_slam
based on https://github.com/VIS4ROB-lab/ccm_slam
## Compile DBoW2:
```
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/ccmslam_ws/src/ccm_slam/cslam/thirdparty/DBoW2/
mkdir build
cd build
cmake ..
make -j8
```
## Compile g2o:
```
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/ccmslam_ws/src/ccm_slam/cslam/thirdparty/g2o
mkdir build
cd build
cmake --cmake-args -DG2O_U14=0 ..
make -j8
```
## Unzip Vocabulary:
```
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/ccmslam_ws/src/ccm_slam/cslam/conf
unzip ORBvoc.txt.zip
```
## Build the code:
```
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/ccmslam_ws/
source /opt/ros/melodic/setup.bash
catkin init
catkin config --extend /opt/ros/melodic
catkin build ccmslam --cmake-args -DG2O_U14=0 -DCMAKE_BUILD_TYPE=Release
```

If Gives error -  ROS distro neither indigo nor kinetic - change the makefile, use CmakeFile_changed2.
## Add the enviroment setup to bashrc
```
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Notes
### Flock
we have used code from https://github.com/clydemcqueen/flock
