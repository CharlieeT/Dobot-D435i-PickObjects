# Dobot-D435i-PickObject
 
<a name="readme-top"></a>
# Industrial Robotics
### **Dobot Magician and Intel D435i Colour Blocks Picker**

Contributors: Li-Ting Tsai (Charlie Tsai)

## Project Description
The main focus of this project is to implement the Intel D435i RGB-D camera to sense the environment while using the collected data to determine the ideal response for the Dobot Magician robotic arm for picking and placing the object of user’s choosing. In this project, Robotic Operating System (ROS) and MATLAB was used to stream and access the 3D point cloud data for colour identification as well as target pose calculation for the setting of waypoints for the picking location and utilise said information to manipulate the position of different colour blocks to their allocated drop zone.

## Installation and Setup Guide
### Matlab Codes:
1. Download or git clone the repository into a folder of your choice
2. Install the following add-ons in MATLAB:
    - Robotics System Toolbox
    - ROS Toolbox
3. Open MATLAB and add the folder into your MATLAB file path
4. Run 'startup_rvc.m' from the path (.\Dobot-D435i-PickObjects\src\rvctools)
5. Run 'main_realDobot.m' located at path  (\Dobot-D435i-PickObjects\src\Real Demo)

### Running the DOBOT Magician and Intel D435i using Ubuntu 18.04 and ROS Melodic
1. [Install](http://wiki.ros.org/melodic/Installation/Ubuntu) ROS Melodic and setup a catkin workspace using the tutorial [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
2. Download the required DOBOT Magician packages, refer to the guide [here](https://github.com/gapaul/dobot_magician_driver/wiki/Instructions-For-Native-Linux)
3. Download and Install the Intel Realsense D435i package. Please visit the official guide [here]
(https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
4. Git clone the repository onto your linux system
```sh
git clone https://github.com/CharlieeT/Robotics.git
```
* For private repo
```sh
git clone https:yourusername@github.com/Robotics.git
```
5. Follow step 2-4 in the simulation section
6. Run 'main_simulation.m' which is located in Robotics\Code\Real Demo

## Troubleshooting
-  If you are having python version issue with MATLAB please type this in command box of MATLAB to set python to 3.9
```sh
pyenv("Version", "/usr/bin/python3.9") 
```
-  Use a USB 3.2 cable for the connection between your realsense camera to your computer.


<p align="right">(<a href="#readme-top">back to top</a>)</p>
