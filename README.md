# ROS2 Sample program for trutlebot3 (Gazebo simulator)  
## Overview
This sample program keeps tb3 moving on Gazebo.

[Demo (gif)](https://github.com/TomoyaKamimura/nav2_gazebo/blob/main/nav2_gazebo_sample.gif)

## Tested environment
- OS : Ubuntu 22.04 (Virtualbox)
- ROS2 Distribution : humble
## Getting Started
### ROS2 setup
#### 1. Set locale
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```



#### 2. Setup Sources
Activate Ubuntu Universe
```
sudo apt update
sudo apt-get install software-properties-common
sudo add-apt-repository universe
```
Add ROS2 GPG key
```
sudo apt-get install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Add repository to sources list
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```



#### 3. Install ROS2 packages
```
sudo apt update
```
Then,
```
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
sudo apt install python3-argcomplete
```



#### 4. Install Colcon for build project
```
sudo apt install python3-colcon-common-extensions
```



#### 5. Install Turtlebot3 & Gazebo simulator
Install turtlebot3 package
```
sudo apt install ros-humble-turtlebot3
```
Then, install Gazebo
```
sudo apt install ros-humble-turtlebot3-gazebo
```



#### 6. Setup some environment value and setup script
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc
source ~/.bashrc
```


### Build project
#### 1. Clone this repository
```
https://github.com/TomoyaKamimura/nav2_gazebo.git
```
#### 2. Build project
```
cd nav2_gazebo
colcon build
```
## Usage
### 1. launch Gazebo simulator
First, launch terminal and use following command.
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```
Then, you are able to see tb3 on Gazebo simulator


### 2. launch nav2_gazebo program
Launch another terminal and use following commands
```
cd nav2_gazebo
source install/setup.bash
```
Then, launch program
```
ros2 run nav2_gazebo navigation
```
## Reference
ROS2 Setup
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Nav2
https://docs.nav2.org/getting_started/index.html
https://docs.nav2.org/commander_api/index.html
