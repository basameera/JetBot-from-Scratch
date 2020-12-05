# ROS Packages for JetBot

* Install ROS [link](https://github.com/JetsonHacksNano/installROS)

## `jetbot_hw` - Jetbot Hardware package

* Access to INA219 - `pip2 install pi-ina219`

### Create catkin workspace

Create a ROS Catkin workspace to contain our ROS packages:

```bash
# create the catkin workspace
mkdir -p ~/workspace/catkin_ws/src
cd ~/workspace/catkin_ws
catkin_make

# add catkin_ws path to bashrc
sudo sh -c 'echo "source ~/workspace/catkin_ws/devel/setup.bash" >> ~/.bashrc'

# source bashrc
source ~/.bashrc
```

### Build jetbot_hw

```bash

# clone the repo and submodules
cd ~/workspace
git clone https://github.com/basameera/JetBot-from-Scratch.git

# copy ros package to the catkin_ws/src
cp -r JetBot-from-Scratch/ROS/jetbot_hw/ ~/workspace/catkin_ws/src

# build 
cd ~/workspace/catkin_ws
catkin_make
```

### Run jetbot_hw

```bash
# Terminal 1
roscore
# Terminal 2
rosrun jetbot_hw ina_pub.py
# Terminal 3
rosrun jetbot_hw ina_sub.py
```