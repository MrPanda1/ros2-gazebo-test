# ros2-gazebo-test

## Setup

### Ensure locale is installed and setup
```
locale
```

### If locale does not exist, install and setup:
```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8


locale # Verify local install worked
```

### Ensure the Ubuntu Universe Repository is enabled
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### Get the ROS2 GPG Key
```
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Add the ROS2 Repository to our source list
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS2
```
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

### Source the setup script (You may want to add this to .bashrc so it is sourced every time)
```
source /opt/ros/humble/setup.bash
```

### Get the Gazebo and ROS2 dependencies
```
sudo apt-get install -y ros-humble-gazebo-ros-pkgs
```

### Get the turtlebot3 dependencies for this POC
```
sudo apt-get install -y ros-humble-turtlebot3 ros-humble-turtlebot3-simulations
```

### Create Catkin Workspace
```
mkdir -p catkin_ws/src
cd catkin_ws/src
```

### Create ROS Package
```
catkin_create_pkg ros_gazebo_poc
```

### Create mover.py
See repo for file

### Create test_bot_moves.py
See repo for file

### Create launch file ros_gazebo_poc.test
See repo for file

### Update CMakeLists.txt
See repo for file

### Run the test. This requires 3 terminal windows
```
# Window 1: Run gazebo
TURTLEBOT3_MODEL=burger ros2 launch turtlebot3_gazebo empty_world.launch.py

# Window 2: Run mover.py
./mover.py

# Window 3: Run test
launch_test ./test_does_bot_move.py
```

## Refereces:
- https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/
- https://formant.io/news-and-blog/2022/06/14/development/creating-a-robotics-simulation-pipeline-with-github-actions-and-ros/
