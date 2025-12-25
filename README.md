# gnsslib

**Authors/Maintainers:** Ren Zhongbin

The *gnsslib* package is a ROS-based GNSS navigation and positioning tool that currently supports SPP and RTK processing. It encompasses the processing of raw measurement data and fundamental definitions for global navigation satellite systems.

The project currently has some issues and we will keep you updated. If you have any questions, please contact us. (RZB24@buaa.edu.cn)

## 1. Prerequisites

### 1.1 C++11 Compiler
This package requires some features of C++11.

### 1.2 ROS
This package is developed under [ROS Kinetic](http://wiki.ros.org/kinetic) environment.

### 1.3 Eigen
Our code uses [Eigen 3.3.3](https://gitlab.com/libeigen/eigen/-/archive/3.3.3/eigen-3.3.3.zip) for matrix manipulation. After downloading and unzipping the Eigen source code package, you may install it with the following commands:

```
cd eigen-3.3.3/
mkdir build
cd build
cmake ..
sudo make install
```

### 1.4 Glog
We use google's glog library for message output. If you are using Ubuntu, install it by:
```
sudo apt-get install libgoogle-glog-dev
```
If you are on other OS or just want to build it from source, please follow [these instructions](https://github.com/google/glog#building-glog-with-cmake) to install it.


## 2. Build gnsslib library
Clone the repository to your catkin workspace (for example `~/catkin_ws/`):
```
cd ~/catkin_ws/src/
git clone https://github.com/RenZongbin/gnsslib.git
```
Then build the package with:
```
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 4. Run gnsslib library
If you have successfully built this package, you could download this example [GVINS-Dataset](https://github.com/HKUST-Aerial-Robotics/GVINS-Dataset) and launch :
```
roslaunch gnss_comm rtk_file.launch
```
Then play the bag:
```
rosbag play sports_field.bag
```

## 3. Part of gnsslib
	gnsslib/
	├── CMakeLists.txt       # Build configuration
	├── package.xml          # ROS package manifest
	├── exampdata            # Example data files
	├── src/                 # Source code
	├── include/             # Header files
	├── msg/                 # Custom ROS messages
	├── launch/              # ROS launch files
	├── doc/                 # Documentation
	├── cmake/               # CMake modules
	└── docker/              # Docker configuration

## 5. Acknowledgements
This package is developed based on [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm)and [RTKLIB](http://www.rtklib.com/).

Many of the definitions and utility functions in this package are adapted from [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) and [RTKLIB](http://www.rtklib.com/).

## 6. License
The source code is released under [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html) license.


