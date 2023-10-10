# eseo-s9-aquabot

## Setup

We need to install some tools to be able to run the simulation. Follow each step if you have a fresh Ubuntu 22.04 installation. If you already have some of the tools installed, you can skip the corresponding step. This was tested on a fresh Ubuntu 22.04 installation.

### Setup ROS humble

First we need to install ROS humble, which is the required version for vrx. This script will install ROS humble based on [official documentation](https://docs.ros.org/en/humble/Installation.html).

```bash
./install_ros_humble.sh
```

### Setup GAZEBO garden

Then we install GAZEBO garden, which is the required version for vrx. This script will install GAZEBO garden based on [official documentation](https://gazebosim.org/docs/garden/install_ubuntu).

```bash
./install_gazebo_garden.sh
```

### Setup VRX

[Virtual RobotX](https://github.com/osrf/vrx) (VRX) is a high fidelity, Gazebo-based simulation environment for testing maritime robots. This script will install VRX based on [official documentation](https://github.com/osrf/vrx/wiki/getting_started_tutorial)

```bash
./install_vrx.sh
```
