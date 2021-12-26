# WA Simulator ROS 2 Bridge

This package provides a ROS 2 interface to [WA Simulator](https://github.com/WisconsinAutonomous/wa_simulator). 

## Getting Started

You must have `wa_simulator` installed on your system. To do this, please refer to the [documentation](https://wisconsinautonomous.github.io/wa_simulator). You will also need ROS 2 installed. It's recommended you use Docker, as seen in our [ROS 2 tutorial](https://github.com/wisconsiniautonomous/wa_ros_tutorial).

You will then need to add this repository as a submodule in your workspace or simply clone it. You can do that with one of the following commands:

**Submodule (Recommended)**
```bash
git submodule add -b foxy git@github.com:WisconsinAutonomous/wa_simulator_ros_bridge.git
```

**Clone**
```bash
git clone -b foxy git@github.com:WisconsinAutonomous/wa_simulator_ros_bridge.git
```

## Usage

The `wa_simulator_ros_bridge` has one node: `bridge.py`. To run the node, run the following command:

```bash
ros2 run wa_simulator_ros_bridge bridge.py
```

*More documentation to come...*
