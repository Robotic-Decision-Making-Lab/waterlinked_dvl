# Water Linked DVL API

waterlinked_dvl is a C++ library and ROS 2 driver designed to interface with
[Water Linked DVL devices](https://waterlinked.com/dvl), including the DVL-A50
and DVL-A125. Get started with waterlinked_dvl by installing the project,
exploring the implemented library [examples](https://github.com/Robotic-Decision-Making-Lab/waterlinked_dvl/tree/main/examples),
or by launching the ROS 2 driver.

> :warning: This project is not affiliated with or maintained by Water Linked.
> Please refer to the Water Linked [GitHub Organization](https://github.com/waterlinked/)
> for all official software.

## Installation

To install waterlinked_dvl, first clone the repository to the `src/` directory
of your ROS 2 workspace

```bash
git clone git@github.com:Robotic-Decision-Making-Lab/waterlinked_dvl.git
```

Then install the project dependencies using vcstool and rosdep

```bash
vcs import src < src/waterlinked_dvl/ros2.repos && \
rosdep install --from paths src -y --ignore-src --skip-keys nlohmann_json
```

Finally, build the workspace using colcon

```bash
colcon build && source install/setup.bash
```

## Usage

Run the following command to launch the DVL ROS 2 driver:

```bash
ros2 launch dvl_driver dvl.launch.py
```

## Getting help

If you have questions regarding usage of waterlinked_dvl or regarding contributing
to this project, please ask a question on our [Discussions](https://github.com/Robotic-Decision-Making-Lab/waterlinked_dvl/discussions)
board.

## License

waterlinked_dvl is released under the MIT license.
