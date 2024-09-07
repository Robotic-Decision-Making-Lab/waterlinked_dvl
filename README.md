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

<<<<<<< HEAD
1. Use this repository [as a template](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template)
for your project
<<<<<<< HEAD
2. Replace all instances of "ros2-template" with your own project's name
=======
2. Replace all instances of "libwaterlinked" with your own project's name
>>>>>>> cc84fbc (Finished initial implementation of driver and renamed to libwaterlinked)
3. Replace the source code with your own project!
=======
```bash
vcs import src < src/waterlinked_dvl/ros2.repos && \
rosdep install --from paths src -y --ignore-src --skip-keys nlohmann_json
```
>>>>>>> 2d5736b (Added readme)

Finally, build the workspace using colcon

```bash
colcon build && source install/setup.bash
```

## Usage

Prior to using waterlinked_dvl, first ensure that you can successfully connect
to your respective device. For additional information, please refer to the
[Water Linked networking documentation](https://waterlinked.github.io/dvl/networking/).
After verifying the network connection, the DVL ROS 2 driver can be launched
with the following command:

```bash
ros2 launch waterlinked_dvl_driver dvl.launch.py
```

## Getting help

<<<<<<< HEAD
<<<<<<< HEAD
If you have questions regarding usage of this project or would like to
<<<<<<< HEAD
contribute, please ask a question on our [Discussions](https://github.com/Robotic-Decision-Making-Lab/ros2-template/discussions)
=======
contribute, please ask a question on our [Discussions](https://github.com/Robotic-Decision-Making-Lab/libwaterlinked/discussions)
>>>>>>> cc84fbc (Finished initial implementation of driver and renamed to libwaterlinked)
board!
=======
If you have questions regarding usage of libreach or regarding contributing to
this project, please ask a question on our [Discussions](https://github.com/Robotic-Decision-Making-Lab/libwaterlinked/discussions)
=======
If you have questions regarding usage of waterlinked_dvl or regarding contributing
to this project, please ask a question on our [Discussions](https://github.com/Robotic-Decision-Making-Lab/waterlinked_dvl/discussions)
>>>>>>> b4ac968 (Finished connection timeout implementation and renamed packages)
board.

## License

<<<<<<< HEAD
libwaterlinked is released under the MIT license.
>>>>>>> 2d5736b (Added readme)
=======
waterlinked_dvl is released under the MIT license.
>>>>>>> b4ac968 (Finished connection timeout implementation and renamed packages)
