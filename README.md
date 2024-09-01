# libwaterlinked

libwaterlinked is a C++ library and ROS 2 driver designed to interface with
[Water Linked DVL devices](https://waterlinked.com/dvl), including the DVL-A50
and DVL-A125. Get started with libwaterlinked by installing the project or by
exploring the implemented [examples](https://github.com/Robotic-Decision-Making-Lab/libwaterlinked/tree/main/examples).

> :warning: This project is not affiliated with or maintained by Water Linked.
> Please refer to the Water Linked [GitHub Organization](https://github.com/waterlinked/)
> for all official software.

## Installation

To install libwaterlinked, first clone the repository to the `src/` directory
of your ROS 2 workspace

```bash
git clone git@github.com:Robotic-Decision-Making-Lab/libwaterlinked.git
```

Then install the project dependencies using vcstool and rosdep

```bash
vcs import src < src/libwaterlinked/ros2.repos && \
rosdep install --from paths src -y --ignore-src
```

Finally, build the workspace using colcon

```bash
colcon build
```

## Getting help

If you have questions regarding usage of libreach or regarding contributing to
this project, please ask a question on our [Discussions](https://github.com/Robotic-Decision-Making-Lab/libwaterlinked/discussions)
board.

## License

libwaterlinked is released under the MIT license.
