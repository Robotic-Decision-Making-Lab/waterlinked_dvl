# libwaterlinked

libwaterlinked is a C++ library with accompanying ROS 2 nodes designed to
interface with [Water Linked DVL devices](https://waterlinked.com/dvl), including
the DVL-A50 and DVL-A125. Get started with libwaterlinked by installing the
project or by exploring the implemented [examples](https://github.com/Robotic-Decision-Making-Lab/libwaterlinked/tree/main/examples).

> :warning: This project is not affiliated with or maintained by Water Linked.
> Please refer to the Water Linked [GitHub Organization](https://github.com/waterlinked/)
> for all official software.

## Installation

To install libwaterlinked, first clone the repository to the `src/` directory
of your ROS 2 workspace

```bash
git clone -b ros2 git@github.com:Robotic-Decision-Making-Lab/libwaterlinked.git
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
vcs import src < src/libwaterlinked/ros2.repos && \
rosdep install --from paths src -y --ignore-src
```
>>>>>>> 2d5736b (Added readme)

Finally, build the workspace using colcon

```bash
colcon build
```

## Getting help

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
board.

## License

libwaterlinked is released under the MIT license.
>>>>>>> 2d5736b (Added readme)
