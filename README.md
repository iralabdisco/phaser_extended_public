# PHASER Extended

----------------------------
# Getting Started

## Overview
Pointcloud registration using correspondences is inefficient and prone to errors in the many steps of correspondence extraction, description, and matching.
Similarly, the most widespread registration methods work only locally, requiring an initial guess already close to the true solution, something unaffordable in real robotic deployments.
We propose an algorithm for the registration of partially overlapping pointclouds that operates at the global level and on the raw data, i.e., no initial guess as well as no candidate matches are required.
We exploit the properties of Fourier analysis to derive a novel registration pipeline based on the cross-correlation of the phases.

### Packages
PHASER is composed of the following packages:
 - [phaser_core](https://github.com/ethz-asl/phaser/tree/master/phaser_core): The registration core of PHASER. Contains the spherical and spatial correlation.
 - [phaser_ros](https://github.com/ethz-asl/phaser/tree/master/phaser_ros): This is a ROS wrapper to use the PHASER as a registration framework. Hardly used anymore.
 - [phaser_common](https://github.com/ethz-asl/phaser/tree/master/phaser_common): Exposes common classes, utils and models.
 - [phaser_pre](https://github.com/ethz-asl/phaser/tree/master/phaser_pre): Experimental preprocessing of pointcloud data.
 - [phaser_viz](https://github.com/ethz-asl/phaser/tree/master/phaser_viz): Provides visualization functions.
 - [phaser_test_data](https://github.com/ethz-asl/phaser/tree/master/phaser_test_data): Contains example data as PLYs.
 - [phaser_share](https://github.com/ethz-asl/phaser/tree/master/phaser_share): Provides run and build scripts.

## Installation
PHASER requires ROS and some other dependencies to be installed:

### Dependencies

```
  # Some standard requirements
  sudo apt-get install -y doxygen autotools-dev \
     dh-autoreconf libboost-all-dev python-setuptools git g++ cppcheck \
     libgtest-dev python-git pylint \
     python-termcolor liblog4cplus-dev cimg-dev python-wstool \
     python-catkin-tools \

   # Ubuntu 18.04 / ROS Melodic.
   sudo apt-get install -y clang-format-6.0 ros-melodic-pcl-conversions \
     libpcl-dev libnlopt-dev \
```

For the remaining package dependencies, run within the `caktin` workspace

```
  wstool init
  wstool merge phaser/dependencies.rosinstall
  wstool update
```

Building the project:

```
  catkin build phaser_ros
```

Optionally one can build an run all unit tests using:

```
  ./phaser_share/run_build_tests
```
However, this might take some minutes to finish.
