# HyP-DESPOT-Release

The HyP-DESPOT package is developed based on [the DESPOT package](https://github.com/AdaCompNUS/despot). The API in HyP-DESPOT closely follows that in the DESPOT package. See [here](https://github.com/AdaCompNUS/despot/tree/API_redesign/doc) for a detailed documentations of the DESPOT package.

The algorithm was initially published in our RSS paper:

Cai, P., Luo, Y., Hsu, D. and Lee, W.S., HyP-DESPOT: A Hybrid Parallel Algorithm for Online Planning under Uncertainty. Robotics: Science & System 2018. [(PDF)](http://motion.comp.nus.edu.sg/wp-content/uploads/2018/06/rss18hyp.pdf)

## Getting Started

### Pre-requisites
* ROS
* Catkin
* Cmake (version >=3.8)

### Create a catkin workspace:
```bash
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
### Download HyP-DESPOT package:
```bash
git clone https://github.com/AdaCompNUS/HyP-DESPOT-Release.git
mv HyP-DESPOT-Release HyP_despot
```
### Compile HyP-DESPOT package:
```bash
cd ~/catkin_ws
catkin_make --pkg hyp_despot -DCMAKE_BUILD_TYPE=Release 
```
## Main Extensions in HyP-DESPOT from the DESPOT Package
The source files of the HyP-DESPOT solver is in folder [src/HypDespot](src/HypDespot). Main extensions in the folder from DESPOT includes:
```
include/despot/GPUinterface/                   Header files: GPU extension of interface classes in DESPOT
src/GPUinterface                               Source files: GPU extension of interface classes in DESPOT
include/despot/GPUcore/                        Header files: GPU extension of core classes in DESPOT
src/GPUcore                                    Source files: GPU extension of core classes in DESPOT
include/despot/GPUutil/                        Header files: GPU extension of utility classes in DESPOT
src/GPUutil                                    Source files: GPU extension of utility classes in DESPOT
src/solvers/Hyp_despot.cu                      Main file of the HyP-DESPOT solver
src/Parallel_planner.cu                        Parallel extension of the planner class in DESPOT
src/GPUrandom_streams.cu                       GPU version of the RandomStreams class in DESPOT
```

## Car Driving Example
A car driving example (as described in our [RSS paper](http://motion.comp.nus.edu.sg/wp-content/uploads/2018/06/rss18hyp.pdf)) is provided in folder [src/HyP_examples](src/HyP_examples). The key files in this example are:
```
CarDriving/ped_pomdp.cpp                       CPU POMDP model of the car driving problem
CarDriving/GPU_Car_Drive/GPU_Car_Drive.cu      GPU POMDP model of the car driving problem
CarDriving/simulator.cpp                       Custom World (simulator) of the problem
CarDriving/controller.cpp                      The custom planner and the main function
```

## (Optional) Debugging Tools in HyP-DESPOT Package
The  [tools](tools) folder provides tools for debugging HyP-DESPOT, including:
```
Particles*.txt                                 Text files: Particles for different simulation steps to be loaded and used to fix scenarios in HyP-DESPOT.
Streams*.txt                                   Text files: Random streams for different simulation steps to be loaded and used to fix scenarios in HyP-DESPOT
draw_car_cross.py                              Script: to visualize the execution record output by HyP-DESPOT (through cout and cerr)
run_Car_hyp_debug.sh                           Script: to run experiments with HyP-DESPOT
```
The best way to perform debuging is to fix the scenarios and output the search process. This can be acheived by setting the **FIX_SCENARIO** flag defined in [GPUcore/thread_globals.h](src/HypDespot/include/despot/GPUcore/thread_globals.h). Possible vaues are:
```
0         Normal mode
1         Read scenarios from Particles*.txt and Streams*.txt
2         Run in normal mode and export Particles*.txt and Streams*.txt during each simulation step
```
