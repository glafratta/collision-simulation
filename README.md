# HICS - hierarchical input controller sequences
This library allows obstacle avoidance in an indoor robot by creating Tasks, i.e. input controllers, in order to eliminate Disturbances. With this perspective, we view spatial navigation as an effort to maintain and restore homeostasis. This means that the robot has a preferred default Task, and new Tasks are created and executed upon its disruption, with the goal of returning to the steady state. This process is orchestrated by a Configurator and it is sufficient to perform reactive avoidance. However, Tasks may also be chained in a tree graph and simulated internally in order to find a Task which, if executed next, leads to fulfilment of a control goal.In this implementation the control goal is to maximise the amount of time spent by the robot in the steady state. The Task graph is created based on continuously updating visual information about the robot's surroundings and simulation of each Task in game engine [Box2D](https://github.com/erincatto/box2d).

## Hardware
The indoor robot is equipped with 
* 360 Parallax Continuous Rotation Servo motors (see [here](https://github.com/berndporr/alphabot/blob/main/alphabot.cpp) for wiring)
* A1 SLAMTEC LIDAR (see [here](https://github.com/berndporr/rplidar_rpi) for wiring)
* Raspberry Pi model 3b+

## Prerequisites
### Development packages

* G++ compiler
* CMake
* PiGPIO library
* OpenCV
* Boost
* XOrg
* LibGLU1

`sudo apt install g++ cmake libpigpio-dev libopencv-dev libboost-all-dev xorg-dev libglu1-mesa-dev`

### Compile from source

* [LIDAR API](https://github.com/berndporr/rplidar_rpi)
* [Motors API](https://github.com/berndporr/alphabot)
* [Cpp Timer](https://github.com/berndporr/cppTimer)
* [Box2D v2.4.1](https://github.com/erincatto/box2d)
  ** if not installed automatically, go to `box2d/build` and run `sudo make install`

## Build
```
cd HICS
cmake .
sudo make install
```

## Run
### Navigation demo (Raspberry Pi)
* `sudo ./navigate` : this program demonstrates optimal next mode selection based on its own and its child branches' outcome. By default, this only print out statements which allow to follow the robot's behaviour; you can turn on debug mode by instead typing `sudo ./navigate 1`. This prints out in `/tmp` a) LIDAR data with the prefix `map`, b) the bodies included in Box2D withthe prefix `bodies` and c) the trajectory descried by the robot during simulation with the prefix `robot`

