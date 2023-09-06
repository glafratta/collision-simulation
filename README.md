# HICS - hierarchical input controller sequences
This library is designed for spatial navigation in a wheeled robot equipped with a LIDAR sensor. The navigation problem is broken down in individual Tasks which correspond to the act of performing a single motor Action (go straight, turn left/right 90 degrees, go back) in order to eliminate a Disturbance, which is a process that occurs in a closed-loop fashion. Disturbances are identified by the Configurator, an overarching module capable of predicting the outcome of each Task by simulating it in game engine [Box2D](https://github.com/erincatto/box2d) over a certain time/distance horizon. Using this framework, sequences of Tasks representing plans towards a control goal can be generated into a graph structure, and evaluated using the simulation environment. In this implementation, nodes are discovered in the graph using the A* algorithm.

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
* `sudo ./reactive`: this program demonstrates reactive avoidance
* `sudo ./targetless` : this program demonstrates planning over a 1m distance horizon for a control goal that is not a target location but rather an objective to drive straight for the longest time with the least amount of disturbances
* `sudo ./target`: this program (under construction) demonstrates target seeking behaviour, where the target is imaginary and located at x=1.0m, y=0m.
* `sudo ./targetless_benchmark`: for data collection; the program creates a folder called bodiesSpeedStats1.0, which you can navigate to and run the script `sh ../speed_analysis.sh`

### Settings
The automatic settings is for debug files to be turned off and for the robot to simulate its plans in chunks of 1m each (half the sensor range). To change these settings just run the above executables as `your_executable 1 your_step_size`. From experimental findings, we recommend using a step the size of the robot's diameter.
