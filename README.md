# carmelo
A library to implement spatial navigation based on homeostatic regulation of navigation mode and forward-chaining reasoning for optimal path planning for an indoor robot. Homeostatic navigation means that the robot has a preferred default mode, and new modes are created and executed upon its disruption, with the goal of returning to the steady state. The optimal next mode is chosen by selecting, in a graph of potentially unlimited future modes, the one which maximises the amount of time spent by the robot in the steady state. The mode graph is created based on continuously updating visual information about the robot's surroundings and simulation of the outcome of sequential mode-switching in game engine [Box2D](https://github.com/erincatto/box2d).

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
* Box2D
* OpenCV
* Boost

`sudo apt install g++ cmake libpigpio-dev libbox2d-dev libopencv-dev libboost-all-dev`

### GitHub repositories

* [LIDAR API](https://github.com/berndporr/rplidar_rpi)
* [Motors API](https://github.com/berndporr/alphabot)
* [Cpp Timer](https://github.com/berndporr/cppTimer)

## Build
```
cd carmelo
cmake .
make
```
Warning: the CMakeLists.txt file is built in such a way that it will only build the exectuable for the Raspberry Pi if an aarch64 architecture is detected, so you can only build simulated navigation programs on a desktop computer).

## Run
### Navigation demos (Raspberry Pi)
* `sudo ./navigate` : this program demonstrates optimal next mode selection based on its own and its child branches' outcome. 
### Test cases (Simulation
* `./test/noRobot <your-map-folder> <bool timerOff> optional:<bool planningOn>` : this program allows to test the functionality of the "navigate" executable on any machine. Just provide it with a folder containg TSV files of (x, y) coordinates, whether you'd like the simulation to run in real-time and optionally if you'd like to see navigation with no mode planning
* `./test/SPF` : this program illustrates in sequence how the mode tree is build using a Boost class
* `./test/testEnvironmentRepresentation`: this program illustrates the selective object representation in Box2D, tailored to the execution of the mode being simulated
* `./test/synthData`: allows to generate synthetic data for debug purposes from one single frame of coordinates
