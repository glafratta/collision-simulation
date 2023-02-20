# carmelo
 A library for real-time planning of optimal trajectories for an indoor robot in completely unseen environments, without requiring any learning. This project utilises 2D physics library [box2d](https://github.com/erincatto/box2d) to implement innate understanding of physics and online assignment of physical properties to the robot's surroundings to inform spatial decision making. In this framework, decision making and path planning are the output of a closed-loop input control process, where the inputs are spatial information about any disturbances to the robot's current state (i.e. obstacles), which the spatial decisions work to eliminate. The robot's state is therefore simply the current task being carried out which corresponds to a stereotyped motor output and a new task or state is created upon detection of a disturbance; this can be used for reflexive obstacle avoidance, but also for more complex, waypoint-based optimal trajectory planning by creating a tree of states and choosing the next task which is predicted to lead to the best outcome.
 
## Hardware

The robot is controlled by a Raspberry Pi model 3b+ and currently equipped with 2 Parallax continuous rotation servos and a SLAMTEC A1 LIDAR wired and controlled as per the [alphabot] (https://github.com/berndporr/alphabot) and [rplidar_rpi](https://github.com/berndporr/rplidar_rpi) APIs, respectively.
 
## Software 

- [box2d](https://github.com/erincatto/box2d)
- OpenCV Core module
- Boost
- [cpptimer] (https://github.com/berndporr/cppTimer) for the test program noRobot.h
- [alphabot] (https://github.com/berndporr/alphabot)
- [rplidar_rpi](https://github.com/berndporr/rplidar_rpi)
Box2D, OpenCV development files can be installed on Linux OS's via dselect ('libbox2d-dev', 'libopencv-core-dev').

## Build

build with cmake:
```
cd carmelo
cmake .
make
```


