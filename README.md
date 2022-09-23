# carmelo
Under construction. A library to implement "innate" understanding of physics in an obstacle-avoidant robot, and utilise it for planning trajectories and learning about the environment.
The program turnMaps.sh goes into the selected folders containing maps and turns them 90 degrees.
the subdirectory "test" contains test cases before the program is actually implemented in a real robot. noRobot.h is a simulation which does not require a robot but illustrates path planning and trajectory correction.

######################################
## prerequisites
### install box2d and opencv
using `dselect` , install `libbox2d-dev` and 'libopencv-core-dev'

### rplidar_rpi API
https://github.com/berndporr/rplidar_rpi

###alphabot API
https://github.com/berndporr/alphabot

See here for the wiring of the robot.
### timer
cpptimer:https://github.com/berndporr/cppTimer

##build
build with cmake:

`cd carmelo`
`cmake .`
`make`


