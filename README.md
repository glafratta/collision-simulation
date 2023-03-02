# carmelo
A library to implement spatial navigation based on homeostatic regulation of navigation mode and forward-chaining reasoning for optimal path planning for an indoor robot

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


