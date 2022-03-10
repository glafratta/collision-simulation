# collision-simulation
under construction! 

build with cmake:

`cd collision-simulation`
`cmake .`
`make`

run: `sudo ./simulate` : creates a representation of the external world and calculates the velocity at which the robot is moving [rplidar_rpi] 
(https://github.com/berndporr/rplidar_rpi).In this program the robot is always at (0,0).

## prerequisites
### install box2d and opencv
using `dselect` , install `libbox2d-dev` and 'libopencv-core-dev'

### rplidar
rplidar: https://github.com/berndporr/rplidar_rpi

### timer
cpptimer:https://github.com/berndporr/cppTimer

