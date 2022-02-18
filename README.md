# collision-simulation
under construction! 

build with cmake:

`cd collision-simulation`
`cmake .`
`make`

run: `sudo ./simulate` : this searches into the sample_maps folder and every 200ms creates a static body in box2d world per (x,y) point. This looks different to the data acquired using [rplidar_rpi] 
(https://github.com/berndporr/rplidar_rpi) so acquiring data using that program will not work. In this program the robot is always at (0,0).

## prerequisites
### install box2d
using `dselect` , install `libbox2d-dev`

### rplidar
rplidar: https://github.com/berndporr/rplidar_rpi

### timer
cpptimer:https://github.com/berndporr/cppTimer

