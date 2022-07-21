# collision-simulation
under construction! 

build with cmake:

`cd collision-simulation`
`cmake .`
`make`

currently not tested on a live robot. In the 'test' folder you can test the code on offline maps. The program turnMaps.sh goes into the selected folders containing maps and turns them 90 degrees.

## prerequisites
### install box2d and opencv
using `dselect` , install `libbox2d-dev` and 'libopencv-core-dev'

### rplidar
rplidar: https://github.com/berndporr/rplidar_rpi

### timer
cpptimer:https://github.com/berndporr/cppTimer

