#include "custom.h"

void forget(Configurator *c){}

void Configurator::done_that(vertexDescriptor& src, bool& plan_works, b2World& world, std::vector<vertexDescriptor> &plan_provisional){

}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	configurator.planning =1;
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
	}
	configurator.setSimulationStep(.27);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface);
	MotorCallback cb(&configurator);
	lidar.registerInterface(&dataInterface);
	CameraCallback cameraCB(&cb);
	Libcam2OpenCV camera;
    camera.registerCallback(&cameraCB);
    Libcam2OpenCVSettings settings;
    settings.framerate = FPS;
	motors.registerStepCallback(&cb);
	configurator.start();
	lidar.start();
    camera.start(settings);
	motors.start();
	getchar();
	configurator.stop();
	motors.stop();
	lidar.stop();
	camera.stop();
}
	
	
