#include "custom.h"
void Configurator::done_that(vertexDescriptor& src, bool& plan_works, b2World& world, std::vector<vertexDescriptor> &plan_provisional){

}

Disturbance set_target(int& run, b2Transform start){

}

void forget(Configurator* c){}


int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	configurator.planning =0;
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
	}
	if (argc>2){
		configurator.setSimulationStep(.50);
	}
	printf("CHASING A TARGET\n");
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface);
	Callback cb(&configurator);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	configurator.start();
	lidar.start();
	motors.start();
	do {
	} while (!getchar());
	configurator.stop();
	motors.stop();
	lidar.stop();

}
	
	
