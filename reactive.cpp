#include "custom.h"


int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal; //this line can be removed, it is for illustation purposes
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal); //this line can be removed, it is for illustation purposes
	configurator.planning=0;
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
	}
	configurator.setSimulationStep(.5);
	configurator.worldBuilder.simulationStep = configurator.simulationStep;
	printf("REACTIVE NAVIGATION\n");
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
	
	
