
#include "custom.h"

void forget(Configurator *c){}

Disturbance set_target(int& run, b2Transform start){
	Disturbance result;
	if (run%2==0){
		result=Disturbance(PURSUE, start.p, start.q.GetAngle());
		run++;
	}
	return result;
}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal;
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	configurator.planning=1;
	configurator.setBenchmarking(1);
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
		printf("debug = %i\n", atoi(argv[1]));
	}
	if (argc>2){
		configurator.simulationStep = atof(argv[2]);
	}
	printf("TARGETLESS PLANNING, hz=%f, turn f=%f yallaaaaa", HZ, TURN_FRICTION);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface);
	MotorCallback cb(&configurator);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	lidar.start();
	motors.start();
	configurator.start();
	do {
	} while (!getchar());
	configurator.stop();
	motors.stop();
	lidar.stop();

}
	
	
