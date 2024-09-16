
#include "custom.h"

void forget(Configurator *c){
	//c->transitionSystem.clear();
}

Disturbance set_target(int &run, b2Transform start){
	Disturbance result;
	if (run%2!=0){
		result=Disturbance(PURSUE, start.p, start.q.GetAngle());
		run++;
	}
	return result;
}
void Configurator::done_that(vertexDescriptor& src, bool& plan_works, b2World& world, std::vector<vertexDescriptor> &plan_provisional){

}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal;
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	configurator.planning=1;
	char name[60];
	//sprintf(name, "targetless_forget");
	//get_Foldername(,name);
	configurator.setBenchmarking(1, "targetless_forget");

	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
		printf("debug = %i\n", atoi(argv[1]));
	}
	configurator.setSimulationStep(.5);
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
	
	
