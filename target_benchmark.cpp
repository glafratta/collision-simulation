#include "custom.h"

Disturbance set_target(int& run, b2Transform pose){
	pose=b2Transform(b2Vec2(0,0), 0);
	Disturbance result=Disturbance(PURSUE, pose.p, pose.q.GetAngle());
	run++;
	return result;
}

void forget(Configurator* c){}


int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	configurator.setBenchmarking(1);
	configurator.planning =1;
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
		configurator.worldBuilder.debug = atoi(argv[1]);
	}
	if (argc>2){
		configurator.setSimulationStep(atof(argv[2]));
	}
	//printf("debug on = %i, planning on = %i\n", configurator.debugOn, configurator.planning);
	//printf("box2drange = %f\n", BOX2DRANGE);
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
	
	
