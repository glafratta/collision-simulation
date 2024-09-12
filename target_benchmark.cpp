#include "custom.h"

Disturbance set_target(int& run, b2Transform pose){
	pose=b2Transform(b2Vec2(0,0), 0);
	Disturbance result=Disturbance(PURSUE, pose.p, pose.q.GetAngle());
	run++;
	return result;
}

void forget(Configurator* c){}

void Configurator::done_that(vertexDescriptor& src, bool & plan_works, b2World & world, std::vector<vertexDescriptor>& plan_provisional){
		//std::pair<bool, vertexDescriptor> been(false, TransitionSystem::null_vertex());
		//was ve instead of src
		std::vector <vertexDescriptor> options_src;
		if (bool fin=controlGoal.checkEnded(transitionSystem[src], UNDEFINED, true).ended; fin && currentVertex!=movingVertex){
		//printf("is target=%i, task ended = %i\n", target.getAffIndex()==PURSUE, fin);
			std::vector <BodyFeatures> b_features=worldBuilder.getFeatures(data2fp, b2Transform(b2Vec2(0,0), b2Rot(0)), currentTask.direction, BOX2DRANGE);
			//Disturbance where=controlGoal.disturbance;
			if (!b_features.empty()){
				State s_temp;
				s_temp.disturbance= Disturbance(b_features[0]); //assumes 1 item length
				bool closest_match=1;
				findMatch(s_temp,transitionSystem, transitionSystem[movingEdge.m_source].ID, UNDEFINED, StateMatcher::DISTURBANCE, &options_src);
				//FIND STATE WHICH matches the relationship with the disturbance
			}
			//been= been_there(transitionSystem, where); 
		}
		//printf("checked been = %i\n", been.first);
		std::vector <std::pair <vertexDescriptor, vertexDescriptor>> toRemove;

		//std::vector <vertexDescriptor> plan_provisional=planVertices;
	//	if (been.first){
		//	printf("provisional plan\n");
		for (auto o:options_src){
			plan_provisional=planner(transitionSystem, o); //been.second, been.first
			vertexDescriptor end =*(plan_provisional.rbegin().base()-1);
			if (controlGoal.checkEnded(transitionSystem[end]).ended && checkPlan(world, plan_provisional, transitionSystem)){
				plan_works=true;
				break;
			}
		}

}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	char name[60];
	sprintf("target_%s", get_Foldername());
	configurator.setBenchmarking(1, name);
	configurator.planning =1;
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
		configurator.worldBuilder.debug = atoi(argv[1]);
	}
	configurator.setSimulationStep(.27);
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
	
	
