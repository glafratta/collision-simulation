#include "custom.h"

void forget(Configurator *c){}

void Configurator::done_that(vertexDescriptor& src, bool & plan_works, b2World & world, std::vector<vertexDescriptor>& plan_provisional){
		//std::pair<bool, vertexDescriptor> been(false, TransitionSystem::null_vertex());
		//was ve instead of src
		std::vector <vertexDescriptor> options_src;
		bool fin=controlGoal.checkEnded(transitionSystem[src], UNDEFINED, true).ended;
		printf("finished=%i, plan=%i, step=%i\n", fin, !planVertices.empty(), currentTask.motorStep);
		if ( !fin && planVertices.empty()&& currentTask.motorStep==0){
		//printf("is target=%i, task ended = %i\n", target.getAffIndex()==PURSUE, fin);
			printf("NOW CHECKING FOR MATCHES\n");
			std::vector <BodyFeatures> b_features=worldBuilder.getFeatures(data2fp, b2Transform(b2Vec2(0,0), b2Rot(0)), currentTask.direction, BOX2DRANGE);
			//Disturbance where=controlGoal.disturbance;
			if (!b_features.empty()){
				printf("there is disturbance\n");
				State s_temp;
				s_temp.disturbance= Disturbance(b_features[0]); //assumes 1 item length
				bool closest_match=1;
				findMatch(s_temp,transitionSystem, transitionSystem[movingEdge.m_source].ID, UNDEFINED, StateMatcher::DISTURBANCE, &options_src);
				//FIND STATE WHICH matches the relationship with the disturbance
			}
			//been= been_there(transitionSystem, where); 
		}
		else{
			plan_works=true;//temporary to simplify things
		}
		//printf("checked been = %i\n", been.first);
		//std::vector <std::pair <vertexDescriptor, vertexDescriptor>> toRemove;

		//std::vector <vertexDescriptor> plan_provisional=planVertices;
	//	if (been.first){
		//	printf("provisional plan\n");
		for (auto o:options_src){
			printf("FOUND MATCH WITH %i!", int(o));
			plan_provisional=planner(transitionSystem, o); //been.second, been.first
			vertexDescriptor end =*(plan_provisional.rbegin().base()-1);
			if (controlGoal.checkEnded(transitionSystem[end]).ended && checkPlan(world, plan_provisional, transitionSystem)){
				plan_works=true;
				break;
			}
		}

}


Disturbance set_target(int& run, b2Transform start){

}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	configurator.planning =1;
	char name[60];
	//sprintf(name, "Dmatch_target");
	//get_Foldername(,name);
	//printf("foldername = %s\n", name);
	configurator.setBenchmarking(1, "Dmatch_target");
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
	
	
