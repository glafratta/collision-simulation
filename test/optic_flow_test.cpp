#include "custom.h"
#include "CppTimer.h"

class TimerStep: public CppTimer{
    StepCallback sc;
    public:
    TimerStep(StepCallback & _step): sc(_step){}
    void timerEvent(){
        sc.step();
    }
};

class StepCallback{
    float L=0,R=0;
    Configurator * c;
    int ogStep=0;
public:
    StepCallback(Configurator * _c): c(_c){}
    void step(){
        float task_rotation_error=c->taskRotationError();
		
    }
};


void Configurator::done_that(vertexDescriptor& src, bool& plan_works, b2World& world, std::vector<vertexDescriptor> &plan_provisional){

}

Disturbance set_target(int& run, b2Transform start){

}

void forget(Configurator* c){}

float Configurator::taskRotationError(){
    float error=0, theta_left=0, theta_right=0; //difference in rotation between left and right visual field
    //get Left
    if (visual_field.empty()){
        return error;
    }
    // cv::Mat left_vf= imgProc.cropLeft(ci->visual_field);
    // //get Right
    // cv::Mat right_vf= imgProc.cropRight(ci->visual_field);
    // //get optic flow
    // b2Vec2 left_optic_flow= imgProc.opticFlow(left_vf, corners_left, previous_grey_left);
    // b2Vec2 right_optic_flow= imgProc.opticFlow(right_vf, corners_right, previous_grey_right);
	// printf("L optic flow=%f, %f\t R optic flow= %f, %f\n", left_optic_flow.x, left_optic_flow.y, right_optic_flow.x, right_optic_flow.x);
    // theta_left=atan(left_optic_flow.y/left_optic_flow.x);
    // theta_right =atan(right_optic_flow.y/right_optic_flow.x);
    // if ((theta_left<0 & theta_right>0)||(theta_left>0 & theta_right<0)){
    //     printf("optic flows diverge\n");
    // }
    // //get difference between optic flows = error
    // error = theta_left-theta_right;
	b2Vec2 optic_flow=imgProc.opticFlow(ci->visual_field, imgProc.corners(), imgProc.previous());
	return -optic_flow.x;
}

int main(int argc, char** argv) {
	//A1Lidar lidar;
	//AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    //Task controlGoal(target, DEFAULT);
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	// configurator.numberOfM = THREE_M;
	// configurator.graphConstruction = A_STAR;
	// configurator.planning =1;
	// if (argc>1){
	// 	configurator.debugOn= atoi(argv[1]);
	// 	configuratorInterface.debugOn = atoi(argv[1]);
	// }
	// if (argc>2){
	// 	configurator.simulationStep = atof(argv[2]);
	// }
	//LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface);
	Callback cb(&configurator);
	//lidar.registerInterface(&dataInterface);
	//motors.registerStepCallback(&cb);
	configurator.start();
	//lidar.start();
	//motors.start();
	do {
	} while (!getchar());
	configurator.stop();
	//motors.stop();
	//lidar.stop();

}
	
	
