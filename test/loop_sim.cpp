#include "callbacks.h"



class DataInterface2 {
    ConfiguratorInterface *ci;
    int iteration=0;
public:

char * folder;
    DataInterface2(ConfiguratorInterface* _ci): ci(_ci){}

	bool newScanAvail(){ //uncomment sections to write x and y to files		
        ci->data2fp= {Pointf(-1.0, -1.0)};
        ci->ready=1;
        iteration++;
        printf("iteration=%i\n", iteration);
        return true;
	}
};

class FakeLidar: public CppTimer{
    DataInterface2 di;
    public:
    FakeLidar(DataInterface2 & _di): di(_di){}
    void timerEvent(){
        if (!di.newScanAvail()){
            this->stop();
        }
    }
};


void Configurator::done_that(vertexDescriptor& src, bool& plan_works, b2World& world, std::vector<vertexDescriptor> &plan_provisional){

}

Disturbance set_target(int& run, b2Transform start){

}

void forget(Configurator* c){}


int main(int argc, char** argv){
    //we imagine that we have executed a plan and then the robot is instructed to go back on its steps
	Disturbance target(PURSUE, b2Vec2(BOX2DRANGE, 0));    
    Task controlGoal;
    bool debug=1;
    Configurator conf(controlGoal, debug);
    conf.simulationStep=0.5;
    conf.timerOff=1;
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    DataInterface2 di2(&ci);
    StepCallback cb(&conf);
    conf.running=1;           
    do{
        cb.step();
        conf.data2fp=ci.data2fp;
        conf.Spawner();
        cb.step();

    }while(di2.newScanAvail());
}