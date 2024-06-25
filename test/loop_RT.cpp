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



int main(int argc, char** argv){
    //we imagine that we have executed a plan and then the robot is instructed to go back on its steps
	Disturbance target(PURSUE, b2Vec2(BOX2DRANGE, 0));    
    Task controlGoal;
    bool debug=1;
    Configurator conf(controlGoal, debug);
    conf.simulationStep=0.5;
    conf.setBenchmarking(1);
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    DataInterface2 di2(&ci);
    StepCallback cb(&conf);
    FakeLidar lidar(di2);
    TimerStep motors(cb);
    lidar.startms(200);
    motors.startms(100);
    conf.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(300000)); //simulates lidar
    lidar.stop();
    motors.stop();
    
    conf.stop();
}