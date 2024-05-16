#include "callbacks.h"
// #include <thread>
// #include "configurator.h"
// #include <unistd.h>
// #include <time.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <iomanip> 
// #include <sstream> //for writing string into file, for checking, std::ostringstream
// #include <ncurses.h>
// #include <ctime>
// #include <dirent.h>
// #include <filesystem>
// #include "CppTimer.h"

// void printGraph(TransitionSystem& g){
//     boost::print_graph(g);
// }

// Remember remember;
// Visited visited;

// template <typename Predicate> 
// void printEdges(TransitionSystem& g, Predicate p){
//     auto es = boost::edges(g);
//     for (auto ei=es.first; ei!=es.second;ei++){
//         if (!p(*ei)){
//             printf("%i->%i, direction=%i,probability=%f\n", (*ei).m_source, (*ei).m_target, g[(*ei).m_target].direction, g[*ei].probability);
//         }
//     }
// }

// void print_forget(TransitionSystem& g){
//     Remember p;
//     printEdges(g, p);
// }

// float print_belowP(TransitionSystem& g, float p){
//     auto es = boost::edges(g);
//     float ct=0;
//     for (auto ei=es.first; ei!=es.second;ei++){
//         if (g[*ei].probability<p){
//             ct++;
//             //printf("%i->%i, direction=%i,probability=%f\n", (*ei).m_source, (*ei).m_target, g[(*ei).m_target].direction, g[*ei].probability);
//         }
//     }
//     return ct/g.m_vertices.size();
// }

// void getVisited(TransitionSystem& g, vertexDescriptor cv){
//     auto es = boost::edges(g);
//     float ct=0;
//     for (auto ei=es.first; ei!=es.second;ei++){
//         if ((g[(*ei).m_source].visited()|| (*ei).m_source==0 || (*ei).m_source==cv)& g[(*ei).m_target].visited()){
//             ct++;
//             //printf("%i->%i, direction=%i,probability=%f\n", (*ei).m_source, (*ei).m_target, g[(*ei).m_target].direction, g[*ei].probability);
//         }
//     }
// }

// class DataInterface {
// public:
// int iteration = 0;
// ConfiguratorInterface * ci;
// //CoordinateContainer data;
// // CoordinateContainer data2fp;
// char * folder;
//     DataInterface(ConfiguratorInterface * _ci): ci(_ci){}

// 	bool newScanAvail(){ //uncomment sections to write x and y to files		
//         iteration++;
//     	ci->ready=0;
// 		ci->data.clear();
// 		ci->data2fp.clear();
// 		char filePath[256];
//         char folderName[256];
//         sprintf(folderName,"%s", folder);
// 		sprintf(filePath, "%smap%04d.dat", folderName, iteration);
//         printf("%s\n", filePath);
//         FILE *f;
//         if (!(f=fopen(filePath, "r"))){
//             return false;
//         }
//         else {
//             fclose(f);
//         }
//         // std::vector<Pointfff > data;
// 		std::ifstream file(filePath);
//         float x, y, x2, y2;
//         Pointf  p1, p2_1;
// 		while (file>>x>>y){
//             x = round(x*1000)/1000;
// 			y = round(y*1000)/1000;
//             x2 = round(x*100)/100;
// 			y2 = round(y*100)/100;
//             Pointf  p(x,y);
//             Pointf  p2(x2,y2);
//             ci->data.insert(p);
//             ci->data2fp.insert(p2);
// 		}
// 		file.close();
//         ci->setReady(1);
//         ci->iteration++;
//         return true;
		


// 	}
// };

// class StepCallback{
//     float L=0,R=0;
//     Configurator * c;
//     int ogStep=0;
// public:
//     StepCallback(Configurator * _c): c(_c){}
//     void step(){
//     ExecutionError ee =c->trackTaskExecution(*c->getTask());
//     c->getTask()->controller(ee.theta(), MOTOR_CALLBACK);
// 	//c->controlGoal.trackDisturbance(c->controlGoal.disturbance, c->getTask()->getAction(), error);
// 	c->changeTask(c->getTask()->change,  ogStep);
//     if (c->debugOn){
// 		printf("current vertex= %i\n", c->currentVertex);
// 	}
//     L=c->getTask()->getAction().getLWheelSpeed();
//     R= c->getTask()->getAction().getRWheelSpeed();
    

//     }
// };

// std::vector <BodyFeatures> WorldBuilder::processData(CoordinateContainer points){
//     int count =0;
// 	//buildType =2;
//     std::vector <BodyFeatures> result;
//     for (Pointf p: points){
//         if (count%2==0){
//             BodyFeatures feature;
//             feature.pose.p = getb2Vec2(p); 
//             result.push_back(feature);  
//         }
//         count++;
//     }
//     return result;
// }


// //FOR THREAD DEBUGGING

// class TimerStep: public CppTimer{
//     StepCallback sc;
//     public:
//     TimerStep(StepCallback & _step): sc(_step){}
//     void timerEvent(){
//         sc.step();
//     }
// };

// class TimerDI: public CppTimer{
//     DataInterface di;
//     public:
//     TimerDI(DataInterface & _di): di(_di){}
//     void timerEvent(){
//         if (!di.newScanAvail()){
//             this->stop();
//         }
//     }
// };

// float Configurator::taskLateralError(){
// 	b2Transform velocity;
//     float error;
// 	if (currentTask.action.getOmega()==0){
// 		float dataRange=0.25;
// 	 	velocity= sensorProc->affineTransEstimate(std::vector <Pointf>(data.begin(), data.end()), currentTask.action, timeElapsed, dataRange);
// 		//GetRealVelocity(current, previous); //closed loop, sensor feedback for velocity
// 	 }
// 	else{
// 		velocity = b2Transform(currentTask.getAction().getTransform()); //open loop
// 	}
// 	currentTask.action.setRec(SignedVectorLength(velocity.p), velocity.q.GetAngle());
//     error = atan2(velocity.y,velocity.x)- currentTask.action.getOmega();
//     ExecutionError exErr;
//     if (auto it=errorMap.find(transitionSystem[currentVertex].ID); it!=errorMap.end()){
//         exErr= it->second;
//     }
//     //exErr.setTheta(error);
//     //errorMap.insert_or_assign(g[currentVertex].ID, exErr);
//     return error;

// }

//END THREAD DEBUGGING

//argv: 1. directory to open 2. timeoff (0=timeron) 3. planning on 4. debug on

int main(int argc, char** argv) {
    //BENCHMARK
//	auto begin = std::chrono::high_resolution_clock::now();


    //HOW MANY FILES IN DIRECTORY
    DIR *dp;
    int i = 0;
    struct dirent *ep;     
    dp = opendir(argv[1]);

    if (dp != NULL)
    {
        while (ep = readdir(dp)){
            i++;
        }
        closedir(dp);
    }
    else{
        printf("Couldn't open the directory%s", argv[1]);
    }


    char filePrefix[5];
    sprintf(filePrefix, "map");
    int fileCount =1;
    char file[256];
    FILE *f;
    while ( sprintf(file, "%s%s%04i.dat", argv[1], filePrefix, fileCount)>-1 &&(f = fopen(file, "r"))!=NULL){   
        fileCount++;
        memset(file, 0, sizeof(file));
        fclose(f);
    }
    printf("\n%i files\n", fileCount);

    //DATA INTERCFACE

    bool timerOff=atoi(argv[2]);
	Disturbance target(PURSUE, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
    Configurator configurator(controlGoal, 1, timerOff);
    configurator.planning=1;
    if (argc>3){
		configurator.setSimulationStep(atof(argv[3]));
    }
    ConfiguratorInterface configuratorInterface;
    configurator.registerInterface(&configuratorInterface);
    DataInterface dataInterface(&configuratorInterface); 
    dataInterface.folder = argv[1];
    StepCallback cb(&configurator);

    if (!timerOff){
        TimerDI lidar(dataInterface);
        TimerStep motors(cb);
        lidar.startms(200);
        motors.startms(100);
        configurator.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(200*fileCount)); //simulates lidar
        lidar.stop();
        motors.stop();
        configurator.stop();
    }
    else if (timerOff){
                // b2Vec2 velocity = {0,0};
        //for (int i=0; i<40;i++){
        while  (dataInterface.newScanAvail()){
            configurator.running=1;           
            //configurator.Spawner(configuratorInterface.data, configuratorInterface.data2fp);
            if (configuratorInterface.isReady()){
                if (configuratorInterface.data2fp != configurator.currentBox2D){
                    //printf("confINt data size = %i\n", configuratorInterface.data.size());
                    configurator.Spawner(configuratorInterface.data, configuratorInterface.data2fp);
                    // configuratorInterface.data.clear();
                    // configuratorInterface.data2fp.clear();
                    // configuratorInterface.ready=0;
			}
            }
            cb.step();
            cb.step();
        }
        configurator.running=0;
    }



}