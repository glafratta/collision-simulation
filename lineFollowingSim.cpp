#include "Box2D/Box2D.h"
#include "src/configurator.h"
#include <thread>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip> 
#include <sstream> //for writing string into file, for checking, std::ostringstream
#include <ncurses.h>
#include <ctime>
#include <dirent.h>
#include <filesystem>
#include "CppTimer.h"


// b2Vec2 estimateDisplacementFromWheels(float max, float L, float R, float d, float t=0.2){
//     b2Vec2 vel;
//     	//find angular velocity
// 	float angVel = max *(L-R)/d; //rad/s
// 	//find absolute speed
// 	float speed = max * (L+R)/2; //distance covered
// 	//find velocity vector: this is per second so no need to change this; but it is the instantaneous velocity
// 	//vel= b2Vec2(speed*sin(angVel), speed*(-cos(angVel)));
//     //from the derivation
//     vel.x = (d/2)* sin(d*t/(L-R))
// 	return vel;
// }
float getHypothenuse(float x, float y){
    float d = sqrt(x*x + y*y);
    return d;
}


class DataInterface {
public:
int mapCount = 0;
FILE * dumpPath;
//for debugging
FILE * speed;
Configurator * c;
char * folder;
    DataInterface(Configurator * _c){ //reads data from folder
		c = _c;
        	dumpPath = fopen("/tmp/dumpPath.txt", "w");
            fclose(dumpPath);
            speed = fopen("/tmp/speed.txt", "w");
            fclose(speed);
		// std::filesystem::path folderPath(_folder);
		// if (exists(folderPath)){
		// 	if (is_directory(folderPath)){
		// 		folder = _folder;
        //         c->folder = _folder;
		// 	}
		// 	else{
		// 		printf("not a directory");
		// 	}
		// }
		// else{
		// 	printf("%s doesn't exist", _folder);
		// }


	}

	void newScanAvail(){ //uncomment sections to write x and y to files	
    auto now =std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli>diff= now - c->previousTimeScan; //in seconds
    c->timeElapsed=float(diff.count())/1000; //express in seconds	
    c->totalTime += c->timeElapsed;
    c->previousTimeScan=now; //update the time of sampling
    mapCount++;
    float x,y;
    //float totalTime =0;
    char filePath[256];
    char folderName[256];
    sprintf(folderName,"%s", c->folder);
    sprintf(filePath, "%s%s%04d.dat", folderName, c->getReadMap(),mapCount);
    printf("%s\n", filePath);
    std::ifstream map(filePath);
    std::vector <cv::Point2f> current;
    b2Vec2 coord;
    while (map>>coord.x>>coord.y){
        if (coord.Length()<=1.5){
            //  fprintf(map, "%.2ft%.2f\t%.2f\t%.2f\n", data.x, data.y, data.r, data.phi);
            current.push_back(cv::Point2f(coord.x, coord.y));
        }
    }
    printf("made current\n");
    map.close();
    printf("closed map file\n");
    c->addIteration();
    printf("added iteration : %i\n", c->getIteration());
    b2Vec2 vel = c->GetRealVelocity(current).vector;
    printf("vel = %f, %f\n", vel.x, vel.y);
    c->desiredState.setRecordedVelocity(vel);

	printf("time elapsed between newscans = %f ms\n", c->timeElapsed);
    //printf("velocity: %f, %f\n",c->desiredState.getRecordedVelocity().x, c->desiredState.getRecordedVelocity().y);
    speed = fopen("/tmp/speed.txt", "a+");
    fprintf(speed, "%f\t%f\t%f\n",c->desiredState.getRecordedVelocity().x,c->desiredState.getRecordedVelocity().y, c->desiredState.getRecordedVelocity().Length()); //velocity not displacement i checked!!
    fclose(speed);
    //printf("configurator iteration as in lidar: %i\n", c->getIteration());

    c->updateAbsPos(vel);
	dumpPath = fopen("/tmp/dumpPath.txt", "a+");
	fprintf(dumpPath, "%f\t%f\n", c->getAbsPos().x, c->getAbsPos().y);
	fclose(dumpPath);
	}
};

void Configurator::controller(){ //this needs to be pasted in the straight line because it has to be different when the path is constantly recalculated
//FIND ERROR
b2Vec2 desiredPosition, nextPosition, recordedPosition; //target for genearting a corrective trajectory
State * state;
if (plan.size()==0){
    state = &desiredState;
}
else if (plan.size()>0){
    //state = &plan[0];
	leftWheelSpeed = plan[0].getTrajectory().getLWheelSpeed();
	rightWheelSpeed = plan[0].getTrajectory().getRWheelSpeed();
	return;
}
double angleError=0;
double distanceError=0;

printf("iteration in controller: %i\n", iteration);

if (iteration > 0){
    float x,y, t;
    t=0; //discrete time
    char name[50];
    sprintf(name, "%s", fileNameBuffer); //
    //printf("%s\n", name);
    std::ifstream file(name);

    while (file>>x>>y){ 
        t= t+ 1.0f/60.0f;
        if(totalTime<t && t<=(totalTime+1/60.f)){ 
            desiredPosition = b2Vec2(x,y);
            //printf("desired position out of file: %f, %f\t time recorded as: %f, timeElapsed: %f\n", x, y,t, timeElapsed);
            break;
        }
        // else {
        //     leftWheelSpeed = 0;
        //     rightWheelSpeed = 0;
        // }

    }

    file.close();
    //desiredVelocity =b2Vec2(desiredPosition.x/timeElapsed, desiredPosition.y/timeElapsed);

    //recordedPosition = {absPosition.x, absPosition.y}; //BUG: is the velocity being extracted from the current state?

	recordedPosition = b2Vec2(state->getRecordedVelocity().x*timeElapsed, state->getRecordedVelocity().y*timeElapsed);

    //printf("recordedpos = %f, %f\n", recordedPosition.x, recordedPosition.y);
    //float desiredAngle = atan2(recordedPosition.y)
   // angleError = atan2(desiredPosition.x, desiredPosition.y)-atan2(recordedPosition.x, recordedPosition.y); //flag
	float desiredAngle, recordedAngle; 
	if (desiredPosition.y ==0 && desiredPosition.x ==0){
		desiredAngle =0;
	}
	else{
		desiredAngle= atan(desiredPosition.y/desiredPosition.x);
	}
	if (recordedPosition.y ==0 && recordedPosition.x ==0){
		recordedAngle =0;
	}
	else{
		recordedAngle= atan(recordedPosition.y/recordedPosition.x);
	}
	
    angleError = desiredAngle - recordedAngle; //flag    
    //normalise error
    double maxError = M_PI_2;
    angleError /= maxError;
	
	state->accumulatedError += angleError;



    //printf("desired angle: %f pi\t recorded angle: %f pi\n", atan2(desiredPosition.y, desiredPosition.x)/M_PI, atan2(state->getRecordedVelocity().y, state->getRecordedVelocity().x)/M_PI);
    //printf("desired position = %f, %f\trecorded position: %f, %f\n", desiredPosition.x, desiredPosition.y, recordedPosition.x, recordedPosition.y);
    //distanceError = desiredPosition.Length() - absPosition.Length();
    //printf("distanceError = %f\n", distanceError);
    }

//    // if (angleError>0){
//         leftWheelSpeed = state->getTrajectory().getLWheelSpeed() + angleError*gain; //if angle is larger than 0 means thta the robot is going to R, slow L wheel
//         printf("initial L wheel speed: %f\n", state->getTrajectory().getLWheelSpeed());
//     //}
//     //else if (angleError<0){
//         rightWheelSpeed = state->getTrajectory().getRWheelSpeed()- angleError *gain;  //viceversa, sign is +ve because the angle error is neg
//         printf("initial R wheel speed: %f\n", state->getTrajectory().getRWheelSpeed());
//             //}

//-ve angle means to the right, so bd e=d-r, +ve angle error means going too right, so need to slow L and speed R

//angle error -ve: means that the robot went to far to the left. L - (-r)= increasing L, R+ (-r)= decreasing R so going Right
//angle error +ve: means that robot went too far to the right because r<0, d-(-r)= +ve; L-(+r)= decreasing L, R+(+r)= increasing R so going L


leftWheelSpeed -= angleError*gain+ distanceError*gain;  //og angle was +angle
rightWheelSpeed += angleError *gain + distanceError*gain; //og was - angle

float deltaV = angleError*gain;
dumpDeltaV = fopen("/tmp/deltaV.txt", "a");
fprintf(dumpDeltaV,"angleError =%f\n", angleError);
fprintf(dumpDeltaV, "angle error*%f = %f\n", gain, deltaV);
if (leftWheelSpeed>1.0){
    leftWheelSpeed=1.0;
}
if (rightWheelSpeed>1.0){
    rightWheelSpeed=1;
}
if (leftWheelSpeed<(-1.0)){
    leftWheelSpeed=-1;
}
if (rightWheelSpeed<(-1.0)){
    rightWheelSpeed=1;

}
fprintf(dumpDeltaV, "Right = %f, Left = %f\n", rightWheelSpeed, leftWheelSpeed);
fclose(dumpDeltaV);

}



class StepCallback{
    float L,R;
    Configurator * c;
    int iteration;
    FILE *dump;
    FILE * control;
public:
    StepCallback(Configurator * c): c(c){
         dump = fopen("/tmp/whereRobotThinkGo.txt", "wt");
         fclose(dump);
         control = fopen("/tmp/control.txt", "wt");
         fclose(control);
    }
    void step(){
        dump = fopen("/tmp/whereRobotThinkGo.txt", "a");
        control = fopen("/tmp/control.txt", "a");
        iteration++;
        c->controller();
        L=c->leftWheelSpeed;
        R= c->rightWheelSpeed;
        printf("R= %f, L = %f\n,", R, L);
        b2Vec2 displacement(0, 0);
        if (c->timeElapsed>0){
            displacement= c->estimateDisplacementFromWheels(); // m/s
        }
        fprintf(control, "%f\t%f\n",displacement.x, displacement.y);
        fprintf(dump, "%f\t%f\n",c->getAbsPos().x+displacement.x, c->getAbsPos().y+displacement.y);
        fclose(dump);
        fclose(control);

    }

};



//FOR THREAD DEBUGGING

class TimerStep: public CppTimer{
    StepCallback sc;
    public:
    TimerStep(StepCallback & _step): sc(_step){}
    void timerEvent(){
        sc.step();
    }
};

class TimerDI: public CppTimer{
    DataInterface di;
    public:
    TimerDI(DataInterface & _di): di(_di){}
    void timerEvent(){
        di.newScanAvail();
    }
};

//END THREAD DEBUGGING

int main(int argc, char** argv) {
    //BENCHMARK
//	auto begin = std::chrono::high_resolution_clock::now();


    //HOW MANY FILES IN DIRECTORY
    // DIR *dp;
    // int howManyFiles = 0;
    // struct dirent *ep;     
    // dp = opendir(argv[2]);

    // if (dp != NULL)
    // {
    //     while (ep = readdir(dp)){
    //         if (ep->d_name)
    //         howManyFiles++;
    //     }
    //     closedir(dp);
    // }
    // else{
    //     printf("Couldn't open the directory%s", argv[2]);
    // }

    FILE * file;
    char filePrefix[5];
    sprintf(filePrefix, "map");
    int fileCount =1;
    bool fileExists = true;
    while (fileExists){
        char fileName[250];
        sprintf(fileName, "%s%s%04i.dat", argv[2], filePrefix, fileCount);
        file = fopen(fileName, "r");
        if (!file){
            fileExists = 0;
            break;
        }
        fclose(file);
        fileCount++;
    }
    printf("%i files\n", fileCount);

    //CREATE A FOLDER TO STORE THE RESULTS FOR THIS RUN

    // //TODAYS DATE AND TIME
    // time_t now =time(0);
    // tm *ltm = localtime(&now);
    // int y,m,d;
    // y=ltm->tm_year-100;
    // m = ltm->tm_mon +1;
    // d=ltm->tm_mday;
    
    // //PICK DIRECTORY NAME
    // char dirName[256];
    // int copynumber=1;
    // while (true){
    //     DIR *dir; 
    //     char tmp[256];
    //     sprintf(tmp, "LoRNoTimer%02i-%02i-%02i_%i",d, m, y, copynumber);
    //     dir = opendir(tmp);
    //     if (dir ==NULL){
    //         sprintf(dirName, "%s", tmp);
    //         std::filesystem::create_directory(dirName);
    //         break;
    //     } //if the directory DOESN'T return a void pointer (= exists already)
    //     else if (dir!=NULL){
    //         copynumber++;                                                           //increase the number at the end of the name
    //         closedir(dir);
    //     }

    // }
                                //should exit when dir is null pointer

    //DATA INTERCFACE
    State desiredState;
    Configurator conf(desiredState);
    conf.setNameBuffer(argv[1]);
    if (argc > 3){
        conf.filterOn = argv[3];
    }
    conf.setReadMap("map");
    //char folderName[250];
    //sprintf(folderName, "test/141022_nostraight/");
    conf.setFolder(argv[2]);
    StepCallback cb(&conf);
    DataInterface dataInterface(&conf);
    TimerDI lidar(dataInterface);
    TimerStep motors(cb);
    lidar.startms(200);
    motors.startms(100);
                //iterate through files
    //b2Vec2 velocity = {0,0};
    // for (int j=0; j<i/3;j++){
    //     dataInterface.newScanAvail();
    //     //c.NewScan();
    //     cb.step();

    // }
    std::this_thread::sleep_for(std::chrono::milliseconds(200*fileCount)); //simulates lidar


    printf("total time = %.4f", conf.totalTime);
    lidar.stop();
    motors.stop();
    // FINISH BENCHMARKING
    // auto end = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    // printf("scanned in %f seconds\n", elapsed.count()* 1e-9);




}


