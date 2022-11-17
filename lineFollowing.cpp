#include "Box2D/Box2D.h"
#include "src/configurator.h"
#include "a1lidarrpi.h"
#include "alphabot.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

//b2Vec2 estimateVelocityFromWheel(float max, float L, float R, float d){
//     	//find angular velocity
// 	float angVel = max *(L-R)/d; //rad/s
// 	//find absolute speed
// 	float speed = max * (L+R)/2; //distance covered
// 	//find velocity vector: this is per second so no need to change this; but it is the instantaneous velocity
// 	b2Vec2 vel(speed*sin(angVel), speed*cos(angVel));
// 	return vel;
// }

class LidarIF : public A1Lidar::DataInterface{ //get data from lidar, not passing it into Box2d
Configurator * c;
FILE * dumpPath;
//for debugging
FILE * speed;
FILE * time;
public: 
    int mapCount =0;

    LidarIF(Configurator * _c): c(_c){
            dumpPath = fopen("/tmp/dumpPath.txt", "w");
            fclose(dumpPath);
            speed = fopen("/tmp/speed.txt", "w");
            fclose(speed);
            time = fopen("/tmp/timeScan.txt", "w");
            fclose(time);
    }
	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files	
        auto now =std::chrono::high_resolution_clock::now();
	    std::chrono::duration<float, std::milli>diff= now - c->previousTimeScan; //in seconds
	    c->timeElapsed=float(diff.count())/1000; //express in seconds	
        c->totalTime += c->timeElapsed;
	    c->previousTimeScan=now; //update the time of sampling
        if (c->timeElapsed >.25){
            return;
        }
        time = fopen("/tmp/timeScan", "a+");
        fprintf(time, "%.4f", c->timeElapsed);
        fclose(time);
        mapCount++;
        char name_r[50];
		sprintf(name_r, "/tmp/%s%04i.dat", c->getReadMap(), mapCount);
		FILE * map = fopen(name_r, "wt");
        //c->current.clear();
        std::vector <cv::Point2f> current;
		for (A1LidarData &data:data){
			if (data.valid){
		        fprintf(map, "%.2f\t%.2f\n", data.x, data.y);
                current.push_back(cv::Point2f(data.x, data.y));
            }
		}
    fclose(map);
    c->addIteration();
    b2Vec2 vel = c->GetRealVelocity(current).vector;
    c->desiredState.setRecordedVelocity(vel);

	//printf("time elapsed between newscans = %f ms\n", c->timeElapsed);
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

class Callback:public AlphaBot::StepCallback{
    int iteration=0;
    int confIteration=0;
    Configurator * c;
    FILE * dump;
    FILE * control;

public:

Callback(Configurator *_conf): c(_conf){
    float L,R;
    Configurator * box2d;
    int iteration;
    FILE * dump;
    dump = fopen("/tmp/whereRobotThinkGo.txt", "w");
    fclose(dump);
    control = fopen("/tmp/control.txt", "wt");
    fclose(control);
}
void step( AlphaBot &motors){
    dump = fopen("/tmp/whereRobotThinkGo.txt", "a");
    control = fopen("/tmp/control.txt", "a");
    iteration++;
   if (confIteration != c->getIteration()){ //every odd iteration because data comes in at even iteration and it needs time for processing
        c->controller();
        motors.setRightWheelSpeed(-(c->leftWheelSpeed)); //temporary fix because motors on despacito are the wrong way around
        motors.setLeftWheelSpeed(-(c->rightWheelSpeed));
        confIteration = c->getIteration();
        //desiredVelocity = c->estimateVelocityFromWheels(.2, c->leftWheelSpeed, c->rightWheelSpeed, 0.08);
    printf("changed speed to R= %f, L= %f\n",c->rightWheelSpeed, c->leftWheelSpeed);
    if (-c->rightWheelSpeed > -c->leftWheelSpeed){
        printf("robot going left\n");
    }
    else if (-c->rightWheelSpeed < -c->leftWheelSpeed){
        printf("robot foing right\n");
    }

  }
    printf("callback iteration %i\n", iteration);
    //b2Vec2 plannedVelocity(0.0f, 0.0f);
    b2Vec2 displacement(0, 0);

        if (c->timeElapsed>0){
            // plannedVelocity.x = c->estimateDisplacementFromWheels().x /c->timeElapsed ; // m/s
            // plannedVelocity.y = c->estimateDisplacementFromWheels().y /c->timeElapsed; // m/s
            displacement = c->estimateDisplacementFromWheels();
        }
        // displacement.x =plannedVelocity.x*c->timeElapsed;
        // displacement.y =plannedVelocity.y*c->timeElapsed;
        fprintf(control, "%f\t%f\n",displacement.x, displacement.y);
        fprintf(dump, "%f\t%f\n",c->getAbsPos().x+displacement.x, c->getAbsPos().y+displacement.y);
        fclose(dump);
        fclose(control);

}
};

// void Configurator::controller(){
//FIND ERROR
// b2Vec2 desiredPosition, nextPosition, recordedPosition; //target for genearting a corrective trajectory
// recordedPosition = b2Vec2(0.0f, 0.0f);
// State * state;
// if (plan.size()==0){
//     state = &desiredState;
// }
// else if (plan.size()>0){
//     state = &plan[0];
// }
// double angleError=0;
// double distanceError=0;

// printf("iteration in controller: %i\n", iteration);

// if (iteration > 0){
//     float x,y, t;
//     t=0; //discrete time
//     char name[50];
//     sprintf(name, "%s", fileNameBuffer); //
//     //printf("%s\n", name);
//     std::ifstream file(name);

//     while (file>>x>>y){ 
//         t= t+ 1.0f/60.0f;
//         if(totalTime<t && t<=(totalTime+1/60.f)){ 
//             desiredPosition = b2Vec2(x,y);
//             printf("desired position out of file: %f, %f\t time recorded as: %f, timeElapsed: %f\n", x, y,t, timeElapsed);
//             break;
//         }
//         // else {
//         //     leftWheelSpeed = 0;
//         //     rightWheelSpeed = 0;
//         // }

//     }

//     file.close();
//     //desiredVelocity =b2Vec2(desiredPosition.x/timeElapsed, desiredPosition.y/timeElapsed);
//     recordedPosition = {state->getRecordedVelocity().x, state->getRecordedVelocity().y};
//     printf("recordedpos = %f, %f\n", recordedPosition.x, recordedPosition.y);
//     //float desiredAngle = atan2(recordedPosition.y)
//    // angleError = atan2(desiredPosition.x, desiredPosition.y)-atan2(recordedPosition.x, recordedPosition.y); //flag
//     angleError = atan2(desiredPosition.y, desiredPosition.x)-atan2(recordedPosition.y, recordedPosition.x); //flag    
//     //normalise error
//     double maxError = 2* M_PI;
//     angleError /= maxError;


//     printf("desired angle: %f pi\t recorded angle: %f pi\n", atan2(desiredPosition.y, desiredPosition.x)/M_PI, atan2(recordedPosition.y, recordedPosition.x)/M_PI);
//     //printf("desired position = %f, %f\trecorded position: %f, %f\n", desiredPosition.x, desiredPosition.y, recordedPosition.x, recordedPosition.y);
//     printf("angleError =%f\n", angleError);
//     distanceError = desiredPosition.Length() - recordedPosition.Length();
//     printf("distanceError = %f\n", distanceError);
//     }

// //    // if (angleError>0){
// //         leftWheelSpeed = state->getTrajectory().getLWheelSpeed() + angleError*gain; //if angle is larger than 0 means thta the robot is going to R, slow L wheel
// //         printf("initial L wheel speed: %f\n", state->getTrajectory().getLWheelSpeed());
// //     //}
// //     //else if (angleError<0){
// //         rightWheelSpeed = state->getTrajectory().getRWheelSpeed()- angleError *gain;  //viceversa, sign is +ve because the angle error is neg
// //         printf("initial R wheel speed: %f\n", state->getTrajectory().getRWheelSpeed());
// //             //}

// // leftWheelSpeed += distanceError*gain; //+ve distance error means too slow, -ve means too fast
// // rightWheelSpeed += distanceError*gain;

// leftWheelSpeed = state->getTrajectory().getLWheelSpeed() - angleError*gain+ distanceError*gain;  //og angle was +angle
// rightWheelSpeed = state->getTrajectory().getRWheelSpeed()+ angleError *gain + distanceError*gain; //og was - angle

// if (leftWheelSpeed>1.0){
//     leftWheelSpeed=1.0;
// }
// if (rightWheelSpeed>1.0){
//     rightWheelSpeed=1;
// }
// if (leftWheelSpeed<(-1.0)){
//     leftWheelSpeed=-1;
// }
// if (rightWheelSpeed<(-1.0)){
//     rightWheelSpeed=1;

// }

// }
 
int main(int argc, char** argv){
    if (argc <2){
        printf("what trajectory to follow?\n");
        return 0;
    }
    A1Lidar lidar;
    AlphaBot motors;
    //printf("made lidar and motors\n");
    Configurator conf;
    //printf("make conf\n");
    conf.setNameBuffer(argv[1]);
    conf.setReadMap("map");
    conf.setFolder("/tmp/");
    LidarIF di(&conf);
    Callback cb(&conf);
    lidar.registerInterface(&di);
    motors.registerStepCallback(&cb);
    lidar.start();
    motors.start();
    do {
    }
    while (!getchar());
    printf("how many iterations: %i in %f\n", di.mapCount, conf.totalTime);
    lidar.stop();
    motors.stop();




}