#include "Box2D/Box2D.h"
#include "src/configurator.h"
#include "a1lidarrpi.h"
#include "alphabot.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

class DataInterface : public A1Lidar::DataInterface{ //get data from lidar, not passing it into Box2d
public: 

    DataInterface(Configurator & c): box2d(_box2d){};
    int mapCount =0;
	void newScanAvail(float, A1A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files	
        mapCount++;
        char name_r[50];
		sprintf(name_r, "/tmp/scan%04i.txt", mapCount);
		FILE * dump = fopen(name_r, "w+");
		for (A1LidarData &data:data){
			if (data.valid){
		        fprintf(dump, "%2f\t%2f\t%2f\t%2f\t%2f\n", data.x, data.y, data.r, data.phi);
                c.current.push_back(cv::Point2f(data.x, data.y));
            }
		}
    fclose(dump);
    desiredState.setRecordedVelocity(c.getVelocity().displacement);
	}
};

class Callback:public Alphabot:StepCallback{
    Configurator c;
public:

Callback(Configurator &_conf): c(_conf){}

void step( Alphabot &motors){
    c.controller();
    motors.SetRightWheelSpeed(c.rightWheelSpeed);
    motors.SetRightWheelSpeed(c.leftWheelSpeed);

}
};

void Configurator::controller(){
//FIND ERROR
b2Vec2 desiredPosition, nextPosition, recordedPosition; //target for genearting a corrective trajectory

float angleError=0;
float distanceError=0;
if (iteration >=1){
    float x,y, t;
    t=0; //discrete time
    char name[50];
    sprintf(name, "%s.txt", fileNameBuffer); //
    //printf("%s\n", name);
    std::ifstream file(name);

    while (file>>x>>y){
        t= t+ 1.0f/60.0f;
        if(totalTime<t && t<=(totalTime+1/60.f)){ 
            desiredPosition = b2Vec2(x,y);
            printf("desired position out of file: %f, %f\t time recorded as: %f, timeElapsed: %f\n", x, y,t, timeElapsed);
            break;
        }
    }
    file.close();
    desiredVelocity =b2Vec2(desiredPosition.x/timeElapsed, desiredPosition.y/timeElapsed);
    recordedPosition = {state->getRecordedVelocity().x*timeElapsed, state->getRecordedVelocity().y*timeElapsed};
    //float desiredAngle = atan2(recordedPosition.y)
    angleError = atan2(desiredPosition.x, desiredPosition.y)-atan2(recordedPosition.x, recordedPosition.y); //flag
    printf("desired position = %f, %f\trecorded position: %f, %f\n", desiredPosition.x, desiredPosition.y, recordedPosition.x, recordedPosition.y);
    printf("angleError =%f\n", angleError);
    distanceError = desiredPosition.Length() - recordedPosition.Length();
    printf("distanceError = %f\n", distanceError);
    }

leftWheelSpeed = state->getTrajectory().getLWheelSpeed() + angleError*gain+ distanceError*gain;
rightWheelSpeed = state->getTrajectory().getRWheelSpeed()- angleError *gain + distanceError*gain; 
}
 
int main(){
    A1Lidar lidar;
    AlphaBot motors;
    Configurator conf;
    DataInterface di(conf);
    Callback cb(conf);
    lidar.registerInterface(&di);
    motors.registerStepCallback(&cb);
    do {

    }
    while (!getchar());
    printf("how many ")




}