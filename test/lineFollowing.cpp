#include "Box2D/Box2D.h"
#include "../src/configurator.h"
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
    Configurator::getVelo
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