//#include "Box2D/Box2D.h"
#include "src/configurator.h"
#include "a1lidarrpi.h"
#include "alphabot.h"
#include "CppTimer.h"
#include <stdio.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#define _USE_MATH_DEFINES

class LidarInterface : public A1Lidar::DataInterface{
ConfiguratorInterface * ci;
public: 
    int mapCount =0;

    LidarInterface(ConfiguratorInterface * _ci): ci(_ci){}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
	    mapCount++;
		//std::vector <Point> current;
		Point p, p2f;
		FILE *f;
		char name[256];
		sprintf(name,"/tmp/map%04i.dat", mapCount);
		printf("%s\n", name);
		f=fopen(name, "w");
		for (A1LidarData &data:data){
			if (data.valid&& data.r <LIDAR_RANGE){
				//DATA IS ROUNDED AND DUPLICATES ARE ELIMINATED
				float x = round(data.x*1000)/1000;
				float y = round(data.y*1000)/1000;
				float x2 = round(data.x*100)/100;
				float y2 = round(data.y*100)/100;
				p= (Point(x, y));
				p2f=Point(x2, y2);
				//if (p1!= p0){
					ci->data.insert(p);
				//}
				//if (p1B2D!= p0B2D){ //SEPARATE VECTOR FOR BOX2D
					ci->data2fp.insert(p2f);
					if (ci->debugOn){
						fprintf(f, "%.2f\t%.2f\n", p2f.x, p2f.y);
					}
				//}
				// p0= p1;
				// p0B2D=p1B2D;
            }
		}
			fclose(f);
		//c->NewScan(current);
		

	}


};

class Callback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
    int iteration=0;
    int confIteration=0;
    Configurator * c;
    float L,R;

public:

Callback(Configurator *conf): c(conf){
}
void step( AlphaBot &motors){
	L= (c->getTask()->getAction().getLWheelSpeed());
	R = (c->getTask()->getAction().getRWheelSpeed());
    motors.setRightWheelSpeed(R); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(L);
	printf("step: R=%f\tL=%f, conf iteration = %i\n", R, L, c->getIteration());
    iteration++;
}
};



int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Task desiredTask;
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(desiredTask);
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
	}
	if (argc>2){
		configurator.planning= atoi(argv[2]);
	}
	printf("debug on = %i, planning on = %i\n", configurator.debugOn, configurator.planning);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface);
	Callback cb(&configurator);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	lidar.start();
	motors.start();
	configurator.start();
	do {
	} while (!getchar());
	configurator.stop();
	motors.stop();
	lidar.stop();

}
	
	
