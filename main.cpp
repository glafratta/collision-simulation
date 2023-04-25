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



//the environment is sampled by the LIDAR every .2 seconds

class LidarInterface : public A1Lidar::DataInterface{
Configurator * c;

char folder[250];
public: 
    int mapCount =0;

    LidarInterface(Configurator * _c): c(_c){
		}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
	    mapCount++;
		std::vector <Point> current;
		Point p1, p0, p1B2D, p0B2D;
		FILE *f;
		char name[256];
		sprintf(name,"/tmp/map%04i.dat", mapCount);
		printf("%s", name);
		if (c->debugOn){
			f=fopen(name, "w");
		}
		for (A1LidarData &data:data){
			if (data.valid&& data.r <1.0){
				//DATA IS ROUNDED AND DUPLICATES ARE ELIMINATED
				float x = round(data.x*1000)/1000;
				float y = round(data.y*1000)/1000;
				float x2 = round(data.x*100)/100;
				float y2 = round(data.y*100)/100;
				p1= (Point(x, y));
				p1B2D=Point(x2, y2);
				if (p1!= p0){
					current.push_back(p1);
				}
				if (p1B2D!= p0B2D){
					c->currentBox2D.push_back(p1B2D);
					if (c->debugOn){
						fprintf(f, "%.2f\t%.2f\n", p1B2D.x, p1B2D.y);
					}
				}
				p0= p1;
				p0B2D=p1B2D;
            }
		}
		if (f !=NULL){
			fclose(f);
		}
		c->NewScan(current);
		

	}

	char * getFolder(){
		return folder;
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
	L= -(c->getDMP()->getAction().getRWheelSpeed());
	R = -(c->getDMP()->getAction().getLWheelSpeed());
    motors.setRightWheelSpeed(R); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(L);
	printf("step: R=%f\tL=%f, conf iteration = %i\n", -R, -L, c->getIteration());
    iteration++;
}
};



int main(int argc, char** argv) {
	//world setup with environment class
	A1Lidar lidar;
	AlphaBot motors;
    Primitive desiredDMP;
    Configurator configurator(desiredDMP);
    configurator.setReadMap("map");
	if (argc>2){
		configurator.debugOn= atoi(argv[2]);
	}
	if (argc>3){
		configurator.planning= atoi(argv[3]);
	}
	LidarInterface dataInterface(&configurator);
	Callback cb(&configurator);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	lidar.start();
	motors.start();

	do {
	} while (!getchar());


	motors.stop();
	lidar.stop();

}
	
	
