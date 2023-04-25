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
std::vector <Point> previous;

char folder[250];
public: 
    int mapCount =0;

    LidarInterface(Configurator * _c): c(_c){
		c->timeElapsed = 0.2;
		}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
	    mapCount++;
		std::vector <Point> current;
		Point p1, p0;
		for (A1LidarData &data:data){
			if (data.valid&& data.r <1.0){
				//DATA IS ROUNDED AND DUPLICATES ARE ELIMINATED
				float x = round(data.x*1000)/1000;
				float y = round(data.y*1000)/1000;
				p1= (Point(x, y));
				if (p1!= p0){
					current.push_back(p1);
				}
				
				p0= p1;
            }
		}
		//c->NewScan(current);
		c->addIteration();
		Configurator::getVelocityResult r =c->GetRealVelocity(current, previous);
		c->getDMP()->setRecordedVelocity(r.vector);
		printf("current = %i, previous = %i\n", current.size(), previous.size());
		printf("velocity = (%f, %f), angle = %f pi, r = %f\n", r.vector.x, r.vector.y, atan(r.vector.y/r.vector.x)/M_PI, r.vector.Length());
		
		previous = current;
		//current.clear();
		c->applyController(1, *c->getDMP());

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
	printf("argc = %i", argc);
	if (argc>1){
	desiredDMP.pGain = (std::stof(argv[2]));
	 	printf("%d\n", argv[2]);
	 }
	desiredDMP.pGain = 0.065;
	printf("pgain = %f\n", desiredDMP.pGain);
    Configurator configurator(desiredDMP);
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
	
	
