//#include "Box2D/Box2D.h"
#include "a1lidarrpi.h"
#include "alphabot.h"
#include "configurator.h"
#include "CppTimer.h"
#include <stdio.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#define _USE_MATH_DEFINES


std::vector <BodyFeatures> WorldBuilder::processData(CoordinateContainer points){
    std::vector <BodyFeatures> result;
	buildType=1;
    for (Point p: points){
            BodyFeatures feature;
            feature.pose.p = p.getb2Vec2(); 
            result.push_back(feature);  
    }
    return result;
}

class LidarInterface : public A1Lidar::DataInterface{
ConfiguratorInterface * ci;
// CoordinateContainer coordinates = {};
// CoordinateContainer coordinates2fp = {};
public: 
    int mapCount =0;

    LidarInterface(ConfiguratorInterface * _ci): ci(_ci){}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
	    //ci->setReady(0);
		if (ci == NULL){
			printf("null pointer to ci\n");
			return;
		}
		//ci->ready=0;
		ci->data.clear();
		ci->data2fp.clear();
		mapCount++;
		//std::vector <Point> current;
		Point p, p2f;
		FILE *f;
		char name[256];
		sprintf(name,"/tmp/map%04i.dat", mapCount);
		printf("%s\n", name);
		f=fopen(name, "w");
		// coordinates.clear();
		// coordinates2fp.clear();
		for (A1LidarData &data:data){
			if (data.valid&& data.r <LIDAR_RANGE){
				//DATA IS ROUNDED AND DUPLICATES ARE ELIMINATED
				float x = round(data.x*1000)/1000;
				float y = round(data.y*1000)/1000;
				float x2 = round(data.x*100)/100;
				float y2 = round(data.y*100)/100;
				p= (Point(x, y));
				p2f=Point(x2, y2);
				//coordinates.insert(p);
				//coordinates2fp.insert(p2f);
				ci->data.insert(p);
				ci->data2fp.insert(p2f);
				if (ci->debugOn){
					fprintf(f, "%.2f\t%.2f\n", p2f.x, p2f.y);
				}
            }
		}
		fclose(f);
		printf("added data to lidar containers, coord = %i, coord2fp = %i\n", ci->data.size(), ci->data2fp.size());

		// for (Point d:coordinates){
		// 	ci->data.insert(d);
		// }
		// for (Point d:coordinates2fp){
		// 	ci->data2fp.insert(d);
		// }
		if (!ci->data.empty()){
			ci->setReady(1);
		}
		//ci->ready=1;
		ci->iteration++;
		printf("added data to interface containers\n");

	}


};

class Callback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
    //int iteration=0;
    //int confIteration=0;
    Configurator * c;
    float L=0;
	float R=0;

public:
int ogStep=0;

Callback(Configurator *conf): c(conf){
}
void step( AlphaBot &motors){
	if (c ==NULL || !c->running){
		motors.setRightWheelSpeed(0); //temporary fix because motors on despacito are the wrong way around
		motors.setLeftWheelSpeed(0);
		printf("null pointer to configurator in stepcallback\n");
	}
	if (c->getIteration() <=0){
		return;
	}
	c->trackTaskExecution(*(c->getTask()));	
//	c->controlGoal.trackDisturbance(controlGoal.disturbance, MOTOR_CALLBACK, deltaPose)
	//EndedResult controlEnded = controlGoal.checkEnded();
	if (c->controlGoal.checkEnded().ended){
		c->controlGoal.change =1;
		return;
	}
	//if (c->getTask()->change){
	c->controlGoal.trackDisturbance(c->controlGoal.disturbance, c->getTask()->getAction());
	c->changeTask(c->getTask()->change, c->plan, c->collisionGraph[0], ogStep);
	float gain=1;
	//  if (c->getTask()->direction==Direction::DEFAULT){
	//  	gain=0.95;
	//  }
    motors.setRightWheelSpeed(c->getTask()->getAction().getRWheelSpeed()*gain); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(c->getTask()->getAction().getLWheelSpeed());
	printf("step: R=%f\tL=%f, conf iteration = %i\n", c->getTask()->getAction().getRWheelSpeed(), c->getTask()->getAction().getLWheelSpeed(), c->getIteration());
    //iteration++;
}
};



int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	configurator.numberOfM = THREE_M;
	configurator.graphConstruction = A_STAR;
	configurator.setBenchmarking(1);
	configurator.planning =1;
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
		configurator.worldBuilder.debug = atoi(argv[1]);
	}
	if (argc>2){
		configurator.setSimulationStep(atof(argv[2]));
	}
	printf("debug on = %i, planning on = %i\n", configurator.debugOn, configurator.planning);
	printf("box2drange = %f\n", BOX2DRANGE);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface);
	Callback cb(&configurator);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	configurator.start();
	lidar.start();
	motors.start();
	do {
	} while (!getchar());
	configurator.stop();
	motors.stop();
	lidar.stop();

}
	
	
