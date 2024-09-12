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
    int count =0;
	buildType=2;
    std::vector <BodyFeatures> result;
    for (Point p: points){
        if (count%2==0){
            BodyFeatures feature;
            feature.pose.p = p.getb2Vec2(); 
            result.push_back(feature);  
        }
        count++;
    }
    return result;
}

class LidarInterface : public A1Lidar::DataInterface{
ConfiguratorInterface * ci;
public: 
    int mapCount =0;

    LidarInterface(ConfiguratorInterface * _ci): ci(_ci){}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
		if (ci == NULL){
			printf("null pointer to ci\n");
			return;
		}
		ci->data2fp.clear();
		mapCount++;
		Point p, p2f;
		FILE *f;
		char name[256];
		sprintf(name,"/tmp/map%04i.dat", mapCount);
		printf("scan %i\n",mapCount );
		f=fopen(name, "w");
		for (A1LidarData &data:data){
			if (data.valid&& data.r <LIDAR_RANGE){
				float x2 = round(data.x*100)/100;
				float y2 = round(data.y*100)/100;
				//p= (Point(x, y));
				p2f=Point(x2, y2);
				ci->data2fp.insert(p2f);
				if (ci->debugOn){
					fprintf(f, "%.2f\t%.2f\n", p2f.x, p2f.y);
				}
            }
		}
		fclose(f);
		printf("added data to lidar containers coord2fp = %i\n",  ci->data2fp.size());
		if (!ci->data2fp.empty()){
			ci->setReady(1);
		}
		ci->iteration++;
		printf("added data to interface containers\n");

	}


};

class Callback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
    Configurator * c;
    float L=0;
	float R=0;

public:
int ogStep=0;


Callback(Configurator *conf): c(conf){
}
void step( AlphaBot &motors){
	if (c ==NULL){
		printf("null pointer to configurator in stepcallback\n");
	}
	if (c->getIteration() <=0){
		return;
	}
	c->trackTaskExecution(*(c->getTask()));	
	if (c->plan.empty()|| !c->running){
		motors.setRightWheelSpeed(0); //temporary fix because motors on despacito are the wrong way around
 		motors.setLeftWheelSpeed(0);
		return;		
	}
	if (c->controlGoal.checkEnded().ended){
		c->controlGoal.change =1;
		return;
	}
	c->controlGoal.trackDisturbance(c->controlGoal.disturbance, c->getTask()->getAction());
	c->changeTask(c->getTask()->change, c->planVertices, ogStep);
    motors.setRightWheelSpeed(c->getTask()->getAction().getRWheelSpeed()); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(c->getTask()->getAction().getLWheelSpeed());
	printf("step: R=%f\tL=%f, conf iteration = %i, plan size = %i\n", c->getTask()->getAction().getRWheelSpeed(), c->getTask()->getAction().getLWheelSpeed(), c->getIteration(), c->plan.size());
    //iteration++;
}
};



int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal;
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	char name[60];
	sprintf("target_benchmark_alt_%s", get_Foldername());
	configurator.setBenchmarking(1, name);
	configurator.planning =1;
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
		configurator.worldBuilder.debug = atoi(argv[1]);
	}
	configurator.setSimulationStep(.27);
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
	
	
