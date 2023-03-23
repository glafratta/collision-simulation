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
<<<<<<< HEAD
	printf("map %i\n", mapCount);
        // auto now =std::chrono::high_resolution_clock::now();
	    // std::chrono::duration<float, std::milli>diff= now - c->previousTimeScan; //in seconds
	    // c->timeElapsed=float(diff.count())/1000; //express in seconds	
        // c->totalTime += c->timeElapsed;
	    // c->previousTimeScan=now; //update the time of sampling
        // if (c->timeElapsed >.25){
        //     return;
        // }
||||||| cdf1af5
        // auto now =std::chrono::high_resolution_clock::now();
	    // std::chrono::duration<float, std::milli>diff= now - c->previousTimeScan; //in seconds
	    // c->timeElapsed=float(diff.count())/1000; //express in seconds	
        // c->totalTime += c->timeElapsed;
	    // c->previousTimeScan=now; //update the time of sampling
        // if (c->timeElapsed >.25){
        //     return;
        // }
=======
>>>>>>> 009791de97438fa942fb34d03c08651b0588cf33
		std::vector <Point> current;
		Point p1, p0;
		for (A1LidarData &data:data){
			if (data.valid&& data.r <1.0){
				//DATA IS ROUNDED AND DUPLICATES ARE ELIMINATED
				float x = round(data.x*100)/100;
				float y = round(data.y*100)/100;
				float r = round(data.r*100)/100;
				float phi = round(data.phi*100)/100;
				p1= (Point(x, y, r, phi));
				if (p1!= p0){
					current.push_back(p1);
				}
				p0= p1;
           	 }
		}
<<<<<<< HEAD
		//fclose(map); //uncomment here - FINISH WRITE TO FILE
		printf("current size = %i\n", current.size());
		printf("is ptr null %i\n", c ==NULL);
||||||| cdf1af5
		//fclose(map); //uncomment here - FINISH WRITE TO FILE
=======
>>>>>>> 009791de97438fa942fb34d03c08651b0588cf33
		c->NewScan(current);
<<<<<<< HEAD
		printf("scan\n");
||||||| cdf1af5
		//printf("scan\n");
=======
>>>>>>> 009791de97438fa942fb34d03c08651b0588cf33
		

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
	
	
