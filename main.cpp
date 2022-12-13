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
FILE * dumpPath;
//for debugging
FILE * speed;
char folder[250];
public: 
    int mapCount =0;

    LidarInterface(Configurator * _c): c(_c){
		time_t now =time(0);
		tm *ltm = localtime(&now);
		int y,m,d, h, min;
		y=ltm->tm_year-100;
		m = ltm->tm_mon +1;
		d=ltm->tm_mday;
		h = ltm->tm_hour;
		min = ltm->tm_min;
		sprintf(folder, "%02i%02i%02i_%02i%02i/",d,m,y, h, min );
		if (mkdir(folder, 0777)!= -1){
			c->setFolder(folder);
		}
        // dumpPath = fopen("/tmp/dumpPath.txt", "w");
        // fclose(dumpPath);
        // speed = fopen("/tmp/speed.txt", "w");
        // fclose(speed);   
		}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
	    mapCount++;
        // auto now =std::chrono::high_resolution_clock::now();
	    // std::chrono::duration<float, std::milli>diff= now - c->previousTimeScan; //in seconds
	    // c->timeElapsed=float(diff.count())/1000; //express in seconds	
        // c->totalTime += c->timeElapsed;
	    // c->previousTimeScan=now; //update the time of sampling
        // if (c->timeElapsed >.25){
        //     return;
        // }
		std::vector <Point> current;
	    // char name_r[50];
		// sprintf(name_r, "%s%s%04i.dat", folder,c.getReadMap(), mapCount);
		// FILE * map = fopen(name_r, "wt");
		Point p1, p0;
		for (A1LidarData &data:data){
			if (data.valid&& data.r <c->getState()->lidarRange){
				//DATA IS ROUNDED AND DUPLICATES ARE ELIMINATED
				float x = round(data.x*100)/100;
				float y = round(data.y*100)/100;
				float r = round(data.r*100)/100;
				float phi = round(data.phi*100)/100;
				p1= (Point(x, y, r, phi));
				if (p1!= p0){
					current.push_back(p1);
				//fprintf(map, "%.2f\t%.2f\n", data.x, data.y);
				}
				p0= p1;
            }
		}
		//fclose(map); //uncomment here - FINISH WRITE TO FILE
		c->NewScan(current);
		//printf("scan\n");
		

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
	// if (c->plan.size()==0){
	// printf("baseline state\n");
    // motors.setRightWheelSpeed(-(c->desiredState.getAction().getLWheelSpeed())); //temporary fix because motors on despacito are the wrong way around
    // motors.setLeftWheelSpeed(-(c->desiredState.getAction().getRWheelSpeed()));
	// }
	// else if ( c->plan.size() >0){   
	// printf("avoidance state\n"); 
	// motors.setRightWheelSpeed(-(c->plan[0].getAction().getLWheelSpeed())); //temporary fix because motors on despacito are the wrong way around
    // motors.setLeftWheelSpeed(-(c->plan[0].getAction().getRWheelSpeed()));

	// }
	L= -(c->getState()->getAction().getRWheelSpeed());
	R = -(c->getState()->getAction().getLWheelSpeed());
    motors.setRightWheelSpeed(R); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(L);
	printf("step: R=%f\tL=%f, conf iteration = %i\n", -R, -L, c->getIteration());
    iteration++;
}
};



int main(int argc, char** argv) {
	//world setup with environment class
	initscr();
	A1Lidar lidar;
	AlphaBot motors;
    State desiredState;
    Configurator configurator(desiredState);
    configurator.setReadMap("map");
	LidarInterface dataInterface(&configurator);
	//DebugClass db(configurator);
	Callback cb(&configurator);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	lidar.start();
	motors.start();

	do {
	} while (!getchar());
	endwin();


	motors.stop();
	lidar.stop();
	//db.stop();


	


//	delete listener;
}
	
	
