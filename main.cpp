//#include "Box2D/Box2D.h"
#include "src/configurator.h"
#include "a1lidarrpi.h"
#include "alphabot.h"
#include <stdio.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#define _USE_MATH_DEFINES



//the environment is sampled by the LIDAR every .2 seconds

class LidarInterface : public A1Lidar::DataInterface{
Configurator c;
FILE * dumpPath;
//for debugging
FILE * speed;
char folder[250];
public: 
    int mapCount =0;

    LidarInterface(Configurator & _c): c(_c){
		// time_t now =time(0);
		// tm *ltm = localtime(&now);
		// int y,m,d, h, min;
		// y=ltm->tm_year-100;
		// m = ltm->tm_mon +1;
		// d=ltm->tm_mday;
		// h = ltm->tm_hour;
		// min = ltm->tm_min;
		// sprintf(folder, "%02i%02i%02i_%02i%02i/",d,m,y, h, min );
		// if (mkdir(folder, 0777)!= -1){
		// 	c.setFolder(folder);
		// }
        // dumpPath = fopen("/tmp/dumpPath.txt", "w");
        // fclose(dumpPath);
        // speed = fopen("/tmp/speed.txt", "w");
        // fclose(speed);   
		}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
	    mapCount++;
		std::vector <Point> current;
        //char name_r[50];
		//sprintf(name_r, "%s%s%04i.dat", folder,c.getReadMap(), mapCount);
		//FILE * map = fopen(name_r, "wt");
		for (A1LidarData &data:data){
			if (data.valid&& data.r <1.5){
				Point p(data.x, data.y, data.r, data.phi);
				if (p!= *(&p-1)){
				current.push_back(p);
				//fprintf(map, "%.2f\t%.2f\n", data.x, data.y);
				}
            }
		}
		//fclose(map); //uncomment here - FINISH WRITE TO FILE
		c.NewScan(current);
		

	}

	char * getFolder(){
		return folder;
	}


};

class Callback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
    int iteration=0;
    int confIteration=0;
    Configurator *c;

public:

Callback(Configurator *conf): c(conf){
    float L,R;
    Configurator * box2d;
}
void step( AlphaBot &motors){
    motors.setRightWheelSpeed(-(c->getCurrentState()->getAction().getLWheelSpeed())); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(-(c->getCurrentState()->getAction().getRWheelSpeed()));
    iteration++;
//         confIteration = c->getIteration();
//         //desiredVelocity = c->estimateVelocityFromWheels(.2, c->leftWheelSpeed, c->rightWheelSpeed, 0.08);
//     printf("changed speed to R= %f, L= %f\n",c->rightWheelSpeed, c->leftWheelSpeed);
//     if (-c->rightWheelSpeed > -c->leftWheelSpeed){
//         printf("robot going left\n");
//     }
//     else if (-c->rightWheelSpeed < -c->leftWheelSpeed){
//         printf("robot foing right\n");
//     }

//   }
//     printf("callback iteration %i\n", iteration);
//     //b2Vec2 plannedVelocity(0.0f, 0.0f);
//     b2Vec2 displacement(0, 0);

//         if (c->timeElapsed>0){
//             // plannedVelocity.x = c->estimateDisplacementFromWheels().x /c->timeElapsed ; // m/s
//             // plannedVelocity.y = c->estimateDisplacementFromWheels().y /c->timeElapsed; // m/s
//             displacement = c->estimateDisplacementFromWheels();
//         }
//         // displacement.x =plannedVelocity.x*c->timeElapsed;
//         // displacement.y =plannedVelocity.y*c->timeElapsed;
//         fprintf(control, "%f\t%f\n",displacement.x, displacement.y);
//         fprintf(dump, "%f\t%f\n",c->getAbsPos().x+displacement.x, c->getAbsPos().y+displacement.y);
//         fclose(dump);
//         fclose(control);

}
};


int main(int, char**) {
	//world setup with environment class
	initscr();
	A1Lidar lidar;
	AlphaBot motors;
    State desiredState;
    Configurator configurator(desiredState);
    configurator.setReadMap("map");
   // configurator.setFolder(lidar.getFolder());
	LidarInterface dataInterface(configurator);
	Callback cb(&configurator);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	lidar.start();
	motors.start();

	do {
	} while (!getchar());
	//std::this_thread::sleep_for(std::chrono::seconds(10));
	// endwin();


	motors.stop();
	//std::cout<<dataInterface.numberOfScans<<std::endl;
	lidar.stop();


	


//	delete listener;
}
	
	
