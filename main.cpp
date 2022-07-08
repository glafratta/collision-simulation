#include "Box2D/Box2D.h"
//#include "listener.h"
#include <thread>
///#include "StepCallback.h"
//#include <unistd.h>
#include "a1lidarrpi.h"
//#include "environment.h"
#include "StepCallback.h"
#include <stdio.h>
#include <stdlib.h>
//#include "obstacle.h"
#include <iostream>
#include <ncurses.h>
#define _USE_MATH_DEFINES



//the environment is sampled by the LIDAR every .2 seconds

class DataInterface : public A1Lidar::DataInterface{
COnfigurator &box2d;
public: 
    int numberOfScans=0; //for testing purposes
	const float distance = 1.0f;

    

    DataInterface(COnfigurator & _box2d): box2d(_box2d){};


	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
		
		// //START BENCHMARKING

		// auto begin = std::chrono::high_resolution_clock::now();
		////WRITE TO FILE
        numberOfScans++; 
		// char tmp[256];
		// sprintf(tmp, "/tmp/map%04d.dat", numberOfScans);
        //  FILE * file =fopen(tmp, "w+"); //to here


		box2d.previous=box2d.current;
		box2d.current.clear();

		for (A1LidarData &data:data){
			if (data.valid&& (-M_PI_4<=data.phi<= M_PI_4)){
				box2d.current.push_back(cv::Point2f(data.x,data.y));
				//fprintf(file, "%f\t%f\n", data.x, data.y);
            }


		}
		//fclose(file); //uncomment here - FINISH WRITE TO FILE
		box2d.NewScan();
		//refresh();



		// FINISH BENCHMARKING
		// auto end = std::chrono::high_resolution_clock::now();
		// auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
		// std::cout<<"scanned in "<<elapsed.count()* 1e-9<<"seconds\n";
		
		

	}


};




int main(int, char**) {
	//world setup with environment class
	initscr();
	A1Lidar lidar;
	AlphaBot motors;
	COnfigurator box2d;
	DataInterface dataInterface(box2d);
	Callback cb(box2d);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	lidar.start();
	motors.start();

	// do {
	// } while (!getchar());
	std::this_thread::sleep_for(std::chrono::seconds(10));
	endwin();


	motors.stop();
	std::cout<<dataInterface.numberOfScans<<std::endl;
	lidar.stop();


	


//	delete listener;
}
	
	
