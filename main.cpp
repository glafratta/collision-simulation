#include "Box2D/Box2D.h"
#include <iostream>
#include "robot.h"
#include "listener.h"
#include <thread>
#include "alphabot.h"
#include <unistd.h>
#include "a1lidarrpi.h"
#include "environment.h"
#include <stdio.h>
#include <stdlib.h>

//the environment is sampled by the LIDAR every .2 seconds

class DataInterface : public A1Lidar::DataInterface{
public: 
    int numberOfScans=0; //for testing purposes
    Box2DEnv * box2d;
    

    void setBox2D(Box2DEnv * _box2d){
        box2d = _box2d;
    }


	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
		
		// //START BENCHMARKING

		// auto begin = std::chrono::high_resolution_clock::now();

		////WRITE TO FILE
        //numberOfScans++; //uncomment for writing
		// std::stringstream tmp; //uncomment from here
        // tmp << "map" << std::setw(4) << std::setfill('0') << numberOfScans << ".dat";
        // const char * filename = tmp.str().c_str();
        // FILE * file =fopen(filename, "w+"); //to here

		for (A1LidarData &data:data){
			if (data.valid){ 
				box2d->frames[1].push_back(cv::Point2f(data.x,data.y));

			}

		}
		//fclose(file); //uncomment here - FINISH WRITE TO FILE
		box2d->lidarIteration++;
		//std::cout<<"scan "<<box2d->lidarIteration<<std::endl;


		// FINISH BENCHMARKING
		// auto end = std::chrono::high_resolution_clock::now();
		// auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
		// std::cout<<"scanned in "<<elapsed.count()* 1e-9<<"seconds\n";
		
		

	}


};



int main(int, char**) {
	//world setup with environment class
	//AlphaBot *motors = new AlphaBot();
	AlphaBot motors;
	Box2DEnv box2d;
	Listener* listener = new Listener;
	box2d.world->SetContactListener(listener);
	A1Lidar lidar;
	DataInterface dataInterface;
	dataInterface.setBox2D(&box2d);
	lidar.registerInterface(&dataInterface);
	//box2d->robot->setMotors(&motors);
	box2d.start();
	lidar.start();
	//motors.start();
	// motors.setRightWheelSpeed(0.0); //possible solution for the gpioinitialise is macros?
	// motors.setLeftWheelSpeed(0.0);
	// motors.setRightWheelSpeed(0.25f); //possible solution for the gpioinitialise is macros?
	// motors.setLeftWheelSpeed(0.25f);
	do {
	} while (!getchar());
	//motors.stop();
	lidar.stop();
	box2d.stop();


	//make lidar and alphabod type classes pointers, can create threads for processes that start and stop lidar and robot separately
	



	delete listener;
}
	
	
