
#include <iostream>
#include <thread>
#include "alphabot.h"
#include <unistd.h>
#include "a1lidarrpi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string> 
#include <iomanip>
#include <sstream> //for writing string into file, for checking, std::ostringstream



//the environment is sampled by the LIDAR every .2 seconds

class DataInterface : public A1Lidar::DataInterface{
public: 
int numberOfScans =1;

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
		
	
		////WRITE TO FILE
        // numberOfScans++; //uncomment for writing
		// std::stringstream tmp; //uncomment from here
        // tmp << "map" << std::setw(4) << std::setfill('0') << numberOfScans << ".dat";
        // const char * filename = tmp.str().c_str();
        // FILE * file =fopen(filename, "w+"); //to here
		// std::ostringstream stream; //uncomment here
		for (A1LidarData &data:data){
			if (data.valid){ 
				//stream <<data.x<<"\t"<<data.y<<"\n"; //uncomment from here
				std::cout<<data.x<<"\t"<<data.y<<"\n";
			} //to here
			// const char * line = stream.str().c_str();
            // 	//std::cout<<line<<"\n";
			// fputs(line, file);
		}
		//fclose(file); //uncomment here
		std::cout<<"\n\n\n";
		//numberOfScans++;

	}


};



int main(int, char**) {
	AlphaBot motors; //records data for 2 seconds
	A1Lidar lidar;
	DataInterface dataInterface;
	lidar.registerInterface(&dataInterface);
	lidar.start();
	motors.start();
	motors.setRightWheelSpeed(0.0); //possible solution for the gpioinitialise is macros?
	motors.setLeftWheelSpeed(0.0);
	motors.setRightWheelSpeed(0.25f); //possible solution for the gpioinitialise is macros?
	motors.setLeftWheelSpeed(0.25f);
	do {}
	while (!getchar());
	motors.stop();
	lidar.stop();


	//make lidar and alphabod type classes pointers, can create threads for processes that start and stop lidar and robot separately
	



	
}
	
	
