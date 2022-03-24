#include "Box2D/Box2D.h"
#include <iostream>
#include "robot.h"
#include "listener.h"
//#include "/usr/local/include/CppTimer.h"
//#include "lidar.h"
#include <thread>
#include "alphabot.h"
#include <unistd.h>
#include "a1lidarrpi.h"
#include "environment.h"
#include <stdio.h>
#include <stdlib.h>
#include <string> 
#include <iomanip>
#include <sstream> //for writing string into file, for checking, std::ostringstream



//the environment is sampled by the LIDAR every .2 seconds

class DataInterface : public A1Lidar::DataInterface{
public: 
    int numberOfScans=0; //for testing purposes
    Box2DEnv * box2d;
    

    void setBox2D(Box2DEnv * _box2d){
        box2d = _box2d;
    }


	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
        box2d->createMap();
        Frame * frame = box2d->frames->at(box2d->frames->size()-1);
        frame->coordinates.resize(240*240); //range is 12m
        cv::Point point;

        float x, y;
        int iteration =0;
		for (A1LidarData &data:data){
			if (data.valid){ 
                x = approximate(data.x);
                y= approximate(data.y);
			}
			point = cv::Point(x, y);
        	frame->coordinates[iteration] = point;
			Obstacle* obstacle = new Obstacle(*(box2d->world), x, y); //doesn't check obstacles for duplicates
			frame->obstacles->push_back(obstacle);
            iteration++;
		}
        box2d->simulate(); //simulates collision for 5 seconds
        std::cout << "velocity estimated to be "<<box2d->realVelocity.x<<", "<<box2d->realVelocity.y<<"\n";

	}


	float approximate(float a){ //more efficient with pointers?? (float a, float * ptr){assign value of a to *ptr}
		a = round(a*10)/10;
		if (a<0){
			a-=0.05;
		}
		else if (a>=0){
			a+= 0.05;
		}
        return a;
	}
    
    // void transformForMatrix(float x, float y, int i, int j){ //transform cartesian coordinates into matrix coordinates
    //     //matrix is 240 bins of .1m, lidar data is in meters
    //     x = int(x*10); // transform x and y in decimeters and rounds down
    //     y = int(y*10);
    //     i= 240/2-y; // x index: columns/2-y
    //     j= 240/2+x; //y index: columns/2+x

    // }
};



int main(int, char**) {
	//world setup with environment class
	//AlphaBot *motors = new AlphaBot();
	AlphaBot motors;
	Box2DEnv* box2d = new Box2DEnv;
	Listener* listener = new Listener;
	box2d->world->SetContactListener(listener);
	A1Lidar lidar;
	DataInterface dataInterface;
	dataInterface.setBox2D(box2d);
	lidar.registerInterface(&dataInterface);
	box2d->setAlphabot(&motors);
	// std::thread startLidar(&A1Lidar::start, lidar, "/dev/serial0", 300); //starts lidar and motors in separate threads
	// std::thread startMotors(startAlphabotWrapper, motors);
	lidar.start();
	motors.start();
	motors.setRightWheelSpeed(0.0); //possible solution for the gpioinitialise is macros?
	motors.setLeftWheelSpeed(0.0);
	motors.setRightWheelSpeed(1.0f); //possible solution for the gpioinitialise is macros?
	motors.setLeftWheelSpeed(1.0f);
	do {
	} while (!getchar());
	// startLidar.join();
	//startMotors.join();
	motors.stop();
	lidar.stop();


	//make lidar and alphabod type classes pointers, can create threads for processes that start and stop lidar and robot separately
	



	delete listener, box2d, motors, lidar;
}
	
	
