#include <iostream>
#include "box2d/box2d.h"
#include "robot.h"
#include "obstacle.h"
#include <fstream>
#include <string>
#include <vector>

//for each point in the lidar reading make a suitably sized static body

//is robot (0,0)?

//each time step update location of world objects wrt to robot (check unit of measurement of robot and lidar)

//how to query whether object will collide? 

//simulate maneuvre: take into account how long it takes to brake irl, inertia and time taken to turn at a certain speed

void storePoints(std::string filename, std::vector <float>* xPtr, std::vector <float>* yPtr) { //reads into .dat file and creates a static body per point. maybe use argv in main to get those, or map+file no
	std::ifstream map(filename);
	float x, y; 
	while (map>>x>>y) {
		xPtr->push_back(x);
		yPtr->push_back(y);
	}
}

void makeObstacles(std::vector <Obstacle*> * bufPtr, std::vector <float>* xPtr, std::vector <float>* yPtr, b2World & world) {
	for (int i = 0; i < xPtr->size(); i++) {
		Obstacle* obPtr = new Obstacle(world, xPtr->at(i), yPtr->at(i));
		bufPtr->push_back(obPtr);
	}
}


int main(int argc, char* argv[]) {// ./executable for pp in *.dat // maye this isn't main.cpp
	//create world and robot
	std::vector <float> x, y;
	std::vector <float>* xPtr; 
	std::vector <float>* yPtr;
	xPtr = &x;
	yPtr = &y;
	std::vector <Obstacle*> obstacleBuffer;
	std::vector <Obstacle*>* bufferPtr;
	bufferPtr = &obstacleBuffer;
	b2Vec2 gravity(0.0f, 0.0f); //set gravity to 0 because it's 2d viewed from above
	b2World world(gravity);
	float timeStep = .2; //environment sampled every 200 ms?
	Robot robot(world); //might need to edit robot.h with dimensions once I 3d print a suitable chassis
	storePoints("map001.dat", xPtr, yPtr);
	makeObstacles(bufferPtr, xPtr, yPtr, world);
	std::cout<<robot.getVelocity()<<std::endl;
	return 0;

}
