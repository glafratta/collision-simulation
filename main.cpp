#include "Box2D/Box2D.h"
#include <iostream>
//#include <chrono>
#include "robot.h"
#include "listener.h"
//#include "/usr/local/include/CppTimer.h"
#include "lidar.h"
#include <thread>
#include "alphabot.h"
#include <unistd.h>



//the environment is sampled by the LIDAR every .2 seconds

void plotData(b2Body* body, b2World& world, std::string filename) { ///prints path on screen	
	std::ofstream file(filename);
	float timeStep = 1.0f / 80.0f;
	int32 velocityIterations = 8;
	int32 positionIterations = 3;
	for (int32 i = 0; i < 500; i++)
	{
		world.Step(timeStep, velocityIterations, positionIterations);
		b2Vec2 position = body->GetPosition();
		file << position.x << "\t" << position.y << std::endl;
	}
}


void startAlphabotWrapper(AlphaBot * motors){ //wrapper for starting the robot
	motors->start();
	motors->setRightWheelSpeed(1.0);
	motors->setLeftWheelSpeed(1.0);
}


int main(int, char**) {
	//world setup with environment class
	//AlphaBot *motors = new AlphaBot();
	AlphaBot motors;
	Box2DEnv* box2d = new Box2DEnv;
	Listener* listener = new Listener;
	box2d->world->SetContactListener(listener);
	//A1Lidar *lidar= new A1Lidar(false);
	A1Lidar lidar;
	DataInterface dataInterface;
	dataInterface.setBox2D(box2d);
	lidar.registerInterface(&dataInterface);
	box2d->setAlphabot(&motors);
	// std::thread startLidar(&A1Lidar::start, lidar, "/dev/serial0", 300); //starts lidar and motors in separate threads
	// std::thread startMotors(startAlphabotWrapper, motors);
	// motors->start();
	// lidar->start();
	motors.start();
	lidar.start();
	//motors->setRightWheelSpeed(0.0); //possible solution for the gpioinitialise is macros?
	//motors->setLeftWheelSpeed(0.0);
	do {
	} while (!getchar());
	// startLidar.join();
	//startMotors.join();
	motors.stop();
	lidar.stop();


	//make lidar and alphabod type classes pointers, can create threads for processes that start and stop lidar and robot separately
	



	delete listener, box2d, motors, lidar;
}
	
	
