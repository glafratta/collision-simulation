#include "box2d/box2d.h"
#include <iostream>
#include <chrono>
#include "robot.h"
#include "listener.h"
#include "environment.h"

#define TIME_STEP 1.0f / 80.0f //environment sampled every .2s
#define VEL_IT 8
#define POS_IT 3

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

int main() {
	//world setup with environment class

	Environment* env = new Environment;
	Listener* listener = new Listener;
	b2World* world = env->world;
	world->SetContactListener(listener);
	Robot* robot = env->robot;
	//robot->setVelocity({ 0.0, 3.0 });

	////////CODE LAYOUT////////////
	///For event handling install Qt
	//Events that need handled: robot-> is crashed, a map.dat file being created


	//timeStart;

	//while(stopKeyPressed = false){
		//if (time==timeStart+.2s){
		//env->update(); //uodates environment data
		// env->step(); //simulates data for the next x seconds and if the robot crashes in that time it makes a maneuvre
		//}
//}
	plotData(robot->body, *world, "robot.dat");


	//for (int i = 0; i < 300; i++) {
	//	world->Step(TIME_STEP, VEL_IT, POS_IT);
	//	std::cout << robot.body->GetPosition().x << "\t" << robot.body->GetPosition().y << std::endl;

	//};

	delete listener, env, world, robot;
}
	
	