#include <iostream>
#include "box2d/box2d.h"
#include "robot.h"
#include "obstacle.h" 
#include "listener.h" 
#include "ball.h" //for testing purposes
#include "map.h"
#include <fstream> //ifstream
#include <string>
#include <vector>
#include <algorithm> //std::max_element
#include <iterator> 

#define TIME_STEP 1.0f / 80.0f //environment sampled every .2s
#define VEL_IT 8
#define POS_IT 3

//these could go into a world.h file

void makeObstacles(std::vector <Obstacle*> * bufPtr, std::vector <float>* xPtr, std::vector <float>* yPtr, b2World & world) {
	for (int i = 0; i < xPtr->size(); i++) {
		Obstacle* obPtr = new Obstacle(world, xPtr->at(i), yPtr->at(i));
		obPtr->body->SetAwake(true);
		bufPtr->push_back(obPtr);
	}
}

int howManyContacts(b2World& world) {
	int n_cont = 0;
	//b2Contact* con = world.GetContactList();
	for (b2Contact* c = world.GetContactList(); c; c = c->GetNext()) {
		n_cont++;
	}
	std::cout << n_cont<<std::endl;
	return n_cont;
}

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
b2Vec2 findForce(b2Vec2 distance_v, double time, double mass = .7) {
	b2Vec2 velocity_v, acc_v, force;
	double angle = atan2(distance_v.y, distance_v.x);
	float velocityScalar = distance_v.Length() / time; 
	velocity_v.x = velocityScalar * cos(angle);
	velocity_v.y = velocityScalar * sin(angle);
	float accScalar = velocity_v.Length() / (time / 10); //for sake of simplicity i divided the time interval into 10
	acc_v.x = accScalar * cos(angle);
	acc_v.y = accScalar * sin(angle);
	double forceScalar = accScalar * mass;
	force.x = forceScalar * cos(angle);
	force.y = forceScalar * sin(angle);
	return force;
	
}

//this could go into a map.h file
void storePoints(std::string filename, std::vector <float>* xPtr, std::vector <float>* yPtr);

b2Vec2 findRealDistance(std::vector <std::string>* filenames, int it) { //takes in the list of maps and the number of (1/5)sec intervals passed since start of simulation
	b2Vec2 velocity, returnVec;
	std::vector <float>* x = new std::vector <float>;
	std::vector <float>* y = new std::vector <float>;
	storePoints((*filenames)[it], x, y);
	if (it == 0) {
		returnVec = { (*std::max_element(x->begin(), x->end())) ,(*std::max_element(y->begin(), y->end())) };
	}
	else if(it > 0) {
		std::vector <float>* xPrev = new std::vector <float>;
		std::vector <float>* yPrev = new std::vector <float>;
		storePoints((*filenames)[it-1], xPrev, yPrev);
		returnVec= { (*std::max_element(x->begin(), x->end()))- (*std::max_element(xPrev->begin(), xPrev->end())) ,(*std::max_element(y->begin(), y->end()))- (*std::max_element(yPrev->begin(), yPrev->end())) };
		delete xPrev, yPrev;
	}
	delete x, y;
	return returnVec; 
}
//world.h
void setFilenames(std::vector <std::string>* filenames) {
	std::ifstream list("maps.txt");
	std::string map;
	while (list >> map) {
		filenames->push_back(map);
	}
}
 //map.h
void printFilenames(std::vector <std::string>* filenames) {
	for (int i = 0; i < filenames->size(); i++) {
		std::cout << (*filenames)[i] << std::endl;
	}
}

void storePoints(std::string filename, std::vector <float>* xPtr, std::vector <float>* yPtr) { //reads into .dat file and creates a static body per point. maybe use argv in main to get those, or map+file no
	std::ifstream map(filename);
	float x, y;
	while (map >> x >> y) {
		xPtr->push_back(x);
		yPtr->push_back(y);
	}
} //this and the following function can be merged once i stop working with text files

//each time step update location of world objects wrt to robot (check unit of measurement of robot and lidar)


//how to query whether object will collide? 

//simulate maneuvre: take into account how long it takes to brake irl, inertia and time taken to turn at a certain speed


int main() {// 


	//set up all variables
	//std::vector <float>* x = new std::vector <float>; //sets of variables for the current map
	//std::vector <float>* y = new std::vector <float>; 
	//std::vector <Obstacle*>* obstacles = new std::vector <Obstacle*>;

	//this is for off-line simulations, maps are stored in a list (maps.txt) and the names into variable filenames

	/*std::vector <std::string>* filenames = new std::vector <std::string>;
	setFilenames(filenames); */

	//create world

	b2Vec2 gravity(0.0f, 0.0f); //set gravity to 0 because it's 2d viewed from above
	b2World *world= new b2World(gravity);
	world->SetAllowSleeping(false);
	Listener* listenerP = new Listener;
	world->SetContactListener(listenerP);
	
	//setup robot
	Robot robot(*world); //might need to edit robot.h with dimensions once I 3d print a suitable chassis

	//storePoints((*filenames)[0], x, y);
	//makeObstacles(obstacles, x, y, world);
	//b2Vec2 dist = findRealDistance(filenames, 1);
	//std::cout << dist.x << "\t" << dist.y << "\n";

	//robot.body->ApplyLinearImpulseToCenter({ 1.75,0 }, true);
	
	robot.setVelocity({ 2.0, 0.0 });
	////insert this in a while loop, like (!keyPressed)
	for (int i = 0; i < 300; i++) {
		world->Step(TIME_STEP, VEL_IT, POS_IT);
		std::cout << robot.body->GetPosition().x << "\t" << robot.body->GetPosition().y << std::endl;

	}

	//TO_DO: figure out how to implement this in a real time scenario
	//std::max_element()

	
	//delete listenerP, obstacles, filenames, x,y;
	//b2World ~world();
	return 0;

}
