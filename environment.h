#include "obstacle.h"
#include "frame.h"
#include "Box2D/Box2D.h"
#include "robot.h"
#include <vector>
//#include <string>
//#include <dirent.h>
#include "opencv2/calib3d.hpp"
#include <stdexcept>

class Box2DEnv{
public:
	//std::vector <std::string> fileList; //list of all the names of the text files containing maps. this list is pulled from maps.txt for now but later it will need to be optimised
	int i = 0; // iteration useful to know what map we're using
	float samplingRate = 1.0f / 5.0f; //for now it is fixed at .2ms but will need to write a function timing each revolution 
	b2Vec2 realVelocity = {0.0,0.0};
	int iteration=-	1; //represents that hasn't started yet, robot isn't moving and there are no map data
	b2Vec2 gravity = { 0.0f, 0.0f };
	b2World* world = new b2World(gravity);
	Robot* robot = new Robot(*world);
	std::vector <Frame*>* frames = new std::vector <Frame*>;
	
	




	void createMap() {
		Frame* frame = new Frame;
		frames->push_back(frame);
	}

	Frame* getFrame(int i=iteration) { //unless prompted otherwise, returns map for current iteration
		return (*frames)[i];
	}

	void setIteration(int i) {
		iteration = i;
	}

	int getIteration() {
		return iteration;
	}


	// 	int howManyContacts() {
	// 	int n_cont = 0;
	// 	for (b2Contact* c = world->GetContactList(); c; c = c->GetNext()) {
	// 		n_cont++;
	// 	}
	// 	std::cout << n_cont << std::endl;
	// 	return n_cont;
	// }


	void findRealVelocity(){
		cv::_InputArray current = cv::_InputArray::_InputArray(getFrame());
		cv::_InputArray previous = cv::_InputArray::_InputArray(getFrame(iteration-1));
		cv::Mat transformMatrix = estimateAffinePartial2D(previous, current);
		if (!transformMatrix.empty()){
			realVelocity.x = -(transformMatrix.at<double>(0,2))*samplingRate;
			realVelocity.y = -(transformMatrix.at<double>(1,2))*samplingRate;
		}
	}



	void simulate(float timeStep = 1.0f/80.0f, int posIt = 3, int velIt = 8) { //simulates wht happens in 5 seconds ////produces segfault
		robot->bodyDef.position.Set(0.0f, 0.0f); //robot is always 0.0 at the beginning of the simulation
		iteration++;
		if (iteration > 0) { //NOT IMPLEMENTING THIS YET BECAUSE OBSTACLE AVOIDANCE IS GOING TO BE A LIL HARDER THAN THIS
			findRealVelocity();
			robot->setVelocity(realVelocity); /
		}
		for (int i = 0; i < 300; i++) {//5 seconds with time step 1/80
			world->Step(timeStep, velIt, posIt);
			if (robot->crashed) {
			 	robot->pathPlan();
			 	return;
			 }
		}
	}
};
	///TO_DO: make the map vector of a max size reflecting working memory ca. 20s = 100 maps

