#include "obstacle.h"
#include "frame.h"
#include "Box2D/Box2D.h"
#include "robot.h"
#include <vector>
#include "opencv2/opencv.hpp"
#include "/usr/local/include/alphabot.h"
//#include <string>
//#include <dirent.h>

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
	AlphaBot * alphabot;
	




	void createMap() {
		Frame* frame = new Frame;
		frames->push_back(frame);
	}

	Frame* getFrame(int i) { //unless prompted otherwise, returns map for current iteration
		return (*frames)[i];
	}

	void setIteration(int i) {
		iteration = i;
	}

	int getIteration() {
		return iteration;
	}

	void setAlphabot(AlphaBot * _alphabot){
		alphabot = _alphabot; 
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
		std::vector <cv::Point> current = getFrame(iteration)->coordinates;
		std::vector <cv::Point> previous = getFrame(iteration-1)->coordinates;
		cv::Mat transformMatrix = cv::estimateAffinePartial2D(previous, current, cv::noArray(), cv::LMEDS);
		if (!transformMatrix.empty()){
			realVelocity.x = -(transformMatrix.at<double>(0,2))*samplingRate;
			realVelocity.y = -(transformMatrix.at<double>(1,2))*samplingRate;
		}
	}

	void printRealVelocity(){
		std::cout<<realVelocity.Length()<<std::endl;
	}



	void simulate(float timeStep = 1.0f/10.0f, int posIt = 3, int velIt = 8) { //simulates wht happens in 5 seconds ////produces segfault
		robot->bodyDef.position.Set(0.0f, 0.0f); //robot is always 0.0 at the beginning of the simulation
		iteration++;
		if (iteration > 0) { 
			findRealVelocity();
			robot->velocity = realVelocity; 
		}
		for (int i = 0; i < 30; i++) {//5 seconds with time step 1/80
			world->Step(timeStep, velIt, posIt);
			if (robot->crashed) {
			 	//robot->pathPlan();
				//alphabot->setRightWheelSpeed(0.0f);
				//alphabot->setLeftWheelSpeed(0.0f);
				return;
			 	break;
			}
		}
		
	}
};
	///TO_DO: make the map vector of a max size reflecting working memory ca. 20s = 100 maps

