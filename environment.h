#include "box2d/box2d.h"
#include "obstacle.h"
#include "map.h"
#include <vector>
#include <string>
#include <dirent.h>


class Environment {
private:
	float timeStepSim = 1 / 80; //sec, in simulation
	int velIt = 8;
	int posIt = 3;
	std::vector <std::string> fileList; //list of all the names of the text files containing maps. this list is pulled from maps.txt for now but later it will need to be optimised
	int i = 0; // iteration useful to know what map we're using
	float timeStepLidar = 1 / 5;
	b2Vec2 realVelocity = 0;
public:
	b2Vec2 gravity = { 0.0f, 0.0f };
	b2World* world = new b2World(gravity);
	Robot* robot = new Robot(*world);
	std::vector <Map*>* maps = new std::vector <Map*>;
	
	
	int howManyContacts() {
		int n_cont = 0;
		for (b2Contact* c = world->GetContactList(); c; c = c->GetNext()) {
			n_cont++;
		}
		std::cout << n_cont << std::endl;
		return n_cont;
	}


	void setFileList() { //i'll need to change this when i stop working offline
		std::ifstream list;
		list.open("maps.txt", std::fstream::in);
		std::string map;
		while (list >> map) {
			fileList.push_back(map);
		}
		list.close();
	}

///____TO DO: FIX THIS FUNCTIONNN_________________

	void setFileList(int i, std::vector <std::string> _filelist) { //can i use dirent.h to do this?
		
	}

	std::vector <std::string> getFileList() {
		return fileList;
	}

	void createMap() {
		Map* map = new Map;
		maps->push_back(map);
	}

	void makeObstacles(Map *map) { //obstacle vector (emp
		for (int i = 0; i < map->xVec->size(); i++) {
			Obstacle* obPtr = new Obstacle(*world, map->xVec->at(i), map->yVec->at(i));
			obPtr->body->SetAwake(true);
			map->obstacles->push_back(obPtr);
		}
	}

	void setMapDetails(int i) { //may not need to be a for loop, just make i=iteration
		//(*maps)[i]->setFilename((*fileList)[i]); //filelist as pointer
		(*maps)[i]->setFilename(fileList[i]);

		(*maps)[i]->setIteration(i);
		makeObstacles((*maps)[i]);
	}

	Map* getMap(int i) {
		return (*maps)[i];
	}

	void setIteration(int i) {
		iteration = i;
	}

	int getIteration() {
		return iteration;
	}

	void update() {
		createMap();
		setMapDetails(iteration);
		iteration++;
	}

	b2Vec2 findRealDistance() { //takes in the list of maps and the number of (1/5)sec intervals passed since start of simulation
		b2Vec2 returnVec = { 0.0f, 0.0f };
		if ((*maps)[i]->getMax() & (*maps)[i - 1]) {
			returnVec = { (*maps)[i]->getMax().x - (*maps)[i - 1]->getMax().x, (*maps)[i]->getMax().y - (*maps)[i - 1]->getMax().y };
		}
		//std::vector <float>* x = new std::vector <float>;
		//std::vector <float>* y = new std::vector <float>;
		//(*maps)[i]->storePoints((*maps)[i]->getFilename(), x, y);
		//if (i == 0) {
		//	returnVec = { (*std::max_element(x->begin(), x->end())) ,(*std::max_element(y->begin(), y->end())) };
		//}
		//else if (i > 0) {
		//	std::vector <float>* xPrev = new std::vector <float>;
		//	std::vector <float>* yPrev = new std::vector <float>;
		//	(*maps)[i-1]->storePoints((*maps)[i-1]->getFilename(), xPrev, yPrev);
		//	returnVec = { (*std::max_element(x->begin(), x->end())) - (*std::max_element(xPrev->begin(), xPrev->end())) ,(*std::max_element(y->begin(), y->end())) - (*std::max_element(yPrev->begin(), yPrev->end())) };
		//	delete xPrev, yPrev;
		//}
		//delete x, y;
		return returnVec;
	}

	void setRealVelocity(b2Vec2 distance) {
		b2Vec2 vel;
		float velScalar = distance.Length() / timeStepLidar;
		float angle= atan2(distance.y, distance.x);//in rad
		vel.x = velScalar * cos(angle);
		vel.y = velScalar * sin(angle);
		realVelocity=vel;
	}

	b2Vec2 getRealVelocity() {
		return realVelocity;
	}

	void step(Robot robot) { //simulates wht happens in 5 seconds ////NOT TESTED!!!!!!!!!!!!!!!!!!
		for (int i = 0; i < 300; i++) {//5 seconds with time step 1/80
			if (i > 0) {
				setRealVelocity(findRealDistance());
				robot.setVelocity(realVelocity);
				robot.bodyDef.position.Set(0.0f, 0.0f); //robot is always 0.0 at the beginning of the simulation
			}
			world->Step(timeStepSim, velIt, posIt);
			if (robot.crashed) {
				break;
			}
		}
		robot.pathPlan(); //can plan paths to 1. get out of being stuck, 2. avoid a close obstacle 3. avoid far away obstacle
	}

	///TO_DO: make the map vector of a max size reflecting working memory ca. 20s = 100 maps
};

