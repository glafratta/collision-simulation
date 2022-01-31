#include "box2d/box2d.h"
#include "obstacle.h"
#include "listener.h"
#include "map.h"
#include <vector>


class Environment : public b2World{
private:
	float timeStep = 1 / 80; //sec
	int velIt = 8;
	int posIt = 3;
	std::vector <std::string>* fileList;
public:
	b2Vec2 gravity = { 0.0f, 0.0f };
	b2World* world = new b2World(gravity);
	Listener* listener = new Listener;
	std::vector <Map*>* maps = new std::vector <Map*>;
	world->SetContactListener(listener);

	/b2world->SetAllowSleeping(false);
	//	b2world->SetContactListener(listener);
	//	
	//};
	
	int howManyContacts() {
		int n_cont = 0;
		for (b2Contact* c = world->GetContactList(); c; c = c->GetNext()) {
			n_cont++;
		}
		std::cout << n_cont << std::endl;
		return n_cont;
	}

	void setWorld(b2World*) {

	}

	void setFileList() { //i'll need to change this when i stop working offline
		std::ifstream list("maps.txt");
		std::string map;
		while (list >> map) {
			fileList->push_back(map);
		}
	}

	std::vector <std::string>* getFileList() {
		return fileList;
	}

	void makeMapFromList() {
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
		(*maps)[i]->setFilename((*fileList)[i]);
		(*maps)[i]->setIteration(i);
		makeObstacles((*maps)[i]);
	}


	//void Step() {
	//	b2world->Step(timeStep, velIt, posIt);
	//}

	///TO_DO: make the map vector of a max size reflecting working memory ca. 20s = 100 maps
};
