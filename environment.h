
#include "obstacle.h"
#include "map.h"
#include "lidar.h"
#include "Box2D/Box2D.h"
#include "robot.h"
#include <vector>
#include <string>
//#include <dirent.h>

class Box2DEnv{
public:
	std::vector <std::string> fileList; //list of all the names of the text files containing maps. this list is pulled from maps.txt for now but later it will need to be optimised
	int i = 0; // iteration useful to know what map we're using
	float timeStepLidar = 1.0f / 5.0f;
	b2Vec2 realVelocity = {0.0,0.0};
	int iteration=-	1; //represents that hasn't started yet, robot isn't moving and there are no map data
	b2Vec2 gravity = { 0.0f, 0.0f };
	b2World* world = new b2World(gravity);
	Robot* robot = new Robot(*world);
	std::vector <Map*>* maps = new std::vector <Map*>;
	
	




	void createMap() {
		Map* map = new Map;
		maps->push_back(map);
	}

	// void makeObstacles(Map *map) { //obstacle vector (emp
	// 	for (int i=0; i<map->xVec->size(); i++) {
	// 		Obstacle* obstacle = new Obstacle(*world, map->xVec->at(i), map->yVec->at(i)); //ok right this aint gon work using iterators, so 
	// 		obstacle->body->SetAwake(true);
	// 		map->obstacles->push_back(obstacle);
	// 	}
	// }

	void setMapDetails(int i) { 
	//	(*maps)[i]->setFilename(fileList[i]); 
		(*maps)[i]->setIteration(i);
	//	(*maps)[i]->storePoints();
		//(*maps)[i]->storeCoords();
		//makeObstacles((*maps)[i]);
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

	// void update() {
	// 	iteration++;
	// 	//createMap();
	// 	//setMapDetails(iteration);

	// }

//THESE FUNCTIONS WERE FROM WHEN I WAS TRYING TO GET THE VELOCITY FROM TH EUPPER CORNER

	// b2Vec2 findRealDistance() {//returns the distance of the robot from upper corner of the room, idk why im spending so much time on this since its just a temporary fix
	// 	b2Vec2 returnVec(0.0, 0.0);
	// 		if ((*maps)[iteration - 1]) {
	// 		returnVec.x = (*maps)[iteration]->upperCorner.x - (*maps)[iteration-1]->upperCorner.x;
	// 		returnVec.y = (*maps)[iteration]->upperCorner.y - (*maps)[iteration-1]->upperCorner.y;
	// 	}
	// 	return returnVec;
	// } //this only woroks in a single room with size less than max lidar range

	// //testing purposes

	// void printRealDistance(){
	// 	std::cout<<findRealDistance().x<<" , "<<findRealDistance().y<<std::endl;  
	// }

	// void setRealVelocity(b2Vec2 distance) {
	// 	float velScalar = distance.Length() / timeStepLidar; //substitue this with time elapsed for 1 lidar revolution
	// 	float angle= atan2(distance.y, distance.x);//in rad
	// 	realVelocity.x = velScalar * cos(angle);
	// 	realVelocity.y = velScalar * sin(angle);
	// }

	// b2Vec2 getRealVelocity() {
	// 	return realVelocity;
	// }

	// void setFileList() { //i'll need to change this when i stop working offline or every
	// 	std::ifstream list;
	// 	list.open("maps.txt", std::fstream::in);
	// 	std::string map;
	// 	while (list >> map) {
	// 		fileList.push_back(map); //map name is sample_maps/map* (= includes file path)
	// 	}
	// 	list.close();
	// }



	// std::vector <std::string> getFileList() { //doesn't work to get the names of the files
	// 	return fileList;
	// }

	// 	int howManyContacts() {
	// 	int n_cont = 0;
	// 	for (b2Contact* c = world->GetContactList(); c; c = c->GetNext()) {
	// 		n_cont++;
	// 	}
	// 	std::cout << n_cont << std::endl;
	// 	return n_cont;
	// }



	void simulate(float timeStep = 1.0f/80.0f, int posIt = 3, int velIt = 8) { //simulates wht happens in 5 seconds ////produces segfault
		robot->bodyDef.position.Set(0.0f, 0.0f); //robot is always 0.0 at the beginning of the simulation
		//update();
		iteration++;
		// if (iteration > 0) { //NOT IMPLEMENTING THIS YET BECAUSE OBSTACLE AVOIDANCE IS GOING TO BE A LIL HARDER THAN THIS
		// 	setRealVelocity(findRealDistance()); 
		// 	robot->setVelocity(realVelocity); /
		// 	}
		for (int i = 0; i < 300; i++) {//5 seconds with time step 1/80
			world->Step(timeStep, velIt, posIt);
			// if (robot->crashed) {
			// 	robot->pathPlan();
			// 	break;
			// }
		}
			//std::cout<<robot->body->GetPosition().x<<"\t"<<robot->body->GetPosition().y<<"\n";
	}
};
	///TO_DO: make the map vector of a max size reflecting working memory ca. 20s = 100 maps

