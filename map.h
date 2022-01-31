#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "box2d/box2d.h"
#include "obstacle.h"

class Map {
private:
	std::string filename;
	int iteration;
	b2Vec2 upperCorner;
public:
	std::vector <float>* xVec= new std::vector <float>;
	std::vector <float>* yVec= new std::vector <float>;
	std::vector <Obstacle*>* obstacles= new std::vector <Obstacle*>;
 
	~Map() {
		delete xVec, yVec, obstacles;
	};

	void setFilename(std::string file) {
		filename = file;
	}

	std::string getFilename() {
		return filename;
	}

	void setIteration(int it) {
		iteration = it;
	}

	int getIteration() {
		return iteration;
	}


	void storePoints(std::string filename, std::vector <float>* xPtr, std::vector <float>* yPtr) { //reads into .dat file and creates a static body per point. maybe use argv in main to get those, or map+file no
		std::ifstream map(filename);
		float x, y;
		while (map >> x >> y) {
			xPtr->push_back(x);
			yPtr->push_back(y);
		}
	}

	void storePoints() { //reads into .dat file and creates a static body per point. maybe use argv in main to get those, or map+file no
		std::ifstream map(filename); //EXCEPTION: filename doesnt exist
		float x, y;
		while (map >> x >> y) {
			xVec->push_back(x);	//EXCEPTION: vector already has content
			yVec->push_back(y);
		}
	}

	//void makeObstacles() { //obstacle vector (emp
	//	for (int i = 0; i < xVec->size(); i++) {
	//		Obstacle* obPtr = new Obstacle(world, xVec->at(i), yVec->at(i));
	//		obPtr->body->SetAwake(true);
	//		obstacles->push_back(obPtr);
	//	}
	//}

	void setMax() {
		//EXCEPTION: map vector is empty
		upperCorner.x = std::max_element(xVec->begin(), xVec->end());
		upperCorner.y = std::max_element(yVec->begin(), yVec->end());
	}

	b2Vec2 getMax() {
		return upperCorner;
	}
};
