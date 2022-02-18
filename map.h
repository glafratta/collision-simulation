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
public:
	std::vector <float>* xVec= new std::vector <float>;
	std::vector <float>* yVec= new std::vector <float>;
	std::vector <Obstacle*>* obstacles= new std::vector <Obstacle*>;
	b2Vec2 upperCorner;

 
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
		std::vector<float>::iterator xIt = std::max_element(xVec->begin(), xVec->end());
		std::vector<float>::iterator yIt = std::max_element(yVec->begin(), yVec->end());
		upperCorner.x = *xIt;
		upperCorner.y = *yIt;
	}

	// b2Vec2 getMax() {
	// 	return upperCorner;
	// }

	///for testing purposes only

	void printMax(){
		std::cout<<upperCorner.x<<"\t"<<upperCorner.y<<std::endl;
	}


	void printMaxType(){
		std::cout<<typeid(upperCorner.x).name()<<" , "<<typeid(upperCorner.y).name()<<std::endl;
	}
};
