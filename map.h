#include "Box2D/Box2D.h"
#include "/usr/include/opencv4/opencv2/core.hpp" 
#include <fstream>
#include <vector>
#include <algorithm>
#include "obstacle.h"

class Map {
private:
	int iteration;
	std::string filename;
public:
	//void * obstacles;  //maybe i'll implement this if I need to make the vector into a 2d array
	// std::vector <float>* xVec= new std::vector <float>;
	// std::vector <float>* yVec= new std::vector <float>;
	std::vector <Obstacle*>* obstacles= new std::vector <Obstacle*>;
	//cv::Mat matrix = cv::estimateAffinePartial2D

 
	~Map() {
		delete obstacles;
	};

	void setFilename(std::string file) {
		filename = file;
	}

	std::string getFilename() {
		return filename;
	}

	// void setIteration(int it) {
	// 	iteration = it;
	// }

	// int getIteration() {
	// 	return iteration;
	// }


	//THIS FUNCTION IS USED FOR COORDINATES STORED AS ONE X AND ONE Y VECTOR
	// void storePoints() { //reads into .dat file and creates a static body per point. maybe use argv in main to get those, or map+file no
	// 	std::ifstream map(filename); //EXCEPTION: filename doesnt exist
	// 	float x, y, temp;
	// 	while (map >> x >> y) { //for actual LIDAR data use (map>>x>>y>>temp>>temp>>temp)
	// 		xVec->push_back(approximate(x)); //rouding and adding .05 to make up for the fact that this is the centre of the obstacle body	
	// 		yVec->push_back(approximate(y));
	// 	}
	// 	map.close();
	// }



	// void cleanUp(std::vector <float> * vec){ //eliminates duplicates
	// 	std::vector <float>* tempVec;
	// 	for (int i=0; i<vec->size(); i++){
	// 		if (((*vec)[i-2])&& (*vec)[i]=(*vec)[i-2]){
	// 			tempVec->push_back((*vec)[i]);
	// 		}
			
	// }

	//TESTING PURPOSES


	// int countDup(){
	// 	int count =0;
	// 	for (int i =1; i<xVec->size(); i++){
	// 		if (xVec->at(i)== xVec->at(i-1) & yVec->at(i)==yVec->at(i-1)){
	// 			count++;
	// 		}
	// 	}
	// 	return count;
	// }

	// int countLines(){
	// 	int count =0;
	// 	for (int i=0; i<xVec->size(); i++){
	// 		count++;
	// 	}
	// 	return count;
	// }





	///for testing purposes only

	// void printMax(){
	// 	std::cout<<upperCorner.x<<"\t"<<upperCorner.y<<std::endl;
	// }


	// void printMaxType(){
	// 	std::cout<<typeid(upperCorner.x).name()<<" , "<<typeid(upperCorner.y).name()<<std::endl;
	// }

};
