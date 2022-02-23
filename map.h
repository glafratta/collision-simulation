//#include "/usr/include/Box2D/Box2D.h"
#include "/usr/include/opencv4/opencv2/core.hpp" 
#include <fstream>
#include <vector>
#include <algorithm>
#include "obstacle.h"
#include <unordered_set>

class Map {
private:
	std::string filename;
	int iteration;
public:
	float coordinates[1000][2]; //chose 1000 because 
	float * coordinatesPtr = &coordinates[0][0];
	std::vector <float>* xVec= new std::vector <float>;
	std::vector <float>* yVec= new std::vector <float>;
	// std::unordered_set <float>* xVec= new std::unordered_set <float>;
	// std::unordered_set <float>* yVec= new std::unordered_set <float>;
	std::vector <Obstacle*>* obstacles= new std::vector <Obstacle*>;
	//cv::Mat matrix = cv::estimateAffinePartial2D
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

	//THIS FUNCTION IS USED FOR COORDINATES STORED AS ONE X AND ONE Y VECTOR
	void storePoints() { //reads into .dat file and creates a static body per point. maybe use argv in main to get those, or map+file no
		std::ifstream map(filename); //EXCEPTION: filename doesnt exist
		float x, y, temp;
		while (map >> x >> y) { //for actual LIDAR data use (map>>x>>y>>temp>>temp>>temp)
			xVec->push_back(approximate(x)); //rouding and adding .05 to make up for the fact that this is the centre of the obstacle body	
			yVec->push_back(approximate(y));
		}
	}

	

	// void storeCoords(){
	// 	std::ifstream map(filename); //EXCEPTION: filename doesnt exist
	// 	float x, y, temp;
	// 	int tempIt = 0;
	// 	float tempArr[2];
	// 	float tempCoord[1000][2];
	// 	//float * coordPtr = //try pointer and next
	// 	while (map>>x>>y){
	// 		// tempVec->push_back(approximate(x));
	// 		// tempVec->push_back(approximate(y));
	// 		tempArr[0] = approximate(x);
	// 		tempArr[1]=approximate(y);
	// 		*tempCoord[tempIt] = *tempArr;
	// 		// tempCoord[tempIt][0] = tempArr[0];
	// 		// tempCoord[tempIt][1]=tempArr[1];

	// 		// // for (int i=0; i<; i++){
	// 		// // 	std::cout<<" for loop entered\n";
	// 		// 	if (!(coordinates[0]) || tempArr!=coordinates[tempIt-1]){
	// 		// 	coordinates[tempIt][0]=tempArr[0];
	// 		// 	coordinates[tempIt][1]=tempArr[1];
	// 		// 	std::cout<<" ifloop entered\n";
	// 		// }
	// 		// std::cout<<coordinates[i][0]<<"\t"<<coordinates[i][1]<<std::endl;
	// 		tempIt++;
	// 		std::cout<<tempIt<<"\n";
	// 	//	}
			 
	// 	}
	// 	// for (int i=0; i<tempVec->size(); i++){
	// 	// 	if (i==0 || i % 2==0){
	// 	// 		coordinates[(int)i/2][0]= (*tempVec)[i];
	// 	// 	}
	// 	// 	else if (i%2==1){
	// 	// 		coordinates[(int)i/2][1]=(*tempVec)[i];
	// 	// 	}
	// 	// }

	// 	// for (int i=0; i<1000; i++){
	// 	// 	for (int j=0; j<2; j++){
	// 	// 		coordinates[i][j]=(*tempVec)[tempIt];
	// 	// 	}
	// 	// }
	// 	delete tempArr;
	// }


	float approximate(float a){
		a = round(a*10)/10;
		if (a<0){
			a-=0.05;
		}
		else if (a>=0){
			a+= 0.05;
		}
		return a;
	}

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



	// void setMax() {
	// 	//EXCEPTION: map vector is empty
	// 	std::vector<float>::iterator xIt = std::max_element(xVec->begin(), xVec->end());
	// 	std::vector<float>::iterator yIt = std::max_element(yVec->begin(), yVec->end());
	// 	upperCorner.x = *xIt;
	// 	upperCorner.y = *yIt;
	// }

	// b2Vec2 getMax() {
	// 	return upperCorner;
	// }

	///for testing purposes only

	// void printMax(){
	// 	std::cout<<upperCorner.x<<"\t"<<upperCorner.y<<std::endl;
	// }


	// void printMaxType(){
	// 	std::cout<<typeid(upperCorner.x).name()<<" , "<<typeid(upperCorner.y).name()<<std::endl;
	// }

};
