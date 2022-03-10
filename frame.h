#include "Box2D/Box2D.h"
#include "opencv2/opencv.hpp"
#include <fstream>
#include <vector>
#include <algorithm>
#include "obstacle.h"

class Frame {
private:
	int iteration;
	//std::string filename; 
public:
	std::vector <Obstacle*>* obstacles= new std::vector <Obstacle*>;
	//cv::Mat matrix = cv::Mat(240, 240, CV_8U); //if no info comes in matrix is empty. If range is 12m, matrix has to be 240dm
	std::vector <cv::Point> coordinates;
 
	~Frame() {
		delete obstacles;
	};


	void setIteration(int i){
		iteration = i;
	}

	int getIteration(){
		return iteration;
	}

	bool isDuplicate(){ //this needs done still
		return false;
	}





};
