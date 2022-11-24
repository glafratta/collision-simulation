#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include "Box2D/Box2D.h"
#include "robot.h"
#include <dirent.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include <thread>
#include <filesystem>
#include <cmath>
#include <unistd.h>
#include <ncurses.h>
#include <fstream>
#include "state.h"
#include <time.h>

//class State;

class Configurator{
protected:
	float maxAbsSpeed = .125;
	float gain = .05;
	//std::vector <float> timeStamps;
	double samplingRate = 1.0/ 5.0; //default
	int iteration=0; //represents that hasn't started yet, robot isn't moving and there are no map data
	bool crashed=0;
	b2Vec2 desiredVelocity;
	b2Vec2 absPosition = {0.0f, 0.0f};
	FILE * dumpPath;
	FILE * dumpDeltaV;
	char fileNameBuffer[50];
public:
	float affineTransError =0;
	bool filterOn=1;
	//std::vector <cv::Point2f> previous;
	char *folder;
	char readMap[50];
	//std::vector <cv::Point2f> current;
	char msg[25];
	std::vector <State> plan; //from here we can find the current state
	State desiredState;
	std::chrono::high_resolution_clock::time_point previousTimeScan;
	float rightWheelSpeed=0;
	float leftWheelSpeed=0;
	float timeElapsed =0;
	float totalTime=0;
	

	

	struct getVelocityResult{
		bool valid =0;
		b2Vec2 vector = {0.0f, 0.0f};
		float angle;
		getVelocityResult(){}
		getVelocityResult(b2Vec2 disp, float maxSpeed = 0.125):vector(disp){
			valid=1;
			if (disp.y ==0 && disp.x==0){
				angle =0;
			}
			else{
				angle= atan(disp.y/disp.x);
			}
		}

	};

//calculuate displacement and angle using partial affine transformation

// 1:  new scan available: box2d world is rebuilt with objects, current Action is checked#

Configurator(){
	previousTimeScan = std::chrono::high_resolution_clock::now();
	totalTime = 0.0f;
	leftWheelSpeed = desiredState.getAction().getLWheelSpeed();
	rightWheelSpeed = desiredState.getAction().getRWheelSpeed();
	dumpDeltaV = fopen("/tmp/deltaV.txt", "w");
}

Configurator(State _state): desiredState(_state){
	previousTimeScan = std::chrono::high_resolution_clock::now();
	leftWheelSpeed = desiredState.getAction().getLWheelSpeed();
	rightWheelSpeed = desiredState.getAction().getRWheelSpeed();
	totalTime =0.0f;
}

class MedianFilter{
    int kSize;
    std::vector <float> bufferX;
    std::vector <float> bufferY;
public:
    MedianFilter(int _k=3): kSize(_k){
        bufferX = std::vector<float>(kSize);
        bufferY = std::vector<float>(kSize);
    }

    void filterFloat(float &c, std::vector<float> & buffer){ //value gets modified in the input vector
        buffer.push_back(c);
        buffer.erase(buffer.begin());
        std::vector <float> tmp = buffer;
        std::sort(tmp.begin(), tmp.end());
        int index = int(buffer.size()/2)+1;
        c = tmp[index];
        //printf("median value at index: %i, value = %f\n", index, tmp[index]);
    }

    void applyToPoint(cv::Point2f &p){
        filterFloat(p.x, bufferX);
        filterFloat(p.y, bufferY);
    }

};

void setNameBuffer(char * str){ //set name of file from which to read trajectories. by default trajectories are dumped by 'state' into a robot000n.txt file.
								//changing this does not change where trajectories are dumped, but if you want the robot to follow a different trajectory than the one created by the state
	sprintf(fileNameBuffer, "%s", str);
	printf("reading trajectories from: %s\n", fileNameBuffer);
}

void setReadMap(char * str){
	sprintf(readMap,"%s", str);
	printf("map name: %s\n", readMap);

}

char * getReadMap(){
	return readMap;
}

char * getFolder(){
	return folder;
}
void setFolder(char * _folder){ //the folder from where LIDAR data is read
	std::filesystem::path folderPath(_folder);
		if (exists(folderPath)){
			if (is_directory(folderPath)){
				folder = _folder;
			}
			else{
				printf("not a directory");
			}
		}
		else{
			printf("%s doesn't exist", _folder);
		}
	printf("maps stored in %s\n", folder);
}

void NewScan(); 

int getIteration(){
	return iteration;
}
Configurator::getVelocityResult GetRealVelocity(std::vector <cv::Point2f>);

void controller();

void addIteration(){
	iteration++;
}


void updateAbsPos(b2Vec2 vel){
	absPosition.x += vel.x*timeElapsed;
	absPosition.y += vel.y*timeElapsed;
}

b2Vec2 getAbsPos(){
	return absPosition;
}

State getCurrentState(){
	if (plan.size()>0){
		return plan[0];
	}
	else {
		return desiredState;
	}
}

b2Vec2 getDesVel(){
	return desiredVelocity;
}

b2Vec2 estimateDisplacementFromWheels();

};




 #endif