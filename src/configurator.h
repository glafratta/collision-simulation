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
#include "general.h" //general functions + point class + typedefs + state.h + boost includes
#include <boost/graph/depth_first_search.hpp>


class Configurator{
protected:
	//std::vector <float> timeStamps;
	double samplingRate = 1.0/ 5.0; //default
	int iteration=0; //represents that hasn't started yet, robot isn't moving and there are no map data
	bool crashed=0;
	b2Vec2 desiredVelocity;
	b2Vec2 absPosition = {0.0f, 0.0f};
	FILE * dumpPath;
	FILE * dumpDeltaV;
	char fileNameBuffer[50];
	int maxObstacleWM =3;
	State state;
	int maxNoChildren =2;
public:
	float affineTransError =0;
	bool filterOn=1;
	char *folder;
	char readMap[50];
	char msg[25];
	State desiredState;
	std::chrono::high_resolution_clock::time_point previousTimeScan;
	float timeElapsed =0;
	float totalTime=0;
	std::vector <Point> current;
	bool planning =1;

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
	state = desiredState;
	dumpDeltaV = fopen("/tmp/deltaV.txt", "w");

}

Configurator(State &_state): desiredState(_state), state(_state){
	previousTimeScan = std::chrono::high_resolution_clock::now();
	totalTime =0.0f;

}

void __init__(){
}

void __init__(State & _state){
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

// void NewScan(std::vector <cv::Point2f> &); 

void NewScan(std::vector <Point> &); 

int getIteration(){
	return iteration;
}
// Configurator::getVelocityResult GetRealVelocity(std::vector <cv::Point2f> &, std::vector <cv::Point2f> &);

Configurator::getVelocityResult GetRealVelocity(std::vector <Point> &, std::vector <Point> &);

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

// Plan * plan(){
// 	return &currentPlan;
// }


State * getState(int advance=0){ //returns state being executed
	return &state;
}


void applyController(bool, State &);

int getMaxObstacleWM(){
	return maxObstacleWM;
}

b2Vec2 estimateDisplacementFromWheels();

int getMaxStepDuration(){
	return state.hz * state.getSimDuration();
}

// State returnBest(State & s1, State & s2){ //returns pointer, remember to dereference
// 	switch(s1.getType() == s2.getType()){
// 	//if one of the state is the desired state, return the desired state
// 		case 0: 
// 			if (s1.getType() == desiredState.getType()){
// 				return s1;
// 			}
// 			else if (s2.getType() == desiredState.getType()){
// 				return s2;
// 			}
// 		break;
// 		case 1:
// 			switch (s1.getType()== desiredState.getType()){
// 				//if both states are avoiding, choose the one with the least duration
// 				case 0: 
// 					if (s1.getStepDuration()< s2.getStepDuration()){
// 						return s1;
// 					}
// 					else if (s2.getStepDuration()< s1.getStepDuration()){
// 						return s2;
// 					}
// 					else {
// 						return s1;
// 					}
// 				//if both states are desired, choose the one with the most
// 				case 1:
// 					if (s1.getStepDuration()> s2.getStepDuration()){
// 						return s1;
// 					}
// 					else if (s2.getStepDuration()> s1.getStepDuration()){
// 						return s2;
// 					}
// 					else {
// 						return s1;
// 					}
// 				default:
// 					break;
// 			}

// 			break;
// 			default: break;
// 	}

// }

// void eliminateDisturbance(b2World &, State &, b2Vec2&, float&, State::Direction); //performs reactive avoidance

bool eliminateDisturbance(b2World &, vertexDescriptor &, Graph &, b2Vec2 &, float &); //adds two states if crashed but always next up is picked

vertexDescriptor eliminateDisturbance(b2World & world, vertexDescriptor v, Graph &g);


bool build_tree(vertexDescriptor v, Graph&g, b2World & w);

//special case if robot is going in circles

State::Direction getOppositeDirection(State::Direction d){
    switch (d){
        case State::Direction::LEFT: return State::Direction::RIGHT;break;
        case State::Direction::RIGHT: return State::Direction::LEFT;break;
        default:
        return State::Direction::NONE; printf("default direction\n");break;
    }
}

bool isFullLength(vertexDescriptor v, Graph &g, float length=0, int edgesTotal =0){
	//length = stepdur/hz *linvel
    if (boost::in_degree(v, g)<=0 && length < g[v].box2dRange){
        return false;
    }
    else if (length >=g[v].box2dRange){
        return true;
    }
    else{
        edgeDescriptor inEdge= boost::in_edges(v, g).first.dereference();
		printf("step dur = %i, linear speed = %f, a\n", g[inEdge].stepDuration, g[v].getAction().getLinearSpeed());
        length += g[inEdge].stepDuration/g[v].hz * g[v].getAction().getLinearSpeed();
		printf("length = %f\n", length);
        vertexDescriptor newV = boost::source(inEdge, g);
		edgesTotal++;
        return isFullLength(newV, g, length, edgesTotal);
    }

}


void addVertex(vertexDescriptor & src, vertexDescriptor& v1, Graph &g, State::Object obs = State::Object()){
	//vertexDescriptor v1=src;
	State::Direction d= State::Direction::NONE;
	if (g[src].options.size()>0){
		v1 = boost::add_vertex(g);
		d = g[src].options[0];
		printf("option = %i\n", d);
		g[src].options.erase(g[src].options.begin());
		g[v1]= State(obs, d);
		add_edge(src, v1, g).first;
		printf("edge %i, %i\n", src, v1);
	}
	else{///for debug
		printf("no options\n");
	}
	// for (auto i =g[src].options.begin(); i!=g[src].options.end(); i++){
	// 	if (*i = g[v1].getAction().getDirection()){
	// 		g[src].options.erase(i);
	// 	}
	// }
	//return v1;
}


};




 #endif