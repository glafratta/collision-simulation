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
#include <algorithm>

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
	printf("simduration = %i\n", state.getSimDuration());

}

void __init__(){
}

void __init__(State & _state){
}

// class MedianFilter{
//     int kSize;
//     std::vector <float> bufferX;
//     std::vector <float> bufferY;
// public:
//     MedianFilter(int _k=3): kSize(_k){
//         bufferX = std::vector<float>(kSize);
//         bufferY = std::vector<float>(kSize);
//     }

//     void filterFloat(float &c, std::vector<float> & buffer){ //value gets modified in the input vector
//         buffer.push_back(c);
//         buffer.erase(buffer.begin());
//         std::vector <float> tmp = buffer;
//         std::sort(tmp.begin(), tmp.end());
//         int index = int(buffer.size()/2)+1;
//         c = tmp[index];
//         //printf("median value at index: %i, value = %f\n", index, tmp[index]);
//     }

//     void applyToPoint(cv::Point2f &p){
//         filterFloat(p.x, bufferX);
//         filterFloat(p.y, bufferY);
//     }

// };

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

void eliminateDisturbance(b2World &, State &, b2Vec2&, float&, State::Direction); //performs reactive avoidance

bool eliminateDisturbance(b2World &, vertexDescriptor &, Graph &, b2Vec2 &, float &); //adds two states if crashed but always next up is picked

vertexDescriptor eliminateDisturbance(b2World & world, vertexDescriptor v, Graph &g);

vertexDescriptor eliminateDisturbance(vertexDescriptor, CollisionTree&, State  , b2World & , std::vector <vertexDescriptor> &);
bool build_tree(vertexDescriptor v, Graph&g, b2World & w);
bool build_tree(vertexDescriptor v, CollisionTree&g, State s, b2World & w, std::vector <vertexDescriptor>&);
//special case if robot is going in circles

State::Direction getOppositeDirection(State::Direction d){
    switch (d){
        case State::Direction::LEFT: return State::Direction::RIGHT;break;
        case State::Direction::RIGHT: return State::Direction::LEFT;break;
        default:
        return State::Direction::NONE; printf("default direction\n");break;
    }
}

template <typename V, typename G>
// bool isFullLength(vertexDescriptor v, Graph &g, float length=0, int edgesTotal =0){
bool isFullLength(V v, const G & g, float length=0, int edgesTotal =0){
	//printf("vertex = %i\tlength so far = %f, in edges = %i\n", v,length, boost::in_degree(v,g));
	//length = stepdur/hz *linvel
    if (boost::in_degree(v, g)==0 && length < desiredState.box2dRange){
		//printf("ended isFullLength, is root\n");
        return false;
    }
    else if (length >=desiredState.box2dRange){
		//printf("reached full length\n");
        return true;
    }
    else{
        edgeDescriptor inEdge= boost::in_edges(v, g).first.dereference();
		//printf("distance = %f\t", g[inEdge].distanceCovered);
        length += g[inEdge].distanceCovered;
		//printf("length = %f\t", length);
        //vertexDescriptor newV = boost::source(inEdge, g);
		//printf("newVertex = %i\n", newV);
		edgesTotal++;
        return isFullLength(boost::source(inEdge, g), g, length, edgesTotal);
    }

}

//FOR OLD STUFF WITH GRAPH
void addVertex(vertexDescriptor & src, vertexDescriptor& v1, Graph &g, State::Object obs = State::Object()){
	//vertexDescriptor v1=src;
	State::Direction d= State::Direction::NONE;
	if (g[src].options.size()>0){
		v1 = boost::add_vertex(g);
		d = g[src].options[0];
		//printf("option = %i\n", d);
		g[src].options.erase(g[src].options.begin());
		g[v1]= State(obs, d);
		add_edge(src, v1, g).first;
		printf("edge %i, %i\n", src, v1);
	}
	else{///for debug
		printf("no options\n");
	}

}

//FOR NEW COLLISIONTREE
void addVertex(vertexDescriptor & src, vertexDescriptor &v1, CollisionTree &g){
	//v1 = src;
	//State::Direction d= State::Direction::NONE;
	printf("vertex %i, options = %i\n", src, g[src].options.size());
	if (g[src].options.size()>0){
		v1 = boost::add_vertex(g);
		//d = g[src].options[0];
		//printf("option = %i\n", d);
		edgeDescriptor e = add_edge(src, v1, g).first;
		g[e].direction =g[src].options[0];
		g[src].options.erase(g[src].options.begin());
		//g[v1]= State(obs, d);
		printf("edge %i, %i, direction = %i\n", src, v1, static_cast<int>(g[e].direction));
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

edgeDescriptor findBestBranch(CollisionTree &g, std::vector <vertexDescriptor> _leaves){
	//FIND BEST LEAF
	vertexDescriptor best = _leaves[0];
	for (vertexDescriptor leaf: _leaves){
		printf("leaf %i, cost = %f\n", leaf, g[leaf].costSoFar);
		if (g[leaf].costSoFar<g[best].costSoFar){
			best = leaf;
		}
	}
	printf("best = %i\n", best);
	//FIND FIRST NODE BEFORE ORIGIN
	edgeDescriptor e;
	while (best != *(boost::vertices(g).first)){
		printf("current = %i\n", best);
		e = boost::in_edges(best, g).first.dereference();
		printf("best direction = %i\n", static_cast<int>(g[e].direction));
		best = e.m_source;
		printf("back = %i\n", best);
	}
	return e;
}

void constructWorldRepresentation(b2World & world, State::Direction d, b2Transform start){
	//TO DO : calculate field of view: has to have 10 cm on each side of the robot
	switch (d){
		case State::Direction::NONE:{
		//FIND BOUNDING BOX FOR POINTS OF INTEREST
		float mHead = sin(start.q.GetAngle())/cos(start.q.GetAngle()); //slope of heading direction
		float mPerp = -1/mHead;
		Point positionVector, radiusVector, maxFromStart; 
		float minX, minY, maxX, maxY;
		radiusVector.polarInit(BOX2DRANGE, start.q.GetAngle());
		maxFromStart = Point(start.p) + radiusVector;
		//printf("maxfrom start = x= %f, y= %f\n", maxFromStart.x, maxFromStart.y);
		b2Vec2 unitPerpR(-sin(start.q.GetAngle()), cos(start.q.GetAngle()));
		b2Vec2 unitPerpL(sin(start.q.GetAngle()), -cos(start.q.GetAngle()));
		//printf("start angle %fpi\ttangle for perpL = %f pi, for perpR = %f pi\n", start.q.GetAngle()/M_PI, atan2(unitPerpL.y, unitPerpL.x)/M_PI, atan2(unitPerpR.y, unitPerpR.x)/M_PI);
		float halfWindowWidth = .1;
		std::vector <Point> bounds;
		bounds.push_back(Point(start.p)+Point(unitPerpL.x *halfWindowWidth, unitPerpL.y*halfWindowWidth));
		bounds.push_back(Point(start.p)+Point(unitPerpR.x *halfWindowWidth, unitPerpR.y*halfWindowWidth));
		bounds.push_back(maxFromStart+Point(unitPerpL.x *halfWindowWidth, unitPerpL.y*halfWindowWidth)); 
		bounds.push_back(maxFromStart+Point(unitPerpR.x *halfWindowWidth, unitPerpR.y*halfWindowWidth));
		//printf("bounds: \t");
		// for (Point &p:bounds){
		// 	printf("%f, %f\t", p.x, p.y);

		// }
		// printf("\n");
		Point top, bottom;
		comparator compPoint;
		std::sort(bounds.begin(), bounds.end(), compPoint); //sort bottom to top
		bottom = bounds[0];
		top = bounds[3];
		// printf("new bounds: \t");
		// for (Point &p:bounds){
		// 	printf("%f, %f\t", p.x, p.y);
		// }
		// printf("\n");
		// printf("top = %f, %f\t bottom = %f, %f\n", top.x, top.y, bottom.x, bottom.y);
		float qBottomH, qTopH, qBottomP, qTopP;
		qBottomH = bottom.y - mHead*bottom.x;
		qTopH = top.y - mHead*top.x;
		qBottomP = bottom.y -mPerp*bottom.x;
		qTopP = top.y - mPerp*top.x;
		//CREATE POINTS
		for (Point &p:current){
			if (p != *(&p-1)&& p.y >= (mHead*p.x +qBottomH)&& p.y <= (mHead*p.x +qTopH)&& p.y >=(mPerp*p.x+qBottomP)&&p.y <=(mPerp*p.x+qTopP)){ //y range less than 20 cm only to ensure that robot can pass + account for error
				b2Body * body;
				b2BodyDef bodyDef;
				b2FixtureDef fixtureDef;
				bodyDef.type = b2_dynamicBody;
				b2PolygonShape fixture; //giving the point the shape of a box
				fixtureDef.shape = &fixture;
				fixture.SetAsBox(.001f, .001f); 
				//if (p.x !=0 && p.y!=0){
				bodyDef.position.Set(p.x, p.y); 
				body = world.CreateBody(&bodyDef);
				body->CreateFixture(&fixtureDef);
				//}

			}
		}
		break;
		}
		default:
		//printf("default case: circle\n");
		for (Point &p:current){			
			//printf("is in radius %i\n", p.isInRadius(start.p));
			if (p != *(&p-1)&& p.isInRadius(start.p, 0.1)){ //y range less than 20 cm only to ensure that robot can pass + account for error
				b2Body * body;
				b2BodyDef bodyDef;
				b2FixtureDef fixtureDef;
				bodyDef.type = b2_dynamicBody;
				b2PolygonShape fixture; //giving the point the shape of a box
				fixtureDef.shape = &fixture;
				fixture.SetAsBox(.001f, .001f); 
				//if (p.x !=0 && p.y!=0){
				bodyDef.position.Set(p.x, p.y); 
				body = world.CreateBody(&bodyDef);
				body->CreateFixture(&fixtureDef);
				//}
			}
		}
		break;

	}
}




};




 #endif