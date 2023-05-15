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
#include "primitive.h"
#include "general.h" //general functions + point class + typedefs + Primitive.h + boost includes
//#include <boost/graph/depth_first_search.hpp>
#include <algorithm>
#include<sys/stat.h>



class Configurator{
protected:
	//std::vector <float> timeStamps;
	double samplingRate = 1.0/ 5.0; //default
	int iteration=0; //represents that hasn't started yet, robot isn't moving and there are no map data
	b2Vec2 desiredVelocity;
	b2Vec2 absPosition = {0.0f, 0.0f};
	FILE * dumpPath;
	//FILE * dumpDeltaV;
	char fileNameBuffer[50];
	Primitive currentDMP;
public:
	bool debugOn=0;
	float affineTransError =0;
//	bool filterOn=1;
	char *folder;
	char readMap[50];
//	char msg[25];
	Primitive desiredDMP;
	std::chrono::high_resolution_clock::time_point previousTimeScan;
	float timeElapsed =0;
	float totalTime=0;
	std::vector <Point> current, currentBox2D;
	bool planning =1;
	char statFile[100];
	bool timerOff=0;
	int bodies=0;


	struct getVelocityResult{
		bool valid =0;
		b2Transform vector;
		b2Transform affineResult;
		//float angle;
		getVelocityResult(){}
		getVelocityResult(b2Transform disp):vector(disp){
			valid=1;
		}

	};

//calculuate displacement and angle using partial affine transformation

// 1:  new scan available: box2d world is rebuilt with objects, current Action is checked#

Configurator(){
	printf("hi\n");
	previousTimeScan = std::chrono::high_resolution_clock::now();
	totalTime = 0.0f;
	currentDMP = desiredDMP;
	//dumpDeltaV = fopen("/tmp/deltaV.txt", "w");
	if (debugOn){
	char dirName[50];
	sprintf(dirName, "bodiesSpeedStats");
	if (!opendir(dirName)){
		mkdir(dirName, 0777);
		printf("made stats directory\n");
	}
	else{
		printf("opened stats directory\n");
	}
	//TODAYS DATE AND TIME
		time_t now =time(0);
		tm *ltm = localtime(&now);
		int y,m,d, h, min;
		y=ltm->tm_year-100;
		m = ltm->tm_mon +1;
		d=ltm->tm_mday;
		h= ltm->tm_hour;
		min = ltm->tm_min;
		sprintf(statFile, "%s/stats%02i%02i%02i_%02i%02i.txt",dirName, d,m,y,h,min);
		FILE * f = fopen(statFile, "w");
		//fprintf(f,"Bodies\tBranches\tTime\n");
		fclose(f);
}

}

Configurator(Primitive &_dmp, bool debug =0, bool noTimer=0): desiredDMP(_dmp), currentDMP(_dmp), debugOn(debug), timerOff(noTimer){
	printf("hey\n");
	previousTimeScan = std::chrono::high_resolution_clock::now();
	totalTime =0.0f;
	if (debugOn){
		char dirName[50];
		sprintf(dirName, "bodiesSpeedStats");
		if (!opendir(dirName)){
			mkdir(dirName, 0777);
			printf("made stats directory\n");
		}
		else{
			printf("opened stats directory\n");
		}
		//TODAYS DATE AND TIME
		time_t now =time(0);
		tm *ltm = localtime(&now);
		int y,m,d, h, min;
		y=ltm->tm_year-100;
		m = ltm->tm_mon +1;
		d=ltm->tm_mday;
		h= ltm->tm_hour;
		min = ltm->tm_min;
		sprintf(statFile, "%s/stats%02i%02i%02i_%02i%02i.txt",dirName, d,m,y,h,min);
		printf("%s\n", statFile);
		FILE * f = fopen(statFile, "w");
		fclose(f);
	}


	//printf("simduration = %i\n", currentDMP.getSimDuration());

}

void __init__(){
}

void __init__(Primitive & _currentDMP){
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

void setNameBuffer(char * str){ //set name of file from which to read trajectories. by default trajectories are dumped by 'currentDMP' into a robot000n.txt file.
								//changing this does not change where trajectories are dumped, but if you want the robot to follow a different trajectory than the one created by the currentDMP
	sprintf(fileNameBuffer, "%s", str);
	//printf("reading trajectories from: %s\n", fileNameBuffer);
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
//Configurator::getVelocityResult GetVelocityFromReference(std::vector <Point> &, std::vector <Point> &);

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


Primitive * getDMP(int advance=0){ //returns Primitive being executed
	return &currentDMP;
}


void applyController(bool, Primitive &);


b2Vec2 estimateDisplacementFromWheels();

int getMaxStepDuration(){
	return currentDMP.hz * currentDMP.getSimDuration();
}

// Primitive returnBest(Primitive & s1, Primitive & s2){ //returns pointer, remember to dereference
// 	switch(s1.getType() == s2.getType()){
// 	//if one of the Primitive is the desired Primitive, return the desired Primitive
// 		case 0: 
// 			if (s1.getType() == desiredDMP.getType()){
// 				return s1;
// 			}
// 			else if (s2.getType() == desiredDMP.getType()){
// 				return s2;
// 			}
// 		break;
// 		case 1:
// 			switch (s1.getType()== desiredDMP.getType()){
// 				//if both Primitives are avoiding, choose the one with the least duration
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
// 				//if both Primitives are desired, choose the one with the most
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

//void reactiveAvoidance(b2World &, Primitive &, b2Vec2&, float&, Primitive::Direction); //performs reactive avoidance

void reactiveAvoidance(b2World &, Primitive::simResult &, Primitive&, b2Vec2 &, float &); //adds two Primitives if crashed but always next up is picked

//vertexDescriptor eliminateDisturbance(b2World & world, vertexDescriptor v, Graph &g);

vertexDescriptor eliminateDisturbance(vertexDescriptor, CollisionGraph&, Primitive  , b2World & , std::vector <vertexDescriptor> &);
bool build_tree(vertexDescriptor v, Graph&g, b2World & w);
bool build_tree(vertexDescriptor v, CollisionGraph&g, Primitive s, b2World & w, std::vector <vertexDescriptor>&);



Primitive::Direction getOppositeDirection(Primitive::Direction d){
    switch (d){
        case Primitive::Direction::LEFT: return Primitive::Direction::RIGHT;break;
        case Primitive::Direction::RIGHT: return Primitive::Direction::LEFT;break;
        default:
        return Primitive::Direction::NONE; //printf("default direction\n");
		break;
    }
}

template <typename V, typename G>
bool isFullLength(V v, const G & g, float length=0){
    if (boost::in_degree(v, g)==0 && length < BOX2DRANGE){
        return false;
    }
    else if (length >=BOX2DRANGE){
        return true;
    }
    else{
        edgeDescriptor inEdge= boost::in_edges(v, g).first.dereference();
        length += g[inEdge].distanceCovered;
		g[v].predecessors++;
        return isFullLength(boost::source(inEdge, g), g, length);
    }

}

//FOR NEW CollisionGraph
void addVertex(vertexDescriptor & src, vertexDescriptor &v1, CollisionGraph &g, Primitive::Object obs = Primitive::Object()){
	if (g[src].options.size()>0){
		v1 = boost::add_vertex(g);
		edgeDescriptor e = add_edge(src, v1, g).first;
		g[e].direction =g[src].options[0];
		g[src].options.erase(g[src].options.begin());
		g[v1].totObstacles=g[src].totObstacles;
	}
}

edgeDescriptor findBestBranch(CollisionGraph &g, std::vector <vertexDescriptor> _leaves){
	//FIND BEST LEAF
	vertexDescriptor best = _leaves[0];
	for (vertexDescriptor leaf: _leaves){
		if (g[leaf].distanceSoFar>g[best].distanceSoFar){
			best = leaf;
		}
		else if (g[leaf].distanceSoFar==g[best].distanceSoFar){
			if (g[leaf].predecessors< g[best].predecessors){ //the fact that this leaf has fewer predecessors implies fewer collisions
				best = leaf;
			}
		}
	}
	if (debugOn){
		printf("best branch has endpose: x = %f, y= %f, angle = %f\n", g[best].endPose.p.x, g[best].endPose.p.y, g[best].endPose.q.GetAngle());
	}
	//FIND FIRST NODE BEFORE ORIGIN
	std::vector <edgeDescriptor> bestEdges;
	edgeDescriptor e;
	while (best != *(boost::vertices(g).first)){
		e = boost::in_edges(best, g).first.dereference();
		bestEdges.push_back(e);
		best = e.m_source;
	}
	//std::sort(bestEdges.begin(), bestEdges.end());
	if (currentDMP.change){
	printf("plan to go: ");
	for (auto eIt = bestEdges.rbegin(); eIt!=bestEdges.rend(); eIt++){
		edgeDescriptor edge = *eIt;
		switch (g[edge].direction){
			case Primitive::NONE: printf("STRAIGHT, "); break;
			case Primitive::LEFT: printf("LEFT, "); break;
			case Primitive::RIGHT: printf("RIGHT, "); break;
			default: break;

		}
	}
	}
	printf("\n");
	
	return e;
}

bool constructWorldRepresentation(b2World & world, Primitive::Direction d, b2Transform start, Primitive * curr = NULL){
	//TO DO : calculate field of view: has to have 10 cm on each side of the robot
	bool obStillThere=0;
	const float halfWindowWidth = .1;
	switch (d){
		case Primitive::Direction::NONE:{
		std::vector <Point> bounds;
		float qBottomH, qTopH, qBottomP, qTopP, mHead, mPerp;
		float ceilingY, floorY, frontX, backX;		
		Point positionVector, radiusVector, maxFromStart; 
		//float minX, minY, maxX, maxY;
		radiusVector.polarInit(BOX2DRANGE, start.q.GetAngle());
		maxFromStart = Point(start.p) + radiusVector;
		//FIND THE BOUNDS OF THE BOX
		b2Vec2 unitPerpR(-sin(start.q.GetAngle()), cos(start.q.GetAngle()));
		b2Vec2 unitPerpL(sin(start.q.GetAngle()), -cos(start.q.GetAngle()));
		bounds.push_back(Point(start.p)+Point(unitPerpL.x *halfWindowWidth, unitPerpL.y*halfWindowWidth));
		bounds.push_back(Point(start.p)+Point(unitPerpR.x *halfWindowWidth, unitPerpR.y*halfWindowWidth));
		bounds.push_back(maxFromStart+Point(unitPerpL.x *halfWindowWidth, unitPerpL.y*halfWindowWidth)); 
		bounds.push_back(maxFromStart+Point(unitPerpR.x *halfWindowWidth, unitPerpR.y*halfWindowWidth));
		Point top, bottom;
		comparator compPoint;
		std::sort(bounds.begin(), bounds.end(), compPoint); //sort bottom to top
		bottom = bounds[0];
		top = bounds[3];
		if (sin(start.q.GetAngle())!=0 && cos(start.q.GetAngle()!=0)){
			//FIND PARAMETERS OF THE LINES CONNECTING THE a
			mHead = sin(start.q.GetAngle())/cos(start.q.GetAngle()); //slope of heading direction
			mPerp = -1/mHead;
			qBottomH = bottom.y - mHead*bottom.x;
			qTopH = top.y - mHead*top.x;
			qBottomP = bottom.y -mPerp*bottom.x;
			qTopP = top.y - mPerp*top.x;
		}
		else{
			ceilingY = std::max(top.y, bottom.y); //top.y
			floorY = std::min(top.y, bottom.y); //bottom.y
			frontX = std::min(top.x, bottom.x);
			backX = std::max(top.x, bottom.x);
		}
		//CREATE POINTS

		for (Point &p:currentBox2D){
			bool include;
			if (sin(start.q.GetAngle())!=0 && cos(start.q.GetAngle()!=0)){
				ceilingY = mHead*p.x +qTopH;
				floorY = mHead*p.x+qBottomH;
				float frontY= mPerp*p.x+qBottomP;
				float backY = mPerp*p.x+qTopP;
				include = (p!= *(&p-1)&& p.y >=floorY && p.y<=ceilingY && p.y >=frontY && p.y<=backY);
			}
			else{
				include = (p!= *(&p-1)&& p.y >=floorY && p.y<=ceilingY && p.x >=frontX && p.x<=backX);
			}
			if (include){
				b2Body * body;
				b2BodyDef bodyDef;
				b2FixtureDef fixtureDef;
				bodyDef.type = b2_dynamicBody;
				b2PolygonShape fixture; //giving the point the shape of a box
				fixtureDef.shape = &fixture;
				fixture.SetAsBox(.001f, .001f); 
				if (curr !=NULL){ //
					if (curr->obstacle.isValid()){
						if (p.isInRadius(currentDMP.obstacle.getPosition())){
							obStillThere =1;
						}
					}
				}
				bodyDef.position.Set(p.x, p.y); 
				bodies++;
				body = world.CreateBody(&bodyDef);
				body->CreateFixture(&fixtureDef);

			}
		}
		break;
		}
		default:
		for (Point &p:currentBox2D){	
			if (p != *(&p-1)&& p.isInRadius(start.p, halfWindowWidth)){ //y range less than 20 cm only to ensure that robot can pass + account for error
				b2Body * body;
				b2BodyDef bodyDef;
				b2FixtureDef fixtureDef;
				bodyDef.type = b2_dynamicBody;
				b2PolygonShape fixture; //giving the point the shape of a box
				fixtureDef.shape = &fixture;
				fixture.SetAsBox(.001f, .001f); 
				if (curr !=NULL){ //
					if (curr->obstacle.isValid()){
						if (p.isInRadius(currentDMP.obstacle.getPosition())){
							obStillThere =1;
						}
					}
				}
				bodyDef.position.Set(p.x, p.y); 
				body = world.CreateBody(&bodyDef);
				bodies++;
				body->CreateFixture(&fixtureDef);
			}
			else if (curr!=NULL){
			if (curr->obstacle.isValid()){
				if (p.isInRadius(currentDMP.obstacle.getPosition()) & p !=*(&p-1)){
					obStillThere =1;
					b2Body * body;
					b2BodyDef bodyDef;
					b2FixtureDef fixtureDef;
					bodyDef.type = b2_dynamicBody;
					b2PolygonShape fixture; //giving the point the shape of a box
					fixtureDef.shape = &fixture;
					fixture.SetAsBox(.001f, .001f); 
					bodyDef.position.Set(p.x, p.y); 
					body = world.CreateBody(&bodyDef);
					bodies++;
					body->CreateFixture(&fixtureDef);
				}
			}
		}
		}
		break;

	}
	//bodies+=world.GetBodyCount();
	return obStillThere;
}




};




 #endif