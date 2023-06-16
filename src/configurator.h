#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include "opencv2/opencv.hpp"
#include "Box2D/Box2D.h"
//#include "robot.h"
#include <dirent.h>
#include <vector>
#include <thread>
#include <filesystem>
#include <cmath>
#include <unistd.h>
#include <ncurses.h>
#include <fstream>
//#include "task.h"
#include "general.h" //general functions + point class + typedefs + Primitive.h + boost includes
#include <algorithm>
#include <sys/stat.h>
class ConfiguratorInterface{
public:
	bool debugOn=0;
	CoordinateContainer data;
	CoordinateContainer data2fp;
	bool ready=0;

	void setReady(bool b);

	bool isReady();
};

class Configurator{
protected:
	double samplingRate = 1.0/ 5.0; //default
	int iteration=0; //represents that hasn't started yet, robot isn't moving and there are no map data
	b2Vec2 absPosition = {0.0f, 0.0f};
	FILE * dumpPath;
	char fileNameBuffer[50];
	Task currentTask;
public:
	ConfiguratorInterface * ci;
	bool running =0;
	std::thread * t=NULL;
	bool debugOn=0;
	float affineTransError =0;
	char *folder;
	char readMap[50];
	Task desiredTask;
	std::chrono::high_resolution_clock::time_point previousTimeScan;
	float timeElapsed =0;
	float totalTime=0;
	CoordinateContainer current;
	CoordinateContainer currentBox2D;
	bool planning =1;
	char statFile[100];
	bool timerOff=0;
	int bodies=0;

Configurator(){
	previousTimeScan = std::chrono::high_resolution_clock::now();
	totalTime = 0.0f;
	currentTask = desiredTask;
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
		fclose(f);
	}
}

Configurator(Task &_task, bool debug =0, bool noTimer=0): desiredTask(_task), currentTask(_task), debugOn(debug), timerOff(noTimer){
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
}


void Spawner(CoordinateContainer &, CoordinateContainer &); 

int getIteration(){
	return iteration;
}

DeltaPose GetRealVelocity(CoordinateContainer &, CoordinateContainer &);

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

Task * getTask(int advance=0){ //returns Task being executed
	return &currentTask;
}


void applyController(bool, Task &);


b2Vec2 estimateDisplacementFromWheels();

void reactiveAvoidance(b2World &, Task::simResult &, Task&); //adds two Tasks if crashed but always next up is picked

vertexDescriptor nextNode(vertexDescriptor, CollisionGraph&, Task  , b2World & , std::vector <vertexDescriptor> &);

bool build_tree(vertexDescriptor v, CollisionGraph&g, Task s, b2World & w, std::vector <vertexDescriptor>&);

Direction getOppositeDirection(Direction d){
    switch (d){
        case Direction::LEFT: return Direction::RIGHT;break;
        case Direction::RIGHT: return Direction::LEFT;break;
		//case Direction::DEFAULT: return Direction::BACK; break;
        default:
        return Direction::DEFAULT;
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

void addVertex(vertexDescriptor & src, vertexDescriptor &v1, CollisionGraph &g, Task::Disturbance obs = Task::Disturbance()){
	if (g[src].options.size()>0){
		v1 = boost::add_vertex(g);
		edgeDescriptor e = add_edge(src, v1, g).first;
		g[e].direction =g[src].options[0];
		g[src].options.erase(g[src].options.begin());
		g[v1].totDs=g[src].totDs;
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
	if (currentTask.change){
	printf("plan to go: ");
	for (auto eIt = bestEdges.rbegin(); eIt!=bestEdges.rend(); eIt++){
		edgeDescriptor edge = *eIt;
		switch (g[edge].direction){
			case Direction::DEFAULT: printf("STRAIGHT, "); break;
			case Direction::LEFT: printf("LEFT, "); break;
			case Direction::RIGHT: printf("RIGHT, "); break;
			case Direction::BACK: printf("BACK, "); break;
			default: break;

		}
	}
	}
	printf("\n");
	
	return e;
}

bool constructWorldRepresentation(b2World & world, Direction d, b2Transform start, Task * curr = NULL){
	//TO DO : calculate field of view: has to have 10 cm on each side of the robot
	bool obStillThere=0;
	const float halfWindowWidth = .1;
	switch (d){
		case Direction::DEFAULT:{
		std::vector <Point> bounds;
		float qBottomH, qTopH, qBottomP, qTopP, mHead, mPerp;
		float ceilingY, floorY, frontX, backX;		
		Point positionVector, radiusVector, maxFromStart; 
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
			ceilingY = std::max(top.y, bottom.y); 
			floorY = std::min(top.y, bottom.y); 
			frontX = std::min(top.x, bottom.x);
			backX = std::max(top.x, bottom.x);
		}
		//CREATE POINTS

		// for (auto _p = currentBox2D.begin(); _p!= currentBox2D.end(); ++_p){	
		// 	auto p = *_p;
		for (auto p:currentBox2D){
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
					if (curr->disturbance.isValid()){
						if (p.isInRadius(currentTask.disturbance.getPosition())){
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
		// for (auto _p = currentBox2D.begin(); _p!= currentBox2D.end(); ++_p){	
		// 	auto p = *_p;
		for (auto p:currentBox2D){
			if (p != *(&p-1)&& p.isInRadius(start.p, halfWindowWidth)){ //y range less than 20 cm only to ensure that robot can pass + account for error
				b2Body * body;
				b2BodyDef bodyDef;
				b2FixtureDef fixtureDef;
				bodyDef.type = b2_dynamicBody;
				b2PolygonShape fixture; //giving the point the shape of a box
				fixtureDef.shape = &fixture;
				fixture.SetAsBox(.001f, .001f); 
				if (curr !=NULL){ //
					if (curr->disturbance.isValid()){
						if (p.isInRadius(currentTask.disturbance.getPosition())){
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
			if (curr->disturbance.isValid()){
				if (p.isInRadius(currentTask.disturbance.getPosition()) & p !=*(&p-1)){
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
	return obStillThere;
}

void addOptionsToNode(CollisionGraph &, vertexDescriptor &); //adds options for the robot to travel only straight, left and right

void start(); //data interface class collecting position of bodies

void stop();

void registerInterface(ConfiguratorInterface *);

static void run(Configurator *);

void makeBody(b2World &, b2Vec2, int); //takes world, position and body count

void snapNodes();


};




 #endif