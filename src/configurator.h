#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include "opencv2/opencv.hpp"
//#include "box2d/box2d.h"
//#include "robot.h"
#include <dirent.h>
#include <vector>
#include <thread>
#include <filesystem>
#include <cmath>
#include <unistd.h>
#include <ncurses.h>
#include <fstream>
#include "task.h"
//#include "general.h" //general functions + point class + typedefs + Primitive.h + boost includes
#include <algorithm>
#include <sys/stat.h>

typedef b2Transform DeltaPose;

class ConfiguratorInterface{
public:
	bool debugOn=0;
	int iteration=0;
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
	bool benchmark=0;
public:
	ConfiguratorInterface * ci;
	bool running =0;
	std::thread * t=NULL;
	bool debugOn=0;
	float affineTransError =0;
	char *folder;
	char readMap[50];
	Task controlGoal;
	std::chrono::high_resolution_clock::time_point previousTimeScan;
	float timeElapsed =0;
	float totalTime=0;
	CoordinateContainer current, currentBox2D;
	bool planning =1;
	char statFile[100];
	char bodyFile[100];
	bool timerOff=0;
	int bodies=0;
	int treeSize = 0; //for debug
	Sequence plan;
	M_CODES numberOfM =THREE_M;
	GRAPH_CONSTRUCTION graphConstruction = BACKTRACKING;
	bool discretized =0;
Configurator()=default;

Configurator(Task _task, bool debug =0, bool noTimer=0): controlGoal(_task), currentTask(_task), debugOn(debug), timerOff(noTimer){
	previousTimeScan = std::chrono::high_resolution_clock::now();
	totalTime =0.0f;
}

void setBenchmarking(bool b){
	benchmark =b;
		if (benchmark){
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

void Spawner(CoordinateContainer, CoordinateContainer); 

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

void reactiveAvoidance(b2World &, simResult &, Task&); //adds two Tasks if crashed but always next up is picked

void evaluateNode(vertexDescriptor, CollisionGraph&, Task  , b2World &);

void backtrackingBuildTree(vertexDescriptor v, CollisionGraph&g, Task s, b2World & w, std::vector <vertexDescriptor>&); //builds the whole tree and finds the best solution

void DFIDBuildTree(vertexDescriptor, CollisionGraph&, Task, b2World &, vertexDescriptor &); //only expands after the most optimal node

void DFIDBuildTree_2(vertexDescriptor, CollisionGraph&, Task, b2World &, vertexDescriptor &); //evaluates only after DEFAULT, internal one step lookahead

void Astar(vertexDescriptor, CollisionGraph&, Task, b2World &, vertexDescriptor&); //proper A star implementation with discretixed space

std::vector <vertexDescriptor> splitNode(vertexDescriptor, CollisionGraph&, Direction, b2Transform);

std::pair <bool, Direction> getOppositeDirection(Direction);

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

bool addVertex(vertexDescriptor & src, vertexDescriptor &v1, CollisionGraph &g, Disturbance obs = Disturbance()){
	bool vertexAdded = false;
	if (g[src].options.size()>0){
		v1 = boost::add_vertex(g);
		edgeDescriptor e = add_edge(src, v1, g).first;
		g[e].direction =g[src].options[0];
		g[src].options.erase(g[src].options.begin());
		g[v1].totDs=g[src].totDs;
		g[v1].cost = g[src].cost;
		g[v1].disturbance = obs;
		if (g[e].direction==BACK){
			g[v1].twoStep =1;
		}
		vertexAdded=true;
	}
	return vertexAdded;
}

void removeIdleNodes(CollisionGraph&, vertexDescriptor, vertexDescriptor root=0);

Sequence getCleanSequence(CollisionGraph&, vertexDescriptor, vertexDescriptor root=0); //gets a sequence of summaries of successful tasks, excluding the root node

Sequence getUnprocessedSequence(CollisionGraph&, vertexDescriptor, vertexDescriptor root=0); //gets a sequence of summaries of successful tasks, excluding the root node

vertexDescriptor findBestLeaf(CollisionGraph &, std::vector <vertexDescriptor>, EndCriteria * refEnd = NULL);

EndedResult findError(Task, Node&); //returns whether the controlGoal has ended and fills node with cost and error

EndedResult findError(vertexDescriptor, CollisionGraph &, Direction); //finds error of task against the control goal adn its own cost (checks against itself)

// {
// 	//FIND BEST LEAF
// 	vertexDescriptor best = _leaves[0];
// 	for (vertexDescriptor leaf: _leaves){
// 		if (g[leaf].distanceSoFar>g[best].distanceSoFar){
// 			best = leaf;
// 		}
// 		else if (g[leaf].distanceSoFar==g[best].distanceSoFar){
// 			if (g[leaf].predecessors< g[best].predecessors){ //the fact that this leaf has fewer predecessors implies fewer collisions
// 				best = leaf;
// 			}
// 		}
// 	}
// 	// if (debugOn){
// 	// 	printf("best branch has endpose: x = %f, y= %f, angle = %f\n", g[best].endPose.p.x, g[best].endPose.p.y, g[best].endPose.q.GetAngle());
// 	// }
// 	return best;
// 	//FIND FIRST NODE BEFORE ORIGIN
// 	// std::vector <edgeDescriptor> bestEdges;
// 	// edgeDescriptor e;
// 	// while (best != *(boost::vertices(g).first)){
// 	// 	e = boost::in_edges(best, g).first.dereference();
// 	// 	bestEdges.push_back(e);
// 	// 	best = e.m_source;
// 	// }
// 	// if (!g[0].disturbance.safeForNow){
// 	// printf("plan to go: ");
// 	// for (auto eIt = bestEdges.rbegin(); eIt!=bestEdges.rend(); eIt++){
// 	// 	edgeDescriptor edge = *eIt;
// 	// 	switch (g[edge].direction){
// 	// 		case Direction::DEFAULT: printf("STRAIGHT, "); break;
// 	// 		case Direction::LEFT: printf("LEFT, "); break;
// 	// 		case Direction::RIGHT: printf("RIGHT, "); break;
// 	// 		case Direction::BACK: printf("BACK, "); break;
// 	// 		default: break;

// 	// 	}
// 	// }
// 	// }
// 	// printf("\n");
// 	// return e;
// }

Sequence getPlan(CollisionGraph &, vertexDescriptor);

void printPlan(Sequence);

void makeBody(b2World&, Point, bool&, Task* curr = NULL);

bool constructWorldRepresentation(b2World & world, Direction d, b2Transform start, Task * curr = NULL, bool discrete =0){
	//TO DO : calculate field of view: has to have 10 cm on each side of the robot
	bool obStillThere=0;
	if (NULL !=curr){
		if (curr->getAffIndex()!=PURSUE){
			obStillThere=0;
		}
		else {
			obStillThere=1;
		}
	}
	const float halfWindowWidth = .1;
	//printf("constructing\n");
	if ((d!=LEFT && d!=RIGHT)){ //IF THE ROBOT IS NOT TURNING
		std::vector <Point> bounds;
		float qBottomH, qTopH, qBottomP, qTopP, mHead, mPerp;
		float ceilingY, floorY, frontX, backX;
		float boxLength; 
		if (!discrete){
			boxLength=BOX2DRANGE;	
		}
		else{
			boxLength = DISCRETE_RANGE;
		}
		Point positionVector, radiusVector, maxFromStart; 
		if(d==BACK){
			float x = start.p.x - (SAFE_DISTANCE+ ROBOT_HALFLENGTH)* cos(start.q.GetAngle());
			float y = start.p.y - (SAFE_DISTANCE+ROBOT_HALFLENGTH)* sin(start.q.GetAngle());
			start = b2Transform(b2Vec2(x, y), b2Rot(start.q.GetAngle()));
			//boxLength += BACK_DISTANCE+ROBOT_HALFLENGTH;
			//printf("modified boxlength = %f, start x = %f, y= %f\n", boxLength, start.p.x, start.p.y);
		}
		radiusVector.polarInit(boxLength, start.q.GetAngle());
		//printf("radius vector = x=%f, y=%f\n", radiusVector.x, radiusVector.y);
		maxFromStart = Point(start.p) + radiusVector;
		//printf("max from start length = %f\n", maxFromStart.r);
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
		if (sin(start.q.GetAngle())!=0 && cos(start.q.GetAngle())!=0){
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
		for (auto p:currentBox2D){
			bool include;
			if (sin(start.q.GetAngle())!=0 && cos(start.q.GetAngle()!=0)){
				ceilingY = mHead*p.x +qTopH;
				floorY = mHead*p.x+qBottomH;
				float frontY= mPerp*p.x+qBottomP;
				float backY = mPerp*p.x+qTopP;
				//include = (p!= *(&p-1)&& p.y >=floorY && p.y<=ceilingY && p.y >=frontY && p.y<=backY);
				include = p.y >=floorY && p.y<=ceilingY && p.y >=frontY && p.y<=backY;
			}
			else{
				include = (p.y >=floorY && p.y<=ceilingY && p.x >=frontX && p.x<=backX);
			}
			if (include){
				makeBody(world, p, obStillThere, curr);
			}
		}
		}
		else{ //IF DIRECTION IS LEFT OR RIGHT 
		for (auto p:currentBox2D){
			if (p.isInRadius(start.p, ROBOT_HALFLENGTH -ROBOT_BOX_OFFSET_X)){ //y range less than 20 cm only to ensure that robot can pass + account for error
				makeBody(world, p, obStillThere, curr);
			}
		}
		

	}

		//DEBUG
		if (debugOn){
			FILE *f = fopen(bodyFile, "a+");
			for (b2Body * b = world.GetBodyList(); b!=NULL; b= b->GetNext()){
				fprintf(f, "%f\t%f\n", b->GetPosition().x, b->GetPosition().y);
			}
			fclose(f);
		}
		//END DEBUG


	return obStillThere;
}

void start(); //data interface class collecting position of bodies

void stop();

void registerInterface(ConfiguratorInterface *);

static void run(Configurator *);

void transitionMatrix(CollisionGraph&, vertexDescriptor, Direction); //DEFAULT, LEFT, RIGHT

//void transitionMatrix4M(CollisionGraph&, vertexDescriptor, Direction); //DEFAULT, LEFT, RIGHT, BACK

bool applyTransitionMatrix(CollisionGraph &, vertexDescriptor, Direction,bool, std::vector <vertexDescriptor> leaves = std::vector <vertexDescriptor>());

bool betterThanLeaves(CollisionGraph&, vertexDescriptor, std::vector <vertexDescriptor>, EndedResult &, Direction); //evaluation function

bool hasStickingPoint(CollisionGraph&, vertexDescriptor, EndedResult &);

void backtrack(CollisionGraph&, vertexDescriptor&);

void addToPriorityQueue(CollisionGraph&, vertexDescriptor, std::vector <vertexDescriptor>&);

std::pair <bool, b2Vec2> findNeighbourPoint(b2Vec2,float radius =0.025); //finds if there are bodies close to a point. Used for 

std::pair <bool, float>  findOrientation(b2Vec2, float radius = 0.025); //finds  average slope of line passign through two points in a radius of 2.5 cm. Assumes low clutter 
																		//and straight lines

};




 #endif