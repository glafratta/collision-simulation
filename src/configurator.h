#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include <dirent.h>
#include <vector>
#include <thread>
#include <filesystem>
#include <ncurses.h>
#include <fstream>
#include "worldbuilder.h"
#include "sensor.h"
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
	//TaskSummary ts;

	void setReady(bool b);

	bool isReady();
};

class Configurator{
protected:
	double samplingRate = 1.0/ 5.0; //default
	int iteration=0; //represents that hasn't started yet, robot isn't moving and there are no map data
	char fileNameBuffer[50];
	Task currentTask;
	bool benchmark=0;
public:
	ConfiguratorInterface * ci;
	bool running =0;
	std::thread * t=NULL;
	bool debugOn=0;
	float simulationStep=BOX2DRANGE;
	b2Transform ogGoal;
	Task controlGoal;
	std::chrono::high_resolution_clock::time_point previousTimeScan;
	float timeElapsed =0;
	//float totalTime=0;
	CoordinateContainer current, currentBox2D;
	bool planning =1;
	char statFile[100];
	char bodyFile[100];
	bool timerOff=0;
	int bodies=0;
	//int treeSize = 0; //for debug
	//Sequence plan;
	std::vector <vertexDescriptor> planVertices;
	StateMatcher matcher;
	//M_CODES numberOfM =THREE_M;
	//GRAPH_CONSTRUCTION graphConstruction = A_STAR;
	bool discretized =0;
	//PLAN_BUILD planBuild = STATIC;
	CollisionGraph collisionGraph;
	WorldBuilder worldBuilder;
	vertexDescriptor currentVertex;

Configurator()=default;

Configurator(Task _task, bool debug =0, bool noTimer=0): controlGoal(_task), currentTask(_task), debugOn(debug), timerOff(noTimer){
	//motorStep(currentTask.getAction());
	previousTimeScan = std::chrono::high_resolution_clock::now();
	//totalTime =0.0f;
	ogGoal=controlGoal.disturbance.pose;
	currentVertex = boost::add_vertex(collisionGraph);
}

void setBenchmarking(bool b){
	benchmark =b;
		if (benchmark){
		char dirName[50];
		sprintf(dirName, "benchmark");
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
		//sprintf(statFile,"stat");
		printf("%s\n", statFile);
		FILE * f = fopen(statFile, "w");
		printf("open\n");
		fclose(f);
	}
	printf("set\n");
}

bool Spawner(CoordinateContainer, CoordinateContainer); 

int getIteration(){
	return iteration;
}

DeltaPose GetRealVelocity(CoordinateContainer &, CoordinateContainer &);

//void controller();

void addIteration(){
	iteration++;
}

Task * getTask(int advance=0){ //returns Task being executed
	return &currentTask;
}


//void applyController(bool, Task &);


//b2Vec2 estimateDisplacementFromWheels();

//void reactiveAvoidance(b2World &, simResult &, Task&); //adds two Tasks if crashed but always next up is picked

simResult simulate(State&, State, Task, b2World &, float _simulationStep=BOX2DRANGE);

//void buildTree(vertexDescriptor, CollisionGraph&, Task, b2World &, vertexDescriptor &);

std::vector<std::pair<vertexDescriptor, vertexDescriptor>> propagateD(vertexDescriptor, vertexDescriptor, CollisionGraph&);

bool pruneTarget(std::vector<std::pair<vertexDescriptor, vertexDescriptor>>, CollisionGraph&, vertexDescriptor&); // deletes queued vertices, changes v if it will be afected by the deletion and returns a break signal

void updateGraph(CollisionGraph&);

void adjustStep(vertexDescriptor, CollisionGraph &, Direction, float&);

std::vector <edgeDescriptor> inEdgesRecursive(vertexDescriptor, CollisionGraph&, Direction ); //returns a vector of all in-edges leading to the vertex which have the same direction (most proximal first)

//b2Transform skip(edgeDescriptor&, CollisionGraph&, int&, Task*, float&); //inputs: plan, graph, index in the plan of the current vertex being checked. Returns the next index and the step distance to simulate

std::vector <edgeDescriptor> outEdges(CollisionGraph&, vertexDescriptor, Direction); //returns a vector containing all the out-edges of a vertex which have the specified direction

std::vector <edgeDescriptor> inEdges(CollisionGraph&, vertexDescriptor, Direction); //returns a vector containing all the in-edges of a vertex which have the specified direction

std::pair <edgeDescriptor, bool> maxProbability(std::vector<edgeDescriptor>, CollisionGraph&);

std::pair <bool, vertexDescriptor> findExactMatch(State, CollisionGraph&);

std::pair <bool, vertexDescriptor> findExactMatch(vertexDescriptor, CollisionGraph&); //has a safety to prevent matching a vertex with self

void changeStart(b2Transform&, vertexDescriptor, CollisionGraph&); //if task at vertex v fails, start is set to v's predecessor's end

bool edgeExists(vertexDescriptor, vertexDescriptor, CollisionGraph&);

//void backtrackingBuildTree(vertexDescriptor v, CollisionGraph&g, Task s, b2World & w, std::vector <vertexDescriptor>&); //builds the whole tree and finds the best solution

//void DFIDBuildTree(vertexDescriptor, CollisionGraph&, Task, b2World &, vertexDescriptor &); //only expands after the most optimal node

void explorer(vertexDescriptor, CollisionGraph&, Task, b2World &, vertexDescriptor &); //evaluates only after DEFAULT, internal one step lookahead

// void AlgorithmE(vertexDescriptor, CollisionGraph&, Task, b2World &, vertexDescriptor &); //evaluates only after DEFAULT, internal one step lookahead

// void onDemandAStar(vertexDescriptor, CollisionGraph&, Task, b2World &, vertexDescriptor&); //proper A star implementation with discretixed space

//std::vector <vertexDescriptor> splitNode(vertexDescriptor, CollisionGraph&, Direction, b2Transform);

std::pair <bool, Direction> getOppositeDirection(Direction);

bool addVertex(vertexDescriptor & src, vertexDescriptor &v1, CollisionGraph &g, Disturbance obs = Disturbance(),Direction d=DEFAULT, bool topDown=0){
	// if (!obs.isValid()){
	// 	obs = controlGoal.disturbance;
	// }
	bool vertexAdded = false;
	if (g[src].options.size()>0 || topDown){
		v1 = boost::add_vertex(g);
		edgeDescriptor e = add_edge(src, v1, g).first;
		if (!topDown){
			// g[e].direction =g[src].options[0];
			g[e].direction =d;
			g[src].options.erase(g[src].options.begin());
		}
		else{
			g[e].direction=d;
		}
		g[v1].totDs=g[src].totDs;
		if (!g[v1].filled){
			g[v1].disturbance = obs;
		}
		vertexAdded=true;
		adjustProbability(g, e); //for now predictions and observations carry the same weight
	}
	return vertexAdded;
}

void adjustProbability(CollisionGraph &, edgeDescriptor&);

//void removeIdleNodes(CollisionGraph&, vertexDescriptor, vertexDescriptor root=0);

//Sequence getCleanSequence(CollisionGraph&, vertexDescriptor, vertexDescriptor root=0); //gets a sequence of summaries of successful tasks, excluding the root node

std::vector <vertexDescriptor> planner(CollisionGraph&, vertexDescriptor, vertexDescriptor root=0);

//Sequence getUnprocessedSequence(CollisionGraph&, vertexDescriptor, vertexDescriptor root=0); //gets a sequence of summaries of successful tasks, excluding the root node

//vertexDescriptor findBestLeaf(CollisionGraph &, std::vector <vertexDescriptor>, vertexDescriptor, EndCriteria * refEnd = NULL);

EndedResult estimateCost(State&, b2Transform, Direction); //returns whether the controlGoal has ended and fills node with cost and error

EndedResult estimateCost(vertexDescriptor, CollisionGraph &, Direction); //finds error of task against the control goal adn its own cost (checks against itself)

//void loopGoal();

float evaluationFunction(EndedResult);

//Sequence getPlan(CollisionGraph &, vertexDescriptor);

//void printPlan(Sequence);

void start(); //data interface class collecting position of bodies

void stop();

void registerInterface(ConfiguratorInterface *);

static void run(Configurator *);

void transitionMatrix(State&, Direction); //DEFAULT, LEFT, RIGHT

//void transitionMatrix4M(CollisionGraph&, vertexDescriptor, Direction); //DEFAULT, LEFT, RIGHT, BACK

void applyTransitionMatrix(CollisionGraph&, vertexDescriptor, Direction,bool);

//bool betterThanLeaves(CollisionGraph&, vertexDescriptor, std::vector <vertexDescriptor>, EndedResult &, Direction); //evaluation function

//bool hasStickingPoint(CollisionGraph&, vertexDescriptor, EndedResult &);

//void backtrack(CollisionGraph&, vertexDescriptor&);

void addToPriorityQueue(vertexDescriptor, std::vector <std::pair<vertexDescriptor, float>>&, float phi=0);

std::pair <bool, b2Vec2> findNeighbourPoint(b2Vec2,float radius =0.025); //finds if there are bodies close to a point. Used for 

std::pair <bool, float>  findOrientation(b2Vec2, float radius = 0.025); //finds  average slope of line passign through two points in a radius of 2.5 cm. Assumes low clutter 
																		//and straight lines
//void checkDisturbance(Point, bool&,Task * curr =NULL);

std::vector <vertexDescriptor> checkPlan(b2World&, std::vector <vertexDescriptor> &, CollisionGraph&, b2Transform start=b2Transform(b2Vec2(0,0), b2Rot(0))); //returns if plan fails and at what index in the plan

//std::vector <vertexDescriptor> (vertexDescriptor);
									
void trackTaskExecution(Task &);

DeltaPose assignDeltaPose(Task::Action, float);

//void changeTask(bool, Sequence&, State, int&);

void changeTask(bool, int&);

//int motorStep(Task::Action, EndCriteria);

int motorStep(Task::Action a);

void setSimulationStep(float f){
	simulationStep=f;
	worldBuilder.simulationStep=f;
}
};




 #endif