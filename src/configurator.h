#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include <dirent.h>
#include <thread>
#include <filesystem>
#include <ncurses.h>
#include <fstream>
#include "worldbuilder.h"
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
	CoordinateContainer current, currentBox2D;
	bool planning =1;
	char statFile[100];
	char bodyFile[100];
	bool timerOff=0;
	int bodies=0;
	SensorTools sensorTools;
	std::vector <vertexDescriptor> planVertices;
	bool discretized =0;
	TransitionSystem transitionSystem;
	//Model model;
	StateMatcher matcher;
	WorldBuilder worldBuilder;
	vertexDescriptor currentVertex;

Configurator()=default;

Configurator(Task _task, bool debug =0, bool noTimer=0): controlGoal(_task), currentTask(_task), debugOn(debug), timerOff(noTimer){
	previousTimeScan = std::chrono::high_resolution_clock::now();
	ogGoal=controlGoal.disturbance.pose;
	currentVertex = boost::add_vertex(transitionSystem);
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

//DeltaPose GetRealVelocity(CoordinateContainer &, CoordinateContainer &);

void addIteration(){
	iteration++;
}

Task * getTask(int advance=0){ //returns Task being executed
	return &currentTask;
}

simResult simulate(State&, State, Task, b2World &, float _simulationStep=BOX2DRANGE);

std::vector<std::pair<vertexDescriptor, vertexDescriptor>> propagateD(vertexDescriptor, vertexDescriptor, vertexDescriptor&, TransitionSystem&);

void pruneEdges(std::vector<std::pair<vertexDescriptor, vertexDescriptor>>, TransitionSystem&, vertexDescriptor&, std::vector <std::pair<vertexDescriptor, float>>, std::vector<vertexDescriptor>&); //clears edges out of redundant vertices, removes the vertices from PQ, returns vertices to remove at the end

void removeVertices(std::vector<vertexDescriptor>, TransitionSystem&);

void updateGraph(TransitionSystem&);

void adjustStepDistance(vertexDescriptor, TransitionSystem &, Direction, float&);

std::vector <edgeDescriptor> inEdgesRecursive(vertexDescriptor, TransitionSystem&, Direction ); //returns a vector of all in-edges leading to the vertex which have the same direction (most proximal first)

std::vector <edgeDescriptor> outEdges(TransitionSystem&, vertexDescriptor, Direction); //returns a vector containing all the out-edges of a vertex which have the specified direction

std::vector <edgeDescriptor> inEdges(TransitionSystem&, vertexDescriptor, Direction); //returns a vector containing all the in-edges of a vertex which have the specified direction

std::pair <edgeDescriptor, bool> maxProbability(std::vector<edgeDescriptor>, TransitionSystem&);

std::pair <bool, vertexDescriptor> findExactMatch(State, TransitionSystem&);

std::pair <bool, vertexDescriptor> findExactMatch(vertexDescriptor, TransitionSystem&); //has a safety to prevent matching a vertex with self

void changeStart(b2Transform&, vertexDescriptor, TransitionSystem&); //if task at vertex v fails, start is set to v's predecessor's end

bool edgeExists(vertexDescriptor, vertexDescriptor, TransitionSystem&);

//void backtrackingBuildTree(vertexDescriptor v, TransitionSystem&g, Task s, b2World & w, std::vector <vertexDescriptor>&); //builds the whole tree and finds the best solution

//void DFIDBuildTree(vertexDescriptor, TransitionSystem&, Task, b2World &, vertexDescriptor &); //only expands after the most optimal node

std::vector<vertexDescriptor> explorer(vertexDescriptor, TransitionSystem&, Task, b2World &, vertexDescriptor &); //evaluates only after DEFAULT, internal one step lookahead

std::pair <bool, Direction> getOppositeDirection(Direction);

bool addVertex(vertexDescriptor & src, vertexDescriptor &v1, TransitionSystem &g, Disturbance obs = Disturbance(),Direction d=DEFAULT, bool topDown=0){
	bool vertexAdded = false;
	if (g[src].options.size()>0 || topDown){
		v1 = boost::add_vertex(g);
		edgeDescriptor e = add_edge(src, v1, g).first;
		if (!topDown){
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

void adjustProbability(TransitionSystem &, edgeDescriptor&);

std::vector <vertexDescriptor> planner(TransitionSystem&, vertexDescriptor, vertexDescriptor root=0);

EndedResult estimateCost(State&, b2Transform, Direction); //returns whether the controlGoal has ended and fills node with cost and error

EndedResult estimateCost(vertexDescriptor, TransitionSystem &, Direction); //finds error of task against the control goal adn its own cost (checks against itself)

float evaluationFunction(EndedResult);

void start(); //data interface class collecting position of bodies

void stop();

void registerInterface(ConfiguratorInterface *);

static void run(Configurator *);

void transitionMatrix(State&, Direction); //DEFAULT, LEFT, RIGHT

void applyTransitionMatrix(TransitionSystem&, vertexDescriptor, Direction,bool);

void addToPriorityQueue(vertexDescriptor, std::vector <std::pair<vertexDescriptor, float>>&, float phi=0);

std::pair <bool, b2Vec2> findNeighbourPoint(b2Vec2,float radius =0.025); //finds if there are bodies close to a point. Used for 

std::pair <bool, float>  findOrientation(b2Vec2, float radius = 0.025); //finds  average slope of line passign through two points in a radius of 2.5 cm. Assumes low clutter 
																		//and straight lines

std::vector <vertexDescriptor> checkPlan(b2World&, std::vector <vertexDescriptor> &, TransitionSystem&, b2Transform start=b2Transform(b2Vec2(0,0), b2Rot(0))); //returns if plan fails and at what index in the plan
									
void trackTaskExecution(Task &);

DeltaPose assignDeltaPose(Task::Action, float);

void changeTask(bool, int&);

int motorStep(Task::Action a);

void setSimulationStep(float f){
	simulationStep=f;
	worldBuilder.simulationStep=f;
}
};




 #endif