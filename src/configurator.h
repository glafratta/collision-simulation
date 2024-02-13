#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include "opencv2/opencv.hpp"
#include <dirent.h>
#include <vector>
#include <thread>
#include <filesystem>
#include <cmath>
#include <unistd.h>
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
	TaskSummary ts;

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
	Sequence plan;
	CollisionGraph collisionGraph;
	WorldBuilder worldBuilder;

Configurator()=default;

Configurator(Task _task, bool debug =0, bool noTimer=0): controlGoal(_task), currentTask(_task), debugOn(debug), timerOff(noTimer){
	motorStep(currentTask.getAction());
	previousTimeScan = std::chrono::high_resolution_clock::now();
	totalTime =0.0f;
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

bool Spawner(CoordinateContainer, CoordinateContainer); 

DeltaPose GetRealVelocity(CoordinateContainer &, CoordinateContainer &);

void controller();

Task * getTask(){ //returns Task being executed
	return &currentTask;
}

void applyController(bool, Task &);

int getIteration(){
	return iteration;
}


void reactiveAvoidance(b2World &, simResult &, Task&); //adds two Tasks if crashed but always next up is picked

simResult evaluateNode(vertexDescriptor, CollisionGraph&, Task  , b2World &);

void classicalAStar(vertexDescriptor, CollisionGraph&, Task, b2World &, vertexDescriptor &); //evaluates only after DEFAULT, internal one step lookahead

std::pair <bool, Direction> getOppositeDirection(Direction);

bool addVertex(vertexDescriptor & src, vertexDescriptor &v1, CollisionGraph &g, Disturbance obs = Disturbance()){
	if (!obs.isValid()){
		obs = controlGoal.disturbance;
	}
	bool vertexAdded = false;
	if (g[src].options.size()>0){
		v1 = boost::add_vertex(g);
		edgeDescriptor e = add_edge(src, v1, g).first;
		g[e].direction =g[src].options[0];
		g[src].options.erase(g[src].options.begin());
		g[v1].totDs=g[src].totDs;
		g[v1].disturbance = obs;
		vertexAdded=true;
	}
	return vertexAdded;
}

Sequence getCleanSequence(CollisionGraph&, vertexDescriptor, vertexDescriptor root=0); //gets a sequence of summaries of successful tasks, excluding the root node

EndedResult findError(Task, Node&); //returns whether the controlGoal has ended and fills node with cost and error

EndedResult findError(vertexDescriptor, CollisionGraph &, Direction); //finds error of task against the control goal adn its own cost (checks against itself)

Sequence getPlan(CollisionGraph &, vertexDescriptor);

void printPlan(Sequence);

void start(); //data interface class collecting position of bodies

void stop();

void registerInterface(ConfiguratorInterface *);

static void run(Configurator *);

void transitionMatrix(CollisionGraph&, vertexDescriptor, Direction); //DEFAULT, LEFT, RIGHT

bool applyTransitionMatrix(CollisionGraph &, vertexDescriptor, Direction,bool, std::vector <vertexDescriptor> leaves = std::vector <vertexDescriptor>());

void addToPriorityQueue(CollisionGraph&, vertexDescriptor, std::vector <vertexDescriptor>&);

void checkDisturbance(Point, bool&,Task * curr =NULL);

void trackTaskExecution(Task &);

DeltaPose assignDeltaPose(Task::Action, float);

void changeTask(bool, Sequence&, Node, int&);


int motorStep(Task::Action a);

void setSimulationStep(float f){
	simulationStep=f;
	worldBuilder.simulationStep=f;
}
};




 #endif