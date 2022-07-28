#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H

#include "Box2D/Box2D.h"
#include "robot.h"
#include <vector>
#include "opencv2/opencv.hpp"
#include <thread>
#include <cmath>
#include <unistd.h>
#include <ncurses.h>
#include <fstream>
#include "state.h"
#include <time.h>

//class State;

class Configurator{
	private:
	float maxAbsSpeed = .2;
	std::vector <float> timeStamps;
	double samplingRate = 1.0/ 5.0; //default
	int iteration=-1; //represents that hasn't started yet, robot isn't moving and there are no map data
	bool crashed=0;
	std::chrono::high_resolution_clock::time_point previousTimeScan;
public:
	std::vector <cv::Point2f> previous;
	std::vector <cv::Point2f> current;
	char msg[25];
	std::vector <State> plan; //from here we can find the current state
	State desiredState;
	

	struct getVelocityResult{
		bool valid =0;
		b2Vec2 displacement;
		float angle;
		getVelocityResult(){}
		getVelocityResult(b2Vec2 disp):displacement(disp){
			valid=1;
			angle= atan2(disp.y, disp.x);
		}
	};

//calculuate displacement and angle using partial affine transformation

// 1:  new scan available: box2d world is rebuilt with objects, current trajectory is checked#

Configurator(State _state): desiredState(_state){
	previousTimeScan = std::chrono::high_resolution_clock::now();
}

void NewScan(); 

Configurator::getVelocityResult GetRealVelocity();

};






 #endif