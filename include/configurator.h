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



class Configurator{
	private:
	//b2Vec2 defaultSpeed = {.125f, 0.0f}; //think the lidar is tilted 90deg
	float maxAbsSpeed = .2;
	std::vector <float> timeStamps;
	float samplingRate = 1.0f / 5.0f; //default
	int iteration=-1; //represents that hasn't started yet, robot isn't moving and there are no map data
	bool crashed=0;
public:
	//Action action;
	std::vector <cv::Point2f> previous;
	std::vector <cv::Point2f> current;
	//std::vector <float> turningAngles; //angles
	Action optimalAction;
	//bool openLoop=0;	
	char msg[25];
	State desiredState(GO);
	State currentState;

//calculuate displacement and angle using partial affine transformation
	b2Vec2 GetRealVelocity(b2Vec2);

// 1:  new scan available: box2d world is rebuilt with objects, current trajectory is checked

void NewScan(); 

void switchState(State, State); //current and previous?
//3: see if at the current velocity detected the robot will collide

	// void WillCollide(Action &action, b2World & _world) {  //for open loop
	// 	crashed =false;
	// 	Listener listener;
	// 	_world.SetContactListener(&listener);		
	// 	//b2Vec2 pointPrevious(0.0f, 0.0f);

		
	// 	Robot robot(&world);
	// 	robot.setAction(action);
	// //	printf("made box2d world and bodies: %i bodies\n", world.GetBodyCount());
	// 	char name_r[50];
	// 	sprintf(name_r, "/tmp/robot%04d_%i.txt", iteration, action.type);
	// 	FILE * robotPath = fopen(name_r, "a+");
	// 	for (int step = 0; step <= (hz*simDuration); step++) {//3 second
	// 		b2Vec2 start=robot.body->GetPosition();
	// 		action.setStep(step);
	// 		robot.setVelocity({action.x, action.y}); //instantaneous veloctiy
	// 		_world.Step(1.0f/hz, 3, 8); //time step 100 ms which also is alphabot callback time
	// 		fprintf(robotPath, "%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y);
	// 		if (listener.collision.isValid){ //
	// 				float maxDist = action.speed/hz; //maxdist in step, calculate time of impact
    //                 b2Vec2 distanceTravelled = listener.collision.where -start;
    //                 float toi = distanceTravelled.Length()/maxDist;
    //                 action.toi = step+toi;
	// 				crashed = 1;
	// 				printf("collision at: %f, %f\n", listener.collision.where.x, listener.collision.where.y);
	// 				//collisions.push_back(collision);
	// 				fclose(robotPath);
	// 			return;
	// 		}
	// 		action.toi=hz*simDuration;


	// 	}
	// 	//setPlan(_action); 		//this needs refinement as the thingy for the alphabot is hard-coded. coudl be, if one callback is 10d R or 13 to the L, get the angle from each speed in the plan and divide it
	// 	printf("path is safe\n");
	// 	fclose(robotPath);
	// }

	
	


};






 #endif