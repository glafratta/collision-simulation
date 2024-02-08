#ifndef ROBOT_H
#define ROBOT_H
//box2d robot body and kinematic model
#include "box2d/box2d.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define SAFE_ANGLE M_PI_2
#define SAFE_DISTANCE sqrt(ROBOT_HALFLENGTH*ROBOT_HALFLENGTH+ROBOT_HALFWIDTH*ROBOT_HALFWIDTH) //DEFAULT backtracking distance
#define MAX_TURN M_PI
#define ROBOT_HALFLENGTH 0.11 //uncomment for robot
#define ROBOT_HALFWIDTH 0.09 //real
//#define ROBOT_HALFLENGTH 0.05 //debug value
//#define ROBOT_HALFWIDTH 0.01 //debug value
#define ROBOT_BOX_OFFSET_X -0.05
//#define ROBOT_BOX_OFFSET_X 0
#define ROBOT_BOX_OFFSET_Y 0
#define ROBOT_BOX_OFFSET_ANGLE 0
#define BETWEEN_WHEELS .15
#define MAX_SPEED 0.156
#define MAX_OMEGA 2 //2 radians
#define ANGLE_ERROR_TOLERANCE 5 * M_PI/180
const float BOX2DRANGE =1.0;
#define LIDAR_RANGE 1.1
#define REACTION_TIME 1.0
#define HZ 60.0
#define MAX_ANGLE_ERROR M_PI
const float MAX_DISTANCE_ERROR =2*BOX2DRANGE;
//const float DISCRETE_RANGE =ROBOT_HALFLENGTH*2; //22 cm, as in Ulrich (2000)
//#define DISCRETE_SIMDURATION DISCRETE_RANGE*2/MAX_SPEED
const float MOTOR_CALLBACK =.1; //MOTOR CALL BACK EVERY .1 s
//const float FRICTION_DAMPENING =15/18.8;
const float DISTANCE_ERROR_TOLERANCE=.00;


//const int maxNodesOnSpot = 2*MAX_TURN/(M_PI_2-atan(ROBOT_HALFWIDTH/ROBOT_HALFLENGTH));
										//this is the angle that the robot would have to turn
										//to be at a safe distance from an obstacle in the worst
										//case scenario, provided that the position of the obstacle
										//refers to the centre of mass of the robot
const int maxNodesOnSpot =4;

class Robot {
private: 
	b2FixtureDef fixtureDef;
	//b2PolygonShape box;
	//b2Vec2 centroid;
public:
	b2Vec2 velocity = {0,0};
	b2Body* body;
	b2BodyDef bodyDef;

	Robot(b2World * world) {
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(0.0f, 0.0f);
		body = world->CreateBody(&bodyDef);
		//body->SetUserData(this);  
		//uintptr_t ptr = static_cast<uintptr_t>(this);
		body->GetUserData().pointer = reinterpret_cast<uintptr_t>(this);
		b2Vec2 center(ROBOT_BOX_OFFSET_X, ROBOT_BOX_OFFSET_Y);
		b2PolygonShape box;
		box.SetAsBox(ROBOT_HALFLENGTH, ROBOT_HALFWIDTH, center, ROBOT_BOX_OFFSET_ANGLE);
		//centroid = box.m_centroid; 
		fixtureDef.shape = &box;
		fixtureDef.friction =0;
		body->CreateFixture(&fixtureDef);
		
	}
};


#endif

