#ifndef _ROBOT
#define _ROBOT

#include "Box2D/Box2D.h"
#include <iostream>


enum ActionType {GO=0, LEFT=1, RIGHT=2, STOP=3};


struct Action {
	int t;
    float initialAngle;
    float speed;
    float acceleration=0; //how much to increment per time the new action for that timestep is updated
    float steeringIncrement=0;
	float x = speed* sin(initialAngle+steeringIncrement*t);
	float y = speed* cos(initialAngle+steeringIncrement*t);
    const float maxSteering_R = -M_PI/18; //10 DEG
    const float maxSteering_L = 0.226892803; //13 DEG
    //enum type{GO=0, TURN=1; STOP=2}; //turn is just dumb turning
	float toi;
	bool isValid =0;
    ActionType type;
    Action(){};
	Action(b2Vec2 velocity){
		initialAngle=-atan2(velocity.y,velocity.x);
		speed = velocity.Length();
	}
    Action(float _speed, float _angle, float _steering =0,int t=0): initialAngle(_angle), speed(_speed), steeringIncrement(_steering){
		isValid=1;
	}
	
	void setStep(int step){
		t=step;
	}
};


class Robot {
public:
	b2Vec2 velocity = {0,0};
	b2Body* body;
	b2BodyDef bodyDef;
	// float m_maxLateralImpulse = 100;
	// float mass = .7; //kg
	const float distanceBeteenWheels = 0.08; // m
	b2Vec2 defaultSpeed = {0.125f, 0.0f}; // meters/s
	float maxAbsSpeed=.2;
	//Action action;
	char msg[8];


	b2Vec2 getVelocity() { 
		return velocity;
	}

	Robot(b2World * world) {
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(0.0f, 0.0f);
		body = world->CreateBody(&bodyDef);
		body->SetUserData(this);  
		b2PolygonShape fixture; //giving the ground the shape of a box. useful to know if collision can be detected without fixture
		fixture.SetAsBox(.04f, .085f); 
		body->CreateFixture(&fixture, 1.0f);
		sprintf(msg, "hello");
		//box2d = _box2d;
		
	}

	void setVelocity(b2Vec2 _velocity) { //apply a vector to robot
		velocity = _velocity;
		float robotAngle = -atan2(velocity.y, velocity.x);
		body->SetLinearVelocity( velocity );
		body->SetTransform( body->GetPosition(), robotAngle);
	}

	void setAction(Action _action){
		action= _action;
	}



	// void pathPlan(){
	// 	std::cout << "oops!\n";
	// }
};


#endif

