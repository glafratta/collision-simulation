#ifndef _ROBOT
#define _ROBOT

#include "Box2D/Box2D.h"
#include <math.h>



class Robot {
public:
	b2Vec2 velocity = {0,0};
	b2Body* body;
	b2BodyDef bodyDef;
	// float m_maxLateralImpulse = 100;
	// float mass = .7; //kg
	//onst float distanceBeteenWheels = 0.08; // m
	//b2Vec2 defaultSpeed = {0.125f, 0.0f}; // meters/s
	//float maxAbsSpeed=.2;
	//Action action;
	char msg[8];


	// b2Vec2 getVelocity() { 
	// 	return velocity;
	// }

	Robot(b2World * world) {
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(0.0f, 0.0f);
		body = world->CreateBody(&bodyDef);
		body->SetUserData(this);  
		b2PolygonShape fixture; //giving the ground the shape of a box. useful to know if collision can be detected without fixture
		fixture.SetAsBox(.04f, .085f); 
		body->CreateFixture(&fixture, 1.0f);
		//sprintf(msg, "hello");
		//box2d = _box2d;
		
	}

	void setVelocity(b2Vec2 _velocity) { //apply a vector to robot
		velocity = _velocity;
		float robotAngle = -atan(velocity.x/velocity.y);
		//float robotAngle = (velocity.x+velocity.y)/(velocity.Length()*sqrt(2)); //angle formed by velocity with xy axes. 1's were omitted

		body->SetLinearVelocity( velocity );
		body->SetTransform( body->GetPosition(), robotAngle);
	}




	// void pathPlan(){
	// 	std::cout << "oops!\n";
	// }
};


#endif

