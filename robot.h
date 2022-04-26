#ifndef _ROBOT
#define _ROBOT

#include "Box2D/Box2D.h"
#include <iostream>

class Robot {
private:
	float timeCrashed;
public:
	b2Vec2 velocity = {0,0};
	b2Body* body;
	b2BodyDef bodyDef;
	// float m_maxLateralImpulse = 100;
	// float acc = 0; // m / sec^2
	// float steer = 0; // angle / sec
	// float mass = .7; //kg
	bool crashed;
	float angle; //rad
	const float distanceBeteenWheels = 0.08; // m


	b2Vec2 getVelocity() { 
		return velocity;
	}

	Robot(b2World* world) {
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(0.0f, 0.0f);
		body = world->CreateBody(&bodyDef);
		body->SetUserData(this);  
		b2PolygonShape fixture; //giving the ground the shape of a box. useful to know if collision can be detected without fixture
		fixture.SetAsBox(.04f, .085f); 
		body->CreateFixture(&fixture, 1.0f);
		crashed = false;
		
	}

	void setVelocity(b2Vec2 _velocity) { //vector represent how much the robot moves in one second
		velocity = _velocity;
		float robotAngle = -atan2(velocity.y, velocity.x);
		body->SetLinearVelocity( velocity );
		body->SetTransform( body->GetPosition(), robotAngle);
	}

	void startContact() {
		std::cout << "bonk\n";
		crashed = true;
	}

	void endContact() {
		//std::cout << "bye-ee!\n";
		crashed = false;
	}


	// void pathPlan(){
	// 	std::cout << "oops!\n";
	// }
};


#endif

