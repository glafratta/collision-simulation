#ifndef _ROBOT
#define _ROBOT

#include "Box2D/Box2D.h"

class Robot {
private:
	b2Vec2 velocity = {0,0};
	float timeCrashed;
public:
	b2Body* body;
	b2BodyDef bodyDef;
	float m_maxLateralImpulse = 100;
	float acc = 0; // m / sec^2
	float steer = 0; // angle / sec
	float mass = .7; //kg
	bool crashed;
	const float distanceBeteenWheels = 0.13; // m

	float getDifferentialVelocity() {
		return steer * distanceBeteenWheels;
	}

	b2Vec2 getVelocity() { 
		return velocity;
	}
	//Robot();

	Robot(b2World& world) {
		bodyDef.type = b2_dynamicBody;
		//robotBodyDef.bullet = true; //to optimise collision detection between fixtures
		bodyDef.position.Set(0.0f, 0.0f);
		body = world.CreateBody(&bodyDef);
		body->SetUserData(this); 
		b2CircleShape circleShape;
		circleShape.m_radius = 0.1;   
		b2FixtureDef fixtureDef;
		//fixtureDef.isSensor = true;
		fixtureDef.shape = &circleShape;
		fixtureDef.density = 2.0f;
		fixtureDef.friction = 1.0f;
		body->CreateFixture(&fixtureDef);
		crashed = false;
		
	}
	//~Robot();

	void setVelocity(b2Vec2 _velocity) { //vector represent how much the robot moves in one second
		velocity = _velocity;
		float robotAngle = -atan2(velocity.y, velocity.x);
		body->SetLinearVelocity( velocity );
		body->SetTransform( body->GetPosition(), robotAngle);
	}

	void applyAccelerationSteering(float _acc, float _steer) {
		acc = _acc;
		steer = _steer;
	}

	void step(float dt) {
		float v = velocity.Length() + acc * dt;
		float p = body->GetAngle() + steer * dt;
		velocity.x = v * cos(p);
		velocity.y = -v * sin(p);
		body->SetLinearVelocity( velocity );
		body->SetTransform( body->GetPosition(), p );
	}

	void startContact() {
		//std::cout << "bonk\n";
		crashed = true;
	}

	void endContact() {
		//std::cout << "bye-ee!\n";
		crashed = false;
	}

	b2Vec2 getAcceleration(b2Vec2 force, float mass) {
		b2Vec2 acc;
		double angle = atan2(force.y , force.x);
		float accScalar = force.Length() / mass;
		acc.x = accScalar * cos(angle);
		acc.y = accScalar * sin(angle);
		return acc;
	}

	void pathPlan(){
		std::cout << "oops!\n";
	}
};


#endif

