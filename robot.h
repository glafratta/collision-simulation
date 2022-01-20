#ifndef _ROBOT
#define _ROBOT

#include "box2d/box2d.h"

class Robot {
public:
	b2Body* body;
	float m_maxLateralImpulse = 100;
	b2Vec2 velocity;
	float acc = 0; // m / sec^2
	float steer = 0; // angle / sec

	const float distanceBeteenWheels = 0.13; // m

	float getDifferentialVelocity() {
		return steer * distanceBeteenWheels;
	}

	float getVelocity() {
		return velocity.Length();
	}
	
	Robot(b2World& world) {
		b2BodyDef robotBodyDef;
		robotBodyDef.type = b2_dynamicBody;
		robotBodyDef.position.Set(0.0f, 0.0f);
		body = world.CreateBody(&robotBodyDef);
		b2CircleShape circleShape;
		circleShape.m_radius = 0.3;   
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &circleShape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 1.0f;
		body->CreateFixture(&fixtureDef);
	}

	void setVelocity(b2Vec2 _velocity) {
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

};


#endif

