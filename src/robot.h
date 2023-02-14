#ifndef _ROBOT
#define _ROBOT

#include "Box2D/Box2D.h"
#include <math.h>



class Robot {
private: 
float halfLength = .085f;
float halfWidth = .04f;
public:
	b2Vec2 velocity = {0,0};
	b2Body* body;
	b2BodyDef bodyDef;

	Robot(b2World * world) {
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(0.0f, 0.0f);
		body = world->CreateBody(&bodyDef);
		body->SetUserData(this);  
		b2PolygonShape fixture; //giving the ground the shape of a box. useful to know if collision can be detected without fixture
		fixture.SetAsBox(halfLength, halfWidth); 
		body->CreateFixture(&fixture, 1.0f);
		
	}

	// void setVelocity(b2Vec2 _velocity) { //apply a vector to robot
	// 	velocity = _velocity;
	// 	float robotAngle = -atan(velocity.y/velocity.x);
	// 	body->SetLinearVelocity( velocity );
	// 	body->SetTransform( body->GetPosition(), robotAngle);
	// }

	// b2Vec3 getPose(){
	// 	b2Vec3 pose;
	// 	pose.x = body->GetPosition().x;
	// 	pose.y = body->GetPosition().y;
	// 	pose.z = body->GetAngle();
	// 	return pose;
	// }

	// void dumpCorners(int it){ //dumps all corners in one file, for debugging and for me to just see what's happening to robot orientation
	// 	char name[256];
	// 	sprintf(name, "corners%04i.txt", iteration);
	// 	FILE * file = fopen(name, "a+");
	// 	fprintf(file, "%f\t%f",);
	// }
	
};


#endif

