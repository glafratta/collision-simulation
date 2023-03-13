#ifndef _ROBOT
#define _ROBOT

#include "Box2D/Box2D.h"
#include <math.h>
#define SAFE_ANGLE M_PI_2
#define MAX_TURN M_PI
#define ROBOT_HALFLENGTH 0.085
#define ROBOT_HALFWIDTH 0.04
const int maxNodesOnSpot = 2*MAX_TURN/(M_PI_2-atan(ROBOT_HALFWIDTH/ROBOT_HALFLENGTH));
										//this is the angle that the robot would have to turn
										//to be at a safe distance from an obstacle in the worst
										//case scenario, provided that the position of the obstacle
										//refers to the centre of mass of the robot


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
	// 	//midpoint is position
	// 	b2Vec2 midFront, midBack, front1, front2, back1, back2;
	// 	midFront.x = (body->GetPosition().Length()+halfLength) *cos(body->GetTransform().q.GetAngle());
	// 	midFront.y = (body->GetPosition().Length()+halfLength) *sin(body->GetTransform().q.GetAngle());
	// 	midBack.x = (body->GetPosition().Length()-halfLength) *cos(body->GetTransform().q.GetAngle());
	// 	midBack.y = (body->GetPosition().Length()-halfLength) *sin(body->GetTransform().q.GetAngle());
	// 	front1.x = midFront.x - halfwidth;
	// 	//fprintf(file, "%f\t%f\n", body.);
	// }
	
};


#endif

