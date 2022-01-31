#ifndef OBSTACLE_H
#define OBSTACLE_H
#include <iostream>
#include "box2d/box2d.h"

//can also make one obstacle body with different fixtures but can do later

class Obstacle {
public:
	b2BodyDef bodyDef;
	b2Body* body;
	Obstacle(b2World& world, float pos_x, float pos_y) {	//commenting this because to create objects iteratively you shouldnt have a default contructor
		bodyDef.type = b2_staticBody;
		//obDef.bullet = true; //optimise collisions, not necessary
		bodyDef.position.Set(pos_x, pos_y); //set origin to ground body (is this in world coor?)
		bodyDef.enabled = true;
		body = world.CreateBody(&bodyDef);
		b2PolygonShape fixture; //giving the ground the shape of a box. useful to know if collision can be detected without fixture
		fixture.SetAsBox(.01f, .01f); 
		b2FixtureDef obFixDef; //this line is not essential
		body->CreateFixture(&fixture, 10.0f);
	}

	//only make the body, make the fixtures iteratively using makeFixtures

	Obstacle(b2World& world) { 
		bodyDef.type = b2_staticBody;
		bodyDef.bullet = true; //optimise collisions, not necessary
		bodyDef.position.Set(0.0f, 0.0f); //set origin to ground body (is this in world coor?)
		body = world.CreateBody(&bodyDef);

	}

	void makeFixtures(b2Body * body, float x, float y) {
		b2CircleShape* fixture = new b2CircleShape();
		fixture->m_p.Set(x, y);//haven't tested this
		fixture->m_radius = 0.01f;
	};


};

#endif // !OBSTACLE_H

