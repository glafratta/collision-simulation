#ifndef OBSTACLE_H
#define OBSTACLE_H
#include <iostream>
#include "box2d/box2d.h"


class Obstacle {
public:
	float x, y;
	b2BodyDef obDef;
	Obstacle(b2World& world, float pos_x, float pos_y) {	//commenting this because to create objects iteratively you shouldnt have a default contructor
		obDef.type = b2_staticBody;
		obDef.position.Set(pos_x, pos_y); //set origin to ground body
		b2Body* ob = world.CreateBody(&obDef);
		b2PolygonShape fixture; //giving the ground the shape of a box
		fixture.SetAsBox(.15, .15); 
		//b2FixtureDef obFixDef; //this line is not essential
		ob->CreateFixture(&fixture, 0.0f);
	}

};

#endif // !OBSTACLE_H

