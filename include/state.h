#pragma once
#include <vector>
#include <stdio.h>
#include <math.h>
#include "Box2D/Box2D.h"

class State{
protected:
    enum StateType{GO=0, AVOID=1, PURSUE =2, PANIC=3}
    StateType type;
    float maxSpeed = .125; //this needs to be defined better
    std::vector <Object> collisions; //collision eliminated if angle between it and the robot is larger than 90deg. Target state: no Objects predicted
    float hz =10.0f;
	float simDuration =3;
    float RightWheelSpeed;
    float LeftWheelSpeed;
    const float distanceBetweenWheels = 0.08;
    b2Vec2 RecordedVelocity=EstimatedLinearVelocity; //velocity recorded at t. if no data is available it falls back on the prediction
    float omega;
    b2Vec2 EstimatedLinearVelocity = {sin(M_PI_2-omega), cos(M_PI_2-omega)}; //is is sin and y is cos, this is pretty much instantaneous to get the system started

public:

struct Object{ //maybe later can susbtitute this for a broader object so you can also set a target without having to make another class for it. Bernd has an enum object identifier
private:
    b2Vec2 where;
    int iteration;
    float step;
    Action action;
    bool isValid= 0;
    ObjectType type;
public:
    enum ObjectType{OBSTACLE = 0, TARGET=1, OTHER =2}
    bool crashed;
    //Object(){}
    Object(ObjectType _t): type(_t){}
    Object(b2Vec2 _where, int _step=0): where(_where), step(_step){
        isValid=1;
    }
	Object(b2Vec2 _where, int _step=0): where(_where),step(_step){
        isValid=1;
    }

    void setIteration(int _it): iteration(_it){};
    void setStep(int _step):step(_step);
}; //sub action for velocity?

State(StateType _t=0): type(_t){
    switch (type){
    case 0: RightWheelSpeed=0.5; LeftWheelSpeed=0.5; omega = 0;break;
    case 1: //pick random L o R
        int isPos = rand() %2;
        switch (isPos){
            case 0: RightWheelSpeed=0.5; LeftWheelSpeed=1;omega =-M_PI_4;break; //negative steering means go right
            case 1: RightWheelSpeed=1; LeftWheelSpeed=0.5; omega = M_PI_4; break;//positive steering means go left
        }
        break;
    }
    // case 2: selectTarget(b2Vec2);break;
    case 3: RightWheelSpeed=0; LeftWheelSpeed=0; velocity = {0,0}; break; //if panic state stops
    default: printf("invalid state type");break;
}

class Listener : public b2ContactListener {
	//register state with listener and add collision into state OR create actual state
	public:
	Object collision;
		void BeginContact(b2Contact * contact) {
			void* bodyData = contact->GetFixtureA()->GetBody()->GetUserData();
			if (bodyData) {
                Robot *robot = static_cast<Robot*>(bodyData);
                collision= Object(robot->body->GetPosition(), robot->action);
                // if ((collisions.size()==0)||((collisions.back().where.x!= collision.where.x)&&(collisions.back().where.y!= collision.where.y))){
                //     collisions.push_back(collision);
                // }
                b2Body * other = contact->GetFixtureB()->GetBody();
                //printf("bonk with a body at{%f, %f}\n", other->GetPosition().x, other->GetPosition().y);
			}


			bodyData = contact->GetFixtureB()->GetBody()->GetUserData();
			if (bodyData) {
                Robot *robot = static_cast<Robot*>(bodyData);
                collsion= Object(robot->body->GetPosition(), robot->action);
                //Collision collision(robot->body->GetPosition(), robot->action);
                // if ((collisions.size()==0)||((collisions.back().where.x!= collision.where.x)&&(collisions.back().where.y!= collision.where.y))){
                //     collisions.push_back(collision);
                // }	
                b2Body * other = contact->GetFixtureA()->GetBody();
                //printf("bonk with a body at{%f, %f}\n", other->GetPosition().x, other->GetPosition().y);
                }
                
        //CHECK IF YOU CAN JUST CHANGE VARIABLES WITHIN STATE
		}

	};


void setObject(b2Vec2 _where, int iteration){
    Object c(_where);
    c.setIteration(iteration);
    Objects.push_back(c);
}

float predictOmega(float max=maxSpeed){ //returns a prediction of what the angular velocity should be with the current settings
    return max*(RightWheelSpeed-LeftWheelSpeed)/distanceBetweenWheels;
}

float predictMaxSpeed(){
    return RecordedVelocity.Length()*(RightWheelSpeed+LeftWheelSpeed)/2; //linear velocity of robot body
}

bool hasCollision(){
    return collisions.size()>0; //if vector containing collisions is not of size 0, the current plan has crashed
}

void findWheelSpeed();


void setVelocity(b2Vec2 vel): RecordedVelocity(vel){} //useful to get the speed.

void trackObjectSite(Object &);

void followPath(AlphaBot& alphabot); //P-controller: takes in error, returns adjusted commands for alphabot. Take in error as angle! not point

void selectTarget();

bool willCollide(b2World &);

void setDesiredSpeed(float);

float averageMaxSpeed(); //read into files and find mean/median/mode of speed to get a better more precise estimation

void noise(); //create a normal distribution for data (position etc) to find a standard deviation and generate a tolerance interval for that parameter

// void switchState(){} //transition function : the listener is the transition function 





};