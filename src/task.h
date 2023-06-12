#ifndef TASK_H
#define TASK_H
#include "Box2D/Box2D.h"
#include <vector>
#include <stdio.h>
#include <math.h> 
#include "measurement.h"
#include <stdexcept>
#include "settings.h"
#include "general.h" //general functions + point class + typedefs + Primitive.h + boost includes
#define BOX2DRANGE 0.5
#define LIDAR_RANGE 1.0
#define REACTION_TIME 2.3
#define HZ 50.0
const float SIM_DURATION = int(BOX2DRANGE*2 /MAX_SPEED);


class Task{
public:
    float accumulatedError=0;
    char planFile[250]; //for debug
    b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    bool change =0;
    float pGain=0.063;
    float endAvoid = M_PI_2;
    EndCriteria endCriteria; //end criteria other than task encounters a disturbance
    Direction direction= Direction::DEFAULT;
protected:
    //Type type = Type::BASELINE;
    b2Vec2 RecordedVelocity ={0.0f, 0.0f};
public:


struct Disturbance{ //this generates error
private:
    bool valid= 0;
    AffordanceIndex affordanceIndex = 0; //not using the enum because in the future we might want to add more affordances
    float angleToRobot=0;
public:
	b2FixtureDef fixtureDef;
    b2Vec2 position;
    bool safeForNow=1;
    Disturbance(){};
    Disturbance(AffordanceIndex i){
        if (i>affordances.size()-1){
            throw std::invalid_argument("Not a valid affordance index\n");
        }
        else{
            affordanceIndex = i;
        }
    }
    Disturbance(AffordanceIndex i, b2Vec2 p){
        if (i>affordances.size()-1){
            throw std::invalid_argument("Not a valid affordance index\n");
        }
        else{
            affordanceIndex = i;
        }
        //bodyDef.type = b2_dynamicBody;
		position.Set(p.x, p.y);
        valid =1;
    }

    void setAngle(float a){
        angleToRobot =a;
    }

    float setAngle(b2Transform t){ //gets the angle of an Disturbance wrt to another Disturbance (robot)
        //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
        float angle;
        b2Vec2 thisToB;
        thisToB.x = position.x-t.p.x;
        thisToB.y = position.y - t.p.y;
        float cosA = (thisToB.x * cos(t.q.GetAngle())+ thisToB.y*sin(t.q.GetAngle()))/thisToB.Length();
        angleToRobot = acos(cosA);
    }


    float getAngle(b2Transform t){ //gets the angle of an Disturbance wrt to another Disturbance (robot)
        //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
        float angle;
        b2Vec2 thisToB;
        thisToB.x = position.x-t.p.x;
        thisToB.y = position.y - t.p.y;
        float cosA = (thisToB.x * cos(t.q.GetAngle())+ thisToB.y*sin(t.q.GetAngle()))/thisToB.Length();
        angle = acos(cosA);
        return angle;
    }

    // float getAngle(float angle2){ //gets the angle of an Disturbance wrt to the heading direction of another Disturbance
    //     //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
    //     float angle1=0;
    //     if (position.y !=0 && position.x !=0){ //inclusive or?
    //         angle1 = atan(position.y/position.x); //own angle to the origin 
    //     }
	//     float angle = angle1-angle2;
    //     return angle;
    // }

    float getAngle(b2Body* b){
        // float angle;
        // b2Vec2 thisToB;
        // thisToB.x = position.x-b->GetPosition().x;
        // thisToB.y = position.y - b->GetPosition().y;./no
        // float cosA = (thisToB.x * cos(b->GetAngle())+ thisToB.y*sin(b->GetAngle()))/thisToB.Length();
        // angle = acos(cosA);
        // return angle;
        return getAngle(b->GetTransform());
    }

    float getAngle(){
        return angleToRobot;
    }

    void setPosition(b2Vec2 pos){
        position.Set(pos.x, pos.y);
        valid =1;
    }
    
    void setPosition(float x, float y){
        position.Set(x, y);
        valid=1;
    }
    
    b2Vec2 getPosition(){
        return position;
    }


    bool isValid(){
        return valid;
    }

    AffordanceIndex getAffIndex(){
        return affordanceIndex;
    }

    void invalidate(){
        valid =0;
    }
}; //sub action f


struct Action{
private:
    float linearSpeed=.0625; //used to calculate instantaneous velocity using omega
    float omega=0; //initial angular velocity is 0
    bool valid=0;
public:
    float R=.5;
    float L=.5;

    Action(Direction direction = DEFAULT){
        switch (direction){
        case Direction::DEFAULT:
        L=0.5;
        R=.5;
        break;
        case Direction::LEFT:
        L = -0.5;
        R=0.5;
        break;
        case Direction::RIGHT:
        L=0.5;
        R = - 0.5;
        break;
        case Direction::BACK:
        L = -0.5;
        R = -0.5;
        break;
        case Direction::STOP:
        L=0;
        R=0;
        break;
        default:
        throw std::invalid_argument("not a valid direction for M");
        break;
    }
    //kinematic model internal to action so it can be versatile for use in real P and simulated P
    omega = (MAX_SPEED*(R-L)/BETWEEN_WHEELS); //instant velocity, determines angle increment in willcollide

    linearSpeed = MAX_SPEED*(L+R)/2;

    valid=1;
        
    }

    void init(Direction direction){
        switch (direction){
        case Direction::DEFAULT:
        break;
        case Direction::LEFT:
        L = -0.5;
        R=0.5;
        break;
        case Direction::RIGHT:
        L=0.5;
        R = - 0.5;
        break;
        case Direction::BACK:
        L = -0.5;
        R = -0.5;
        break;
        case Direction::STOP:
        L=0;
        R=0;
        default:
        throw std::invalid_argument("not a valid direction for M");
        break;
    }
    //kinematic model internal to action so it can be versatile for use in real P and simulated P

    omega = (MAX_SPEED*(R-L)/BETWEEN_WHEELS); //instant velocity, determines angle increment in willcollide

    linearSpeed = MAX_SPEED*(L+R)/2;

    valid=1;
    }

    b2Vec2 getLinearVelocity(){
        b2Vec2 velocity;
        velocity.x = linearSpeed *cos(omega);
        velocity.y = linearSpeed *sin(omega);
        return velocity;
    }

    float getRWheelSpeed(){
        return R;
    }

    float getLWheelSpeed(){
    return L;
    }


    bool isValid(){
        return valid;
    }

    float getLinearSpeed(){
        return linearSpeed;
    }

    float getOmega(){
    return omega;
    }

};


struct simResult{
    enum resultType {successful =0, crashed =1, safeForNow=2}; //successful means no collisions, finished means target reached, for later
    resultType resultCode;
    Disturbance collision;
    bool valid = 0;
    float distanceCovered =0;
    b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));


    simResult(){}

    simResult(resultType code): resultCode(code){
        valid =1;
    }

    simResult(resultType code, Disturbance obst): resultCode(code), collision(obst){
        valid =1;
    }
};


class Listener : public b2ContactListener {
  int iteration=1;
    public:
    Listener(){}
    std::vector <b2Vec2> collisions;
    
		void BeginContact(b2Contact * contact) {
			void* bodyData = contact->GetFixtureA()->GetBody()->GetUserData();
			if (bodyData) {
                b2Body * other = contact->GetFixtureB()->GetBody();
                collisions.push_back(other->GetPosition());
			}
			bodyData = contact->GetFixtureB()->GetBody()->GetUserData();
			if (bodyData) {
                b2Body * other = contact->GetFixtureA()->GetBody();
                collisions.push_back(other->GetPosition());
                }       
		}
        
	};
private:
Action action = Action();
public:
//std::vector <Direction> options;
Disturbance disturbance;

Task::Action getAction(){
    return action;
}

AffordanceIndex getAffIndex(){
    return disturbance.getAffIndex();
}

Direction H(Disturbance, Direction);

void setEndCriteria();

bool checkEnded(b2Transform robotTransform = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)));

Task(){
    action = Action(direction);
    RecordedVelocity = action.getLinearVelocity();
}

Task(Disturbance ob, Direction d){
    disturbance = ob;
    direction = H(disturbance, d);  
    action = Action(direction);
    RecordedVelocity = action.getLinearVelocity();
    setEndCriteria();
}

void setRecordedVelocity(b2Vec2 vel){
    RecordedVelocity = vel;
    
} //useful to get the speed.


b2Vec2 getRecordedVelocity(){
    return RecordedVelocity;
}


void trackDisturbance(Disturbance &, float, b2Vec2, b2Transform);

simResult willCollide(b2World &, int, bool, b2Vec2, float, float);

enum controlResult{DONE =0, CONTINUE =1};

controlResult controller();

void setGain(float f){
    pGain=f;
}


};

#endif