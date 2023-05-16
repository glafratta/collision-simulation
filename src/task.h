#pragma once
#include "Box2D/Box2D.h"
#include <vector>
#include <stdio.h>
#include <math.h> 
#define BOX2DRANGE 0.5
const float REACTION_TIME =2.0;



enum DisturbanceType {obstacle=0, target=1, other=2};  


class Task{
public:
    float hz =50.0f; 
    float accumulatedError=0;
    char planFile[250]; //for debug
    float lidarRange =1.5;
    enum Type {BASELINE =0, AVOID =1, PURSUE =2, PANIC =3};
    b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    bool change =0;
    float pGain=0.063;
    float endAvoid = M_PI_2;
protected:
    Type type;
    float maxSpeed = 0.125f; //this needs to be defined better
    b2Vec2 RecordedVelocity ={0.0f, 0.0f};
    int simDuration =int(BOX2DRANGE*2 /maxSpeed); //in seconds



public:


struct Disturbance{ //maybe later can susbtitute this for a broader Disturbance so you can also set a target without having to make another class for it. Bernd has an enum Disturbance identifier
private:
    bool valid= 0;
    DisturbanceType type;
    int iteration;
    float angleToRobot=0;
public:
	b2FixtureDef fixtureDef;
    b2BodyDef bodyDef;
    b2Body * body;
    bool safeForNow=1;
    Disturbance(){};
    Disturbance(DisturbanceType _t): type(_t){}
    Disturbance(DisturbanceType _t, b2Vec2 position):type(_t){
        bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(position.x, position.y);
        valid =1;
    }

    void setAngle(float a){
        angleToRobot =a;
    }


    float getAngle(b2Vec2 posVector){ //gets the angle of an Disturbance wrt to another Disturbance (robot)
        //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
        float angle1=0;
        float angle2 =0;
        if (bodyDef.position.y !=0 && bodyDef.position.x !=0){ //inclusive or?
            angle1 = atan(bodyDef.position.y/bodyDef.position.x); //own angle to the origin 
        }
        if (posVector.y != 0 && posVector.y !=0){
            angle2 = atan(posVector.y/posVector.x);
        }
	    float angle = angle1-angle2;
        return angle;
    }

    float getAngle(float angle2){ //gets the angle of an Disturbance wrt to the heading direction of another Disturbance
        //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
        float angle1=0;
        if (bodyDef.position.y !=0 && bodyDef.position.x !=0){ //inclusive or?
            angle1 = atan(bodyDef.position.y/bodyDef.position.x); //own angle to the origin 
        }
	    float angle = angle1-angle2;
        return angle;
    }

    float getAngle(b2Body* b){
        float angle;
        b2Vec2 thisToB;
        thisToB.x = bodyDef.position.x-b->GetPosition().x;
        thisToB.y = bodyDef.position.y - b->GetPosition().y;
        float cosA = (thisToB.x * cos(b->GetAngle())+ thisToB.y*sin(b->GetAngle()))/thisToB.Length();
        angle = acos(cosA);
        return angle;
    }

    void setPosition(b2Vec2 pos){
        bodyDef.position.Set(pos.x, pos.y);
        valid =1;
    }
    
    void setPosition(float x, float y){
        bodyDef.position.Set(x, y);
        valid=1;
    }
    
    b2Vec2 getPosition(){
        return bodyDef.position;
    }


    void setIteration(int _it){
        iteration=_it;
    }


    bool isValid(){
        return valid;
    }

    DisturbanceType getType(){
        return type;
    }

    void invalidate(){
        valid =0;
    }
}; //sub action f


enum Direction{LEFT, RIGHT, NONE, BACK, STOP};

struct Action{
private:
    float linearSpeed=.0625; //used to calculate instantaneous velocity using omega
    b2Vec2 velocity;
    float omega=0; //initial angular velocity is 0  
    bool valid=0;
    float distanceBetweenWheels = 0.15f;
    float maxOmega = M_PI; //calculated empirically with maxspeed of .125
    float minAngle = M_PI_2; //turn until the angle between the distance vector and the velocity 
    Direction direction;
public:
    float RightWheelSpeed=0.5;
    float LeftWheelSpeed=0.5;


    Action(){}

    void __init__(){
        direction = Direction::NONE;
    }

    void __init__(Disturbance &ob, Direction d, float simDuration=3, float maxSpeed=0.125, float hz=60.0f, b2Vec2 pos = {0,0}, float end = M_PI_2){
    direction = d;
    float maxDistance = maxSpeed*simDuration;
    if (ob.isValid()==true){
        if (ob.getType()==DisturbanceType::obstacle){
            if (abs(ob.getAngle(pos))<end){
                //NEW LOOP FOR ABOVE
                if (direction == Task::Direction::NONE){ //if there are no constraints on the direction other than where the obstacle is, pick at random
                    if (ob.getPosition().y<0){ //obstacle is to the right, vehicle goes left; ipsilateral excitatory, contralateral inhibitory
                        direction = Task::Direction::LEFT; //go left
                    }
                    else if (ob.getPosition().y>0){ //go right
                        direction = Task::Direction::RIGHT; //go left
                    }   
                    else{
                        int c = rand() % 2;
                        direction = static_cast<Task::Direction>(c);

                    }
                }

            }
            else{
            }     
        }
    }

    switch (direction){
        case Task::Direction::LEFT:
        LeftWheelSpeed = -LeftWheelSpeed;
        break;
        case Task::Direction::RIGHT:
        RightWheelSpeed = - RightWheelSpeed;
        break;
        case Task::Direction::BACK:
        LeftWheelSpeed = - LeftWheelSpeed;
        RightWheelSpeed = -RightWheelSpeed;
        break;
        case Task::Direction::STOP:
        LeftWheelSpeed=0;
        RightWheelSpeed=0;
        default:
        break;
    }


    omega = (maxSpeed*(RightWheelSpeed-LeftWheelSpeed)/distanceBetweenWheels); //instant velocity, determines angle increment in willcollide
        if (abs(omega)>M_PI){ //max turning angle in one second
            float multiplier=1;
            if (omega<0){
                multiplier=-1;
            }
            omega=M_PI*multiplier;
        }

    linearSpeed = maxSpeed*(LeftWheelSpeed+RightWheelSpeed)/2;
    if (abs(linearSpeed)>maxSpeed){
        float multiplier=1;
    if (linearSpeed<0){
        multiplier=-1;
    }
    linearSpeed=maxSpeed*multiplier;
    }
    valid=1;
    }
    b2Vec2 getLinearVelocity(float maxV = 0.125){
        b2Vec2 vel;
        vel.x = linearSpeed *cos(omega);
        vel.y = linearSpeed *sin(omega);
        return vel;
    }

    float getRWheelSpeed(){
        return RightWheelSpeed;
    }

    float getLWheelSpeed(){
    return LeftWheelSpeed;
    }


    float getDistanceWheels(){
        return distanceBetweenWheels;
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

    Task::Direction getDirection(){
        return direction;
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
Action action;
public:
std::vector <Task::Direction> options;
Disturbance obstacle;

Task::Action getAction(){
    return action;
}

Task::Type getType(){
    return type;
}



Task(){
    action.__init__(); //this is a valid trajectory, default going straight at moderate speed
    type = Type::BASELINE;
    RecordedVelocity = action.getLinearVelocity();

}

Task(Disturbance ob, Direction direction = Direction::NONE){
    action.__init__(ob, direction, simDuration, maxSpeed, hz, {0.0f, 0.0f}); 
    RecordedVelocity = action.getLinearVelocity();
    if (ob.getType()== DisturbanceType::obstacle && ob.isValid()==1){ //og obstacle.getTYpe()
        obstacle = ob;
        type =Type::AVOID;
    }
    else{
        type =Type::BASELINE;
    }

}

void __init__(){
    action.__init__(); //this is a valid trajectory, default going straight at moderate speed
    type = Type::BASELINE;
    RecordedVelocity = action.getLinearVelocity();

}

void __init__(Disturbance ob, Direction direction = Direction::NONE){
    action.__init__(ob, direction, simDuration, maxSpeed, hz, {0.0f, 0.0f}); 
    RecordedVelocity = action.getLinearVelocity();
    if (ob.getType()== DisturbanceType::obstacle && ob.isValid()==1){ //og obstacle.getTYpe()
        obstacle = ob;
        type =Type::AVOID;
    }
    else{
        type =Type::BASELINE;
    }

}


void setObstacle(Disturbance ob){
    obstacle = ob;
}

float getMaxSpeed(){
    return maxSpeed;
}

void setHz(float _hz){
    hz = _hz;
}

void setSimDuration(int d){ //in seconds
    simDuration = d;
}

int getSimDuration(){ //in seconds
    return simDuration;
}

void setRecordedVelocity(b2Vec2 vel){
    RecordedVelocity = vel;
    
} //useful to get the speed.


b2Vec2 getRecordedVelocity(){
    return RecordedVelocity;
}

b2Vec2 getLinearVelocity(float R, float L, float maxV = 0.125){
    b2Vec2 vel;
    float realL = maxV*L;
    float realR = maxV*R;
        //find angle theta in the pose:
    float W = (realL-realR)/action.getDistanceWheels(); //rad/s, final angle at end of 1s
    //find absolute speed
    float V =(realL+realR)/2; //velocity
    if (realR-realL == 0){
        vel.x = realL;
        vel.y = 0;
        }
    else {
        vel.x = (action.getDistanceWheels()/2)* sin(action.getDistanceWheels()/(realR-realL));
        vel.y = -(action.getDistanceWheels()/2)* cos(action.getDistanceWheels()/(realR-realL));

    }
    return vel;
}

float getAngularVelocity(float R, float L, float maxV = 0.125){
    float W = (maxV*(R-L)/action.getDistanceWheels()); //instant velocity, determines angle increment in willcollide
    if (abs(W)>M_PI){
        float multiplier=1;
        if (W<0){
            multiplier=-1;
        }
        W= M_PI*multiplier;
    }
    return W;
}

float getLinearSpeed(float R, float L, float maxV = 0.125){
    float v = maxV*(L+R)/2;
    if (abs(v)>maxV){
        float multiplier=1;
    if (v<0){
        multiplier=-1;
    }
    v=maxV*multiplier;
    }
    return v;
}

void trackDisturbance(Disturbance &, float, b2Vec2, b2Vec2);

simResult willCollide(b2World &, int, bool, b2Vec2, float, float);

enum controlResult{DONE =0, CONTINUE =1};

controlResult controller();

void setGain(float f){
    pGain=f;
}


private:



};