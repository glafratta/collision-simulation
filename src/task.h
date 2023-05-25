#pragma once
#include "Box2D/Box2D.h"
#include <vector>
#include <stdio.h>
#include <math.h> 
#include "robot.h"
#define BOX2DRANGE 0.5
#define LIDAR_RANGE 1.0
#define REACTION_TIME 2.0
#define HZ 50.0
const float SIM_DURATION = int(BOX2DRANGE*2 /MAX_SPEED);


enum DisturbanceType {obstacle=0, target=1, other=2};  

class Task{
public:
enum Direction{LEFT, RIGHT, NONE, BACK, STOP};
    float accumulatedError=0;
    char planFile[250]; //for debug
    enum Type {BASELINE =0, AVOID =1, PURSUE =2};
    b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    bool change =0;
    float pGain=0.063;
    float endAvoid = M_PI_2;
    Direction direction= Direction::NONE;
protected:
    Type type = Type::BASELINE;
    b2Vec2 RecordedVelocity ={0.0f, 0.0f};
public:


struct Disturbance{ //maybe later can susbtitute this for a broader Disturbance so you can also set a target without having to make another class for it. Bernd has an enum Disturbance identifier
private:
    bool valid= 0;
    DisturbanceType type;
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

    float setAngle(b2Transform t){ //gets the angle of an Disturbance wrt to another Disturbance (robot)
        //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
        float angle;
        b2Vec2 thisToB;
        thisToB.x = bodyDef.position.x-t.p.x;
        thisToB.y = bodyDef.position.y - t.p.y;
        float cosA = (thisToB.x * cos(t.q.GetAngle())+ thisToB.y*sin(t.q.GetAngle()))/thisToB.Length();
        angleToRobot = acos(cosA);
    }


    float getAngle(b2Transform t){ //gets the angle of an Disturbance wrt to another Disturbance (robot)
        //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
        float angle;
        b2Vec2 thisToB;
        thisToB.x = bodyDef.position.x-t.p.x;
        thisToB.y = bodyDef.position.y - t.p.y;
        float cosA = (thisToB.x * cos(t.q.GetAngle())+ thisToB.y*sin(t.q.GetAngle()))/thisToB.Length();
        angle = acos(cosA);
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

    float getAngle(){
        return angleToRobot;
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


struct Action{
private:
    float linearSpeed=.0625; //used to calculate instantaneous velocity using omega
    b2Vec2 velocity;
    float omega=0; //initial angular velocity is 0  
    bool valid=0;
public:
    float RightWheelSpeed=0.5;
    float LeftWheelSpeed=0.5;


    Action(){}

    Action(Direction direction){
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
    //kinematic model internal to action so it can be versatile for use in real P and simulated P

    omega = (MAX_SPEED*(RightWheelSpeed-LeftWheelSpeed)/BETWEEN_WHEELS); //instant velocity, determines angle increment in willcollide
        // if (abs(omega)>MAX_OMEGA){ //max turning angle in one second
        //     float multiplier=1;
        //     if (omega<0){
        //         multiplier=-1;
        //     }
        //     omega=MAX_OMEGA*multiplier;
        // }

    linearSpeed = MAX_SPEED*(LeftWheelSpeed+RightWheelSpeed)/2;
    // if (abs(linearSpeed)>MAX_SPEED){
    //     float multiplier=1;
    // if (linearSpeed<0){
    //     multiplier=-1;
    // }
    // linearSpeed=MAX_SPEED*multiplier;
    // }
    valid=1;
    }

    b2Vec2 getLinearVelocity(){
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
Action action;
public:
//std::vector <Task::Direction> options;
Disturbance disturbance;

Task::Action getAction(){
    return action;
}

Task::Type getType(){
    return type;
}



Task(){
    RecordedVelocity = action.getLinearVelocity();

}


Task(Disturbance ob, Direction d = Direction::NONE){
    RecordedVelocity = action.getLinearVelocity();
    disturbance = ob;
    if (ob.getType()== DisturbanceType::obstacle && ob.isValid()==1){ //og obstacle.getTYpe()
        type =Type::AVOID;
    }
    else{
        type =Type::BASELINE;
    }
    direction = H(disturbance, d);  
    //BELOW IS DEFINED THE H (AGENT TRANSFER FUNCTION) OF THE TASK, which we define implicitly to the creation of the task
    action = Action(direction); //creates motor output
    }


Direction H(Disturbance ob, Direction d = Direction::NONE){
    if (ob.isValid()==true){
        if (ob.getType()==DisturbanceType::obstacle){
            //NEW LOOP FOR ABOVE
            if (d == Task::Direction::NONE){ //REACTIVE BEHAVIOUR
                if (ob.getAngle()<0)//angle formed with robot at last safe pose
                    d = Task::Direction::LEFT; //go left
                }
                else if (ob.getAngle()>0){ //angle formed with robot at last safe pose
                    d = Task::Direction::RIGHT; //
                }   
                else{
                    int c = rand() % 2;
                    d = static_cast<Task::Direction>(c);

                }
            }
        }
    return d;
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


struct M{
    float L, R, omega, linearSpeed;
    M(){}
    M(float l, float r):
        L(l), 
        R(r)
    {
        omega = (MAX_SPEED*(R-L)/BETWEEN_WHEELS); //instant velocity, determines angle increment in willcollide
        linearSpeed = MAX_SPEED*(L+R)/2;   
    }

};


};