#pragma once
#include "Box2D/Box2D.h"
#include <vector>
#include <stdio.h>
#include <math.h> //recommended cmath?

enum ObjectType {obstacle=0, target=1, other=2};   


class State{

enum stateType {BASELINE =0, AVOID =1, PURSUE =2};

protected:
    stateType type;
    float maxSpeed = 0.125f; //this needs to be defined better
    //b2Vec2 EstimatedLinearVelocity = {float((-sin(omega/hz))), float(cos(omega/hz))}; //is is sin and y is cos, this is pretty much instantaneous to get the system started
    b2Vec2 desiredVelocity, gain;//={maxSpeed *EstimatedLinearVelocity.x, maxSpeed*EstimatedLinearVelocity.y}; //velocity recorded at t. if no data is available it falls back on the prediction
    b2Vec2 RecordedVelocity ={0.0f, 0.0f};
    float simDuration =3;
    float pGain=0.1;
public:
    int planNo=0;
    float hz =50.0f; //og was 60
    float accumulatedError=0;
    char planFile[250];
    //int timesPlanned =0;
    //bool valid;


struct Object{ //maybe later can susbtitute this for a broader object so you can also set a target without having to make another class for it. Bernd has an enum object identifier
private:
    //b2Vec2 where;
    float step;
    bool valid= 0;
    ObjectType type;
    int iteration;
    float angleToRobot=0;
    b2Vec2 distance;
public:
	b2FixtureDef fixtureDef;
    b2BodyDef bodyDef;
    b2Body * body;
    Object(){};
    Object(ObjectType _t): type(_t){}
    Object(ObjectType _t, b2Vec2 position):type(_t){
        bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(position.x, position.y);
		// b2CircleShape fixture; //giving the point the shape of a box
        // fixture.m_radius = .05;
		// fixtureDef.shape = &fixture;
		//fixtureDef.shape->m_radius = .05;
        valid =1;
    }

    void setAngle(float a){
        angleToRobot =a;
    }


    float getAngle(b2Vec2 posVector){ //gets the angle of an object wrt to another object (robot)
        //reference is position vector 2. If the angle >0 means that object 1 is to the left of object 2
        float angle1=0;
        float angle2 =0;
        if (bodyDef.position.y !=0 && bodyDef.position.x !=0){ //inclusive or?
            angle1 = atan(bodyDef.position.y/bodyDef.position.x); //own angle to the origin 
        }
        if (posVector.y != 0 && posVector.y !=0){
            angle2 = atan(posVector.y/posVector.x);
        }
        //printf("angle1 =%f, angle2 =%f\n", angle1, angle2);
	    float angle = angle1-angle2;
        return angle;
    }

    float getAngle(float angle2){ //gets the angle of an object wrt to the heading direction of another object
        //reference is position vector 2. If the angle >0 means that object 1 is to the left of object 2
        float angle1=0;
        if (bodyDef.position.y !=0 && bodyDef.position.x !=0){ //inclusive or?
            angle1 = atan(bodyDef.position.y/bodyDef.position.x); //own angle to the origin 
        }
        //printf("angle1 =%f, angle2 =%f\n", angle1, angle2);
	    float angle = angle1-angle2;
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
    void setTimestamp(int _step, int _it){
        step=_step;
        iteration=_it;
    }

    // b2FixtureDef getFixtureDef(){
    //     return fixtureDef;
    // }

    bool isValid(){
        return valid;
    }

    ObjectType getType(){
        return type;
    }

    void invalidate(){
        valid =0;
    }
}; //sub action f

struct Constraint{
    enum Type {NONE =0, LEFT =1, RIGHT =2};
    Type type;

    Constraint(){
        type = NONE;
    }

};

struct Action{
private:
    //trajecotry is a function of the position of the disturbance
    float linearSpeed=.0625; //used to calculate instantaneous velocity using omega
    float omega=0; //initial angular velocity is 0  
    //b2Vec2 velocity ={0.5, 0};
    bool valid=0;
    float distanceBetweenWheels = 0.08f;
    float maxOmega = M_PI; //calculated empirically with maxspeed of .125
    float minAngle = M_PI_2; //turn until the angle between the distance vector and the velocity 
    //float angleAtStart;
public:
    float RightWheelSpeed=0.5;
    float LeftWheelSpeed=0.5;


    Action(){}

    Action(Object &ob, float simDuration=3, float maxSpeed=0.125, float hz=60.0f, b2Vec2 pos = {0,0}, State::Constraint constraint= State::Constraint()){
    float minWheelSpeedIncrement =0.01; //gives an omega of around .9 degrees/s given a maxSpeed of .125
    float maxDistance = maxSpeed*simDuration;
    if (ob.isValid()==true){
        if (ob.getType()==ObjectType::obstacle){
            //printf("angle to robot: %f\n", abs(ob.getAngle(pos)));
            if (abs(ob.getAngle(pos))<M_PI_2){
                // if (ob.getPosition().y<0){ //obstacle is to the right, vehicle goes left; ipsilateral excitatory, contralateral inhibitory
                //     LeftWheelSpeed = -LeftWheelSpeed; //go left
                // }
                // else if (ob.getPosition().y>0){ //go right
                //     RightWheelSpeed= - RightWheelSpeed;
                // }  
                // else if (constraint.type==State::Constraint::Type::NONE){ //if there are no constraints on the direction other than where the obstacle is, pick at random
                //     int goL = rand()%2;
                //     switch (goL)
                //     {
                //     case 0: //
                //     LeftWheelSpeed = - LeftWheelSpeed;
                //     break;
                //     case 1:
                //     RightWheelSpeed = -RightWheelSpeed;
                //     break;
                //     default:
                //     printf("invalid case\n");
                //         break;
                //     }
                // }
                if (constraint.type==State::Constraint::Type::NONE){ //if there are no constraints on the direction other than where the obstacle is, pick at random
                    if (ob.getPosition().y<0){ //obstacle is to the right, vehicle goes left; ipsilateral excitatory, contralateral inhibitory
                        constraint.type = State::Constraint::Type::LEFT; //go left
                    }
                    else if (ob.getPosition().y>0){ //go right
                        constraint.type = State::Constraint::Type::RIGHT; //go left
                    }   
                    else{
                        int c = rand() % 2;
                        constraint.type = static_cast<State::Constraint::Type>(c);

                    }
                }
            }
            else{
                ob.invalidate();
                //printf("invalidated obstacle because angle >pi/2\n");
            }
     
        }

        switch (constraint.type){
            case State::Constraint::Type::LEFT:
            LeftWheelSpeed = -LeftWheelSpeed;
            break;
            case State::Constraint::Type::RIGHT:
            RightWheelSpeed = - RightWheelSpeed;
            break;
            default:
            break;
        }

        if (LeftWheelSpeed >1.0f){
            LeftWheelSpeed=1;
        }
        if (RightWheelSpeed >1.0f){
            RightWheelSpeed =1;
        }
        if (LeftWheelSpeed <-1.0f){
            LeftWheelSpeed=-1;
        }
        if (RightWheelSpeed <-1.0f){
            RightWheelSpeed =-1;
        }
    }


    omega = (maxSpeed*(LeftWheelSpeed-RightWheelSpeed)/distanceBetweenWheels); //instant velocity, determines angle increment in willcollide
        //printf("omega = %f pi\n", omega/M_PI);
        if (abs(omega)>M_PI){ //max turning angle in one second
            float multiplier=1;
            if (omega<0){
                multiplier=-1;
            }
            omega=M_PI*multiplier;
        }

    linearSpeed = maxSpeed*(LeftWheelSpeed+RightWheelSpeed)/2;
    //linearSpeed = distanceBetweenWheels*omega;
    if (abs(linearSpeed)>maxSpeed){
        float multiplier=1;
    if (linearSpeed<0){
        multiplier=-1;
    }
    linearSpeed=maxSpeed*multiplier;
    }
    //printf("linear speed = %f\n", linearSpeed);


    valid=1;
    }



    b2Vec2 getLinearVelocity(float maxV = 0.125){
        b2Vec2 vel;
        float realL = maxV*LeftWheelSpeed;
        float realR = maxV*RightWheelSpeed;
            //find angle theta in the pose:
        float W = (realL-realR)/distanceBetweenWheels; //rad/s, final angle at end of 1s
        //find absolute speed
        float V =(realL+realR)/2; //velocity
        if (realR-realL == 0){
            vel.x = realL;
            vel.y = 0;
            }
        else {
            vel.x = (distanceBetweenWheels/2)* sin(distanceBetweenWheels/(realR-realL));
            vel.y = -(distanceBetweenWheels/2)* cos(distanceBetweenWheels/(realR-realL));

        }
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




};


struct simResult{
    enum resultType {successful =0, crashed =1, finished =2}; //successful means no collisions, finished means target reached, for later
    resultType resultCode;
    Object collision;
    bool valid = 0;

    simResult(){}

    simResult(resultType code): resultCode(code){
        valid =1;
    }

    simResult(resultType code, int it, Object obst): resultCode(code), collision(obst){
        collision.setIteration(it);
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
Object obstacle;
Object target;

State::Action getAction(){
    return action;
}

State(){
    action = Action(); //this is a valid trajectory, default going straight at moderate speed
    type = stateType::BASELINE;
    printf("in state: L=%f\t R=%f\n", getAction().getLWheelSpeed(), getAction().getRWheelSpeed());

}

State(Object ob){
    action = Action(ob, simDuration, maxSpeed, hz); 
    obstacle = ob;
    if (obstacle.getType()== ObjectType::obstacle){
        type =stateType::AVOID;
    }
    printf("in state: L=%f\t R=%f\n", getAction().getLWheelSpeed(), getAction().getRWheelSpeed());

}

//other state to set both target and obstacle

void setObstacle(Object ob){
    obstacle = ob;
}


float getMaxSpeed(){
    return maxSpeed;
}

void setHz(float _hz){
    hz = _hz;
}

void setSimDuration(int d){
    simDuration = d;
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
    //printf("omega = %f pi\n", W/M_PI);
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

void trackObject(Object &, float, b2Vec2, b2Vec2);

State::simResult willCollide(b2World &, int);

enum controlResult{DONE =0, CONTINUE =1};

controlResult controller();

void tunePGain(float yError){ //unused, not tested
    float learningRate = 0.01;
    pGain -= yError*learningRate;
}

private:



};