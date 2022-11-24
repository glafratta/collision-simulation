#pragma once
#include "Box2D/Box2D.h"
#include <vector>
#include <stdio.h>
#include <math.h> //recommended cmath?

enum ObjectType {obstacle=0, target=1, other=2};   


class State{
protected:
   // StateType type;
    float maxSpeed = 0.125f; //this needs to be defined better
    //b2Vec2 EstimatedLinearVelocity = {float((-sin(omega/hz))), float(cos(omega/hz))}; //is is sin and y is cos, this is pretty much instantaneous to get the system started
    b2Vec2 desiredVelocity, gain;//={maxSpeed *EstimatedLinearVelocity.x, maxSpeed*EstimatedLinearVelocity.y}; //velocity recorded at t. if no data is available it falls back on the prediction
    b2Vec2 RecordedVelocity ={0.0f, 0.0f};
    float simDuration =3;
public:
int planNo=0;
float hz =60.0f;
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


    float getAngle(b2Vec2 posVector2){ //gets the angle of an object wrt to another object (robot)
        //reference is position vector 2. If the angle >0 means that object 1 is to the left of object 2
        float angle1 = atan(bodyDef.position.y/bodyDef.position.x); //own angle to the origin
        float angle2 = atan(posVector2.y/posVector2.x);
	    float angle = angle1-angle2;
       // printf("hi\n");
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


struct Trajectory{
private:
    //trajecotry is a function of the position of the disturbance
    float linearSpeed=.0625; //used to calculate instantaneous velocity using omega
    float omega=0; //initial angular velocity is 0  
    //b2Vec2 velocity ={0.5, 0};
    bool valid=0;
    float RightWheelSpeed=0.5;
    float LeftWheelSpeed=0.5;
    float distanceBetweenWheels = 0.08f;
    //float angleAtStart;
public:
    Trajectory(){}

    Trajectory(Object &ob, float simDuration, float maxSpeed, b2Vec2 vel ={0.0625, 0}, b2Vec2 startPosition = {0.0, 0.0}){
    float minWheelSpeedIncrement =0.01; //gives an omega of around .9 degrees/s given a maxSpeed of .125
    float maxOmega = M_PI; //calculated empirically with maxspeed of .125
    float maxDistance = .6;
    float stimulusMagnitude =0;
  //  printf("max speed = %f, simDuration %f, max dist %f\n", maxSpeed, simDuration, maxDistance);
        if (ob.isValid()==true){
            printf("obstacle at: %f\t%f\n", ob.getPosition().x, ob.getPosition().y);
            //b2Vec2 ObToStartVec(ob.getPosition().x-startPosition.x, ob.getPosition().y -startPosition.y);
            float angleAtStart = ob.getAngle(vel);
            // float angleAtStart = acos((ObToStartVec.x * vel.x+ ObToStartVec.y * vel.y)/(vel.Length()*ObToStartVec.Length())); //dot product etc
            //printf("angle between velocity and obstacle %f pi\n", angleAtStart/M_PI);
            if (ob.getType() == ObjectType::obstacle){
                float angleStimulus = 1-(abs(angleAtStart)/M_PI_2);
                printf("angleStimulus %f \n", angleStimulus);
                if (angleStimulus<0){
                    angleStimulus=0;
                }
                float distanceStimulus =1-(ob.getPosition().Length()/maxDistance);
                printf("distance to obst %f\t\n", ob.getPosition().Length());
                printf("distanceStimulus %f\n", distanceStimulus);
                if (distanceStimulus<0){ //if the object is too far don't bother making a plan to avoid it
                    distanceStimulus=0;
                }
                stimulusMagnitude = distanceStimulus *angleStimulus; //stimMagnitude as fraction of maxDistance (percentage)
                printf("stimulus magnitude: %f\n", stimulusMagnitude);
                //steering has a linear relationship with distance
                if (ob.getPosition().y<0){ //obstacle is to the right, vehicle goes left; ipsilateral excitatory, contralateral inhibitory
                    LeftWheelSpeed-=minWheelSpeedIncrement*(stimulusMagnitude);
                    RightWheelSpeed+=minWheelSpeedIncrement*(stimulusMagnitude);
                }
                else if (ob.getPosition().y>0){ //go right
                    LeftWheelSpeed+=minWheelSpeedIncrement*(stimulusMagnitude);
                    RightWheelSpeed-=minWheelSpeedIncrement*(stimulusMagnitude);
                }
                else {
                    int isLeft = rand()%2; //if x==0 the direction is random
                    switch (isLeft){
                        case 1:     //go left            
                            LeftWheelSpeed-=minWheelSpeedIncrement*(stimulusMagnitude);
                            RightWheelSpeed+=minWheelSpeedIncrement*(stimulusMagnitude);
                        break;
                        case 0: //go right
                            LeftWheelSpeed+=minWheelSpeedIncrement*(stimulusMagnitude);
                            RightWheelSpeed-=minWheelSpeedIncrement*(stimulusMagnitude);
                        break;
                        default:printf("invalid multiplier\n"); break;
                    }
                }
            }
            else if (ob.getType() == ObjectType::target){
                
            }
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

    omega = (maxSpeed*(RightWheelSpeed-LeftWheelSpeed)/distanceBetweenWheels); //instant velocity, determines angle increment in willcollide
    printf("omega = %f pi\n", omega/M_PI);
    if (abs(omega)>maxOmega){
        float multiplier=1;
        if (omega<0){
            multiplier=-1;
        }
        omega=maxOmega*multiplier;
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
    //printf("omega=%f, linearSpeed =%f, L=%f, R=%f, vel.x =%f, vel.y=%f\n", omega, linearSpeed, LeftWheelSpeed, RightWheelSpeed, velocity.x, velocity.y);
    }

    //TO DO: HAVE A FUNCTION UPDATE TRAJECTORY BASED ON OBSTACLE: steering is proportional to distance
 
   void findWheelSpeed();
        //system of equations: 
        //1. L-R= omega *dist
    
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




    // void setPreviousVelocity(b2Vec2 vel){
    //     velocity = vel;
    // }

    // b2Vec2 getVelocity(){
    //     velocity ={linearSpeed *cos(omega), linearSpeed*sin(omega)}; // at t=0
    //     return velocity;
    // }


    // void setSafe(bool s){
    //     safe =s;
    // }

    // bool isSafe(){
    //     return safe;
    // }


};

//or velocity?

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
	//register state with listener and add collision into state OR create actual state
  //  b2Body * robot;
  int iteration=1;
    public:
    Listener(){}
   // Listener(int it): iteration(it){}
   // bool contactEnded=0;
    //Listener(b2Body * rb): robot(rb){}
    std::vector <b2Vec2> collisions;
   // Listener(std::vector <Object> &collisions){};
	//Object collision = Object(ObjectType::obstacle);
		void BeginContact(b2Contact * contact) {
            // //MANIFOLD STUFF IS FOR DEBUGGING
            // int numPoints = contact->GetManifold()->pointCount;
            // b2WorldManifold wm;
            // contact->GetWorldManifold(&wm);
			void* bodyData = contact->GetFixtureA()->GetBody()->GetUserData();
			if (bodyData) {
               // Robot *robot = static_cast<Robot*>(bodyData);
                b2Body * other = contact->GetFixtureB()->GetBody();
                collisions.push_back(other->GetPosition());

			}


			bodyData = contact->GetFixtureB()->GetBody()->GetUserData();
			if (bodyData) {
               // Robot *robot = static_cast<Robot*>(bodyData);
               // collision= Object(robot->body->GetPosition());
                b2Body * other = contact->GetFixtureA()->GetBody();
               // collisions.push_back(Object(ObjectType::obstacle, {other->GetPosition()}));
                collisions.push_back(other->GetPosition());
                //printf("other->GetPosition = %f, %f", other->GetPosition().x, other->GetPosition().y);

                }
                
		}


	};

Trajectory trajectory;
Object obstacle;
Object target;

State(){
    trajectory = Trajectory(); //this is a valid trajectory, default going straight at moderate speed

}

State(Object ob){
    trajectory = Trajectory(ob, simDuration, maxSpeed); 
    obstacle = ob;
}

//other state to set both target and obstacle

void setObstacle(Object ob){
    obstacle = ob;
}


float getMaxSpeed(){
    return maxSpeed;
}


// bool hasCollision(){
//     return obstacle.isValid(); //if vector containing collisions is not of size 0, the current plan has crashed
// }



void setRecordedVelocity(b2Vec2 vel){
    RecordedVelocity = vel;
} //useful to get the speed.


b2Vec2 getRecordedVelocity(){
    return RecordedVelocity;
}


State::Trajectory getTrajectory(){
    return trajectory;
}

// void recomputeTrajectory(){
//     trajectory = Trajectory(obstacle, simDuration, maxSpeed);
// }

void trackObject(Object &, float, b2Vec2, b2Vec2);

//void controller(); //P-controller: takes in error, returns adjusted commands for alphabot. Take in error as angle! not point


State::simResult willCollide(b2World &, int);


float averageMaxSpeed(); //read into files and find mean/median/mode of speed to get a better more precise estimation

void noise(); //create a normal distribution for data (position etc) to find a standard deviation and generate a tolerance interval for that parameter

void findWheelSpeed(); //find the values to enter for the robot wheels

// void switchState(){} //transition function : the listener is the transition function 

void controller();





private:



};