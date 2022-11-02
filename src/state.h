#pragma once
#include "Box2D/Box2D.h"
#include <vector>
#include <stdio.h>
#include <math.h> //recommended cmath?

enum ObjectType {obstacle=0, target=1, other=2};   


class State{
protected:
   // StateType type;
    float maxSpeed = 0.2f; //this needs to be defined better
    //b2Vec2 EstimatedLinearVelocity = {float((-sin(omega/hz))), float(cos(omega/hz))}; //is is sin and y is cos, this is pretty much instantaneous to get the system started
    b2Vec2 desiredVelocity, gain;//={maxSpeed *EstimatedLinearVelocity.x, maxSpeed*EstimatedLinearVelocity.y}; //velocity recorded at t. if no data is available it falls back on the prediction
    b2Vec2 RecordedVelocity ={0.0f, 0.0f};
    float simDuration =3;
public:
int planNo=0;
float hz =60.0f;
//int timesPlanned =0;
//bool valid;

struct Object{ //maybe later can susbtitute this for a broader object so you can also set a target without having to make another class for it. Bernd has an enum object identifier
private:
    //b2Vec2 where;
    float step;
    bool valid= 0;
    ObjectType type;
    float angleToRobot=0; // by default angle is at the robot's midline
    int iteration;
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


    float getAngle(){
        return angleToRobot;
    }

    float getAngle(b2Vec2 robVelocity, b2Vec2 robPos){ //gets the angle of an object wrt to another object (robot)
        b2Vec2 distance = b2Vec2(bodyDef.position.x-robPos.x, bodyDef.position.y-robPos.y);
        // printf("distance vector: %f, %f\n", distance.x, distance.y);
        // printf("distance lenght: %f\n", distance.Length());
        // printf("robVelocity lenght %f\n", robVelocity.Length());
	    float angle = acos((robVelocity.x*distance.x+robVelocity.y*distance.y)/(robVelocity.Length()*distance.Length()));
       // printf("hi\n");
        return angle;
    }

    // b2Transform  GetTransform(){
    //     b2Transform t;
    //     t.Set(bodyDef.position, angleToRobot);
    //     return t;
    // }

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
    float linearSpeed=.5; //used to calculate instantaneous velocity using omega
    float omega=0; //initial angular velocity is 0  
    b2Vec2 velocity ={0.5, 0};
    bool valid=0;
    float RightWheelSpeed=0.5;
    float LeftWheelSpeed=0.5;
    float distanceBetweenWheels = 0.08f;
    //float angleAtStart;
public:
    Trajectory(){}

    Trajectory(Object &ob, float simDuration=3, float maxSpeed=0.2, float hz=60.0f, b2Vec2 vel ={0.5, 0}, b2Vec2 startPosition = {0.0, 0.0}){
    float minWheelSpeedIncrement =0.01; //gives an omega of around .9 degrees/s given a maxSpeed of .125
    float maxOmega = M_PI; //calculated empirically with maxspeed of .125
    float minAngle = M_PI_2; //turn until the angle between the distance vector and the velocity 
    float maxDistance = maxSpeed*simDuration;
  //  printf("max speed = %f, simDuration %f, max dist %f\n", maxSpeed, simDuration, maxDistance);
        if (ob.isValid()==true){
             b2Vec2 ObToStartVec(ob.getPosition().x-startPosition.x, ob.getPosition().y -startPosition.y);
            float angleAtStart = acos((ObToStartVec.x * vel.x + ObToStartVec.y * vel.y)/(vel.Length()*ObToStartVec.Length())); //dot product etc
            //printf("angle between velocity and obstacle %f pi\n", angleAtStart/M_PI);
            if (ob.getType() == ObjectType::obstacle){
                float angleStimulus = 1-(abs(angleAtStart)/minAngle);
                //printf("angleStimulus %f \n", angleStimulus);
                if (angleStimulus<0){
                    angleStimulus=0;
                }
                float distanceStimulus =1-(ObToStartVec.Length()/maxDistance);
                //printf("distance to obst %f\t\n", ObToStartVec.Length());
                //printf("distanceStimulus %f\n", distanceStimulus);
                if (distanceStimulus<0){
                    distanceStimulus=0;
                }
                float stimulusMagnitude = distanceStimulus *angleStimulus; //stimMagnitude as fraction of maxDistance (percentage)
                printf("stimulus magnitude: %f\n", stimulusMagnitude);
                //steering has a linear relationship with distance
                if (ObToStartVec.y<0){ //obstacle is to the right, vehicle goes left; ipsilateral excitatory, contralateral inhibitory
                    LeftWheelSpeed-=minWheelSpeedIncrement*(stimulusMagnitude);
                    RightWheelSpeed+=minWheelSpeedIncrement*(stimulusMagnitude);
                }
                else if (ObToStartVec.y>0){ //go right
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
    // else{
    //     LeftWheelSpeed = defaultLeftWheelSpeed;
    //     RightWheelSpeed = defaultRightWheelSpeed;
    // }
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
            //MANIFOLD STUFF IS FOR DEBUGGING
            int numPoints = contact->GetManifold()->pointCount;
            b2WorldManifold wm;
            contact->GetWorldManifold(&wm);
			void* bodyData = contact->GetFixtureA()->GetBody()->GetUserData();
			if (bodyData) {
               // Robot *robot = static_cast<Robot*>(bodyData);
                b2Body * other = contact->GetFixtureB()->GetBody();
                //collision.setPosition(other->GetPosition());
                //collisions.push_back(Object(ObjectType::obstacle, {other->GetPosition()}));
                collisions.push_back(other->GetPosition());
                // nprintf("other->GetPosition = %f, %f", other->GetPosition().x, other->GetPosition().y);
                
                //DEBUG + use constructor with iteration
                // char name[250];
                // sprintf(name,"manifold%04i.dat", iteration);
                // FILE* file = fopen(name, "w+");
                //printf("obstacle : %f, %f\n", other->GetPosition().x, other->GetPosition().y);
                // printf("robot: %f, %f\n", contact->GetFixtureA()->GetBody()->GetPosition().x, contact->GetFixtureA()->GetBody()->GetPosition().y);
                // for (int i=0; i<numPoints; i++){
                //     fprintf(file, "%f\t%f\n", wm.points[i].x, wm.points[i].y);
                // }
			}


			bodyData = contact->GetFixtureB()->GetBody()->GetUserData();
			if (bodyData) {
               // Robot *robot = static_cast<Robot*>(bodyData);
               // collision= Object(robot->body->GetPosition());
                b2Body * other = contact->GetFixtureA()->GetBody();
               // collisions.push_back(Object(ObjectType::obstacle, {other->GetPosition()}));
                collisions.push_back(other->GetPosition());
                //printf("other->GetPosition = %f, %f", other->GetPosition().x, other->GetPosition().y);

                
                
                //DEBUG
                // char name[250];
                // sprintf(name,"/tmp/manifold%04i.dat", iteration);
                // FILE* file = fopen(name, "w+");

                 //printf("obstacle: %f, %f\n", other->GetPosition().x, other->GetPosition().y);
            // printf("robot: %f, %f\n", contact->GetFixtureB()->GetBody()->GetPosition().x, contact->GetFixtureB()->GetBody()->GetPosition().y);
                // //collision.setPosition(other->GetPosition());
                // for (int i=0; i<numPoints; i++){
                //     fprintf(file, "%f\t%f\n", wm.points[i].x, wm.points[i].y);
                // }

                }
                
        //CHECK IF YOU CAN JUST CHANGE VARIABLES WITHIN STATE
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