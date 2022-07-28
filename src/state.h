#pragma once
#include <vector>
#include <stdio.h>
#include <math.h> //recommended cmath?
#include "Box2D/Box2D.h"

enum ObjectType {obstacle=0, target=1, other=2};   


class State{
protected:
   // StateType type;
    float maxSpeed = 0.125f; //this needs to be defined better
    float hz =60.0f;
    //b2Vec2 EstimatedLinearVelocity = {float((-sin(omega/hz))), float(cos(omega/hz))}; //is is sin and y is cos, this is pretty much instantaneous to get the system started
    b2Vec2 RecordedVelocity;//={maxSpeed *EstimatedLinearVelocity.x, maxSpeed*EstimatedLinearVelocity.y}; //velocity recorded at t. if no data is available it falls back on the prediction
    float simDuration =3;
public:
int planNo=0;
int timesPlanned =0;
//bool valid;

struct Object{ //maybe later can susbtitute this for a broader object so you can also set a target without having to make another class for it. Bernd has an enum object identifier
private:
    b2Vec2 where;
    float step;
    bool valid= 0;
    //bool crashed;
    ObjectType type;
    float angleToRobot=0; // by default angle is at the robot's midline
    int iteration;
public:
    Object(){};
    Object(ObjectType _t): type(_t){}
    Object(ObjectType _t, b2Vec2 position):type(_t), where(position){
        valid =1;
    }

    void setAngle(float a){
        angleToRobot =a;
    }

    float getAngle(){
        return angleToRobot;
    }

    void setPosition(b2Vec2 pos){
        where = pos;
        valid =1;
    }
    
    void setPosition(float x, float y){
        where.x = x;
        where.y = y;
        valid=1;
    }
    
    b2Vec2 getPosition(){
        return where;
    }


    void setIteration(int _it){
        iteration=_it;
    }
    void setTimestamp(int _step, int _it){
        step=_step;
        iteration=_it;
    }

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
    float omega=0; //initial angular velocity is 0  
   // b2Vec2 defaultRobPosition={0.0, 0.0};
    float minWheelSpeedIncrement =0.01; //gives an omega of around .9 degrees/s given a maxSpeed of .125
    //float desiredRelativeSpeed; //of maxspeed, this corresponds to the avg wheel speed (0.5. defualt)
    float linearSpeed=.5; //used to calculate instantaneous velocity using omega
    b2Vec2 velocity ={0, 0.5};
    bool valid=0;
    float RightWheelSpeed;
    float LeftWheelSpeed;
    float distanceBetweenWheels = 0.08f;
    float maxOmega = M_PI; //calculated empirically with maxspeed of .125
    float minAngle = M_PI_2; //turn until the angle between the distance vector and the velocity 
    bool safe;
public:
    Trajectory(){}

    Trajectory(Object ob, float simDuration, float maxSpeed, b2Vec2 vel ={0, 0.5}, b2Vec2 startPosition = {0.0, 0.0}){
    float defaultRightWheelSpeed = 0.5;
    float defaultLeftWheelSpeed=0.5;
    //float defaultRelativeVel = 0.5;
    float maxDistance = vel.Length()*simDuration;
    //b2Vec2 endPosition(startPosition.x+vel.x, startPosition.y+vel.y);
    //float StartToEnd = sqrt(pow(endPosition.x-startPosition.x, 2)+pow(endPosition.y-startPosition.y, 2)); //end is future velocity projection assuming constant v
        if (ob.isValid()){
            if (ob.getType() == ObjectType::obstacle){
                //printf("start = %f, %f\n", startPosition.x, startPosition.y);
                b2Vec2 ObToStartVec(ob.getPosition().x-startPosition.x, ob.getPosition().y -startPosition.y);
                float cosAngle = (ObToStartVec.x * vel.x + ObToStartVec.y * vel.y)/(vel.Length()*ObToStartVec.Length()); //dot product etc
                float angleAtStart = acos(cosAngle);
                //printf("distance from obstacle = %f\n", ObToStartVec.Length());
                //float ObstacleToStart = ObToStartVec.Length();
                //float ObstacleToEnd  =sqrt(pow(endPosition.x-ob.getPosition().x, 2)+pow(endPosition.y-ob.getPosition().y,2));
            //the angle at the start vertex is opposite to ObstacleToEnd (WE ARE INTERESTED IN THIS ONE)
                //float angleAtStart = (pow(StartToEnd,2)+pow(ObstacleToEnd,2) - pow(ObstacleToEnd,2))/(2*ObstacleToStart*StartToEnd);
                ob.setAngle(angleAtStart);
                float angleStimulus = 1-(angleAtStart/minAngle);
               // printf("anglestimulus = %f\n", angleStimulus);
                if (angleStimulus<0){
                    angleStimulus=0;
                }
                float distanceStimulus =1-(ObToStartVec.Length()/maxDistance);
               // printf("distancestimulus = %f\n", distanceStimulus);
                if (distanceStimulus<0){
                    distanceStimulus=0;
                }
                float stimulusMagnitude = distanceStimulus *angleStimulus*100; //stimMagnitude as fraction of maxDistance (percentage)
              //  printf("stimulus magnitude: %f\n", stimulusMagnitude);
                //steering has a linear relationship with distance
                if (ObToStartVec.x<0){ //obstacle is to the right, vehicle goes left; ipsilateral excitatory, contralateral inhibitory
                    LeftWheelSpeed=defaultLeftWheelSpeed-minWheelSpeedIncrement*(stimulusMagnitude);
                    RightWheelSpeed=defaultRightWheelSpeed+minWheelSpeedIncrement*(stimulusMagnitude);
                }
                else if (ObToStartVec.x>0){ //go right
                    LeftWheelSpeed=defaultLeftWheelSpeed+minWheelSpeedIncrement*(stimulusMagnitude);
                    RightWheelSpeed=defaultRightWheelSpeed-minWheelSpeedIncrement*(stimulusMagnitude);
                }
                else {
                    int isLeft = rand()%2; //if x==0 the direction is random
                    switch (isLeft){
                        case 1:     //go left            
                            LeftWheelSpeed=defaultLeftWheelSpeed-minWheelSpeedIncrement*(stimulusMagnitude);
                            RightWheelSpeed=defaultRightWheelSpeed+minWheelSpeedIncrement*(stimulusMagnitude);
                        break;
                        case 0: //go right
                            LeftWheelSpeed=defaultLeftWheelSpeed+minWheelSpeedIncrement*(stimulusMagnitude);
                            RightWheelSpeed=defaultRightWheelSpeed-minWheelSpeedIncrement*(stimulusMagnitude);
                        break;
                        default:printf("invalid multiplier\n"); LeftWheelSpeed=defaultLeftWheelSpeed; RightWheelSpeed=defaultRightWheelSpeed; break;
                    }
                }
            }
    }
    else{
        LeftWheelSpeed = defaultLeftWheelSpeed;
        RightWheelSpeed = defaultRightWheelSpeed;
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

    omega = maxSpeed*(LeftWheelSpeed-RightWheelSpeed)/distanceBetweenWheels;
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
    velocity ={-linearSpeed *sin(omega), linearSpeed*cos(omega)};
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

    void setPreviousVelocity(b2Vec2 vel){
        velocity = vel;
    }

    b2Vec2 getVelocity(){
        return velocity;
    }


    void setSafe(bool s){
        safe =s;
    }

    bool isSafe(){
        return safe;
    }


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
    Listener(int it): iteration(it){}
    bool contactEnded=0;
    //Listener(b2Body * rb): robot(rb){}
    std::vector <Object> collisions;
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
                collisions.push_back(Object(ObjectType::obstacle, {other->GetPosition()}));
                
                //DEBUG
                char name[250];
                sprintf(name,"manifold%04i.dat", iteration);
                FILE* file = fopen(name, "w+");
                printf("obstacle : %f, %f\n", other->GetPosition().x, other->GetPosition().y);
                printf("robot: %f, %f\n", contact->GetFixtureA()->GetBody()->GetPosition().x, contact->GetFixtureA()->GetBody()->GetPosition().y);
                for (int i=0; i<numPoints; i++){
                    fprintf(file, "%f\t%f\n", wm.points[i].x, wm.points[i].y);
                }
			}


			bodyData = contact->GetFixtureB()->GetBody()->GetUserData();
			if (bodyData) {
               // Robot *robot = static_cast<Robot*>(bodyData);
               // collision= Object(robot->body->GetPosition());
                b2Body * other = contact->GetFixtureA()->GetBody();
                collisions.push_back(Object(ObjectType::obstacle, {other->GetPosition()}));
                //DEBUG
                char name[250];
                sprintf(name,"/tmp/manifold%04i.dat", iteration);
                FILE* file = fopen(name, "w+");

                printf("obstacle: %f, %f\n", other->GetPosition().x, other->GetPosition().y);
                printf("robot: %f, %f\n", contact->GetFixtureB()->GetBody()->GetPosition().x, contact->GetFixtureB()->GetBody()->GetPosition().y);
                //collision.setPosition(other->GetPosition());
                for (int i=0; i<numPoints; i++){
                    fprintf(file, "%f\t%f\n", wm.points[i].x, wm.points[i].y);
                }

                }
                
        //CHECK IF YOU CAN JUST CHANGE VARIABLES WITHIN STATE
		}

        void EndContact(b2Contact * contact){
            printf("contact ends here");
            contactEnded=1;
        }

	};

Trajectory trajectory;
Object obstacle;
Object target;

State(){
    trajectory = Trajectory(obstacle, simDuration, maxSpeed); //this is a valid trajectory, default going straight at moderate speed

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

// float predictOmega(float max){ //returns a prediction of what the angular velocity should be with the current settings
//     return max*(RightWheelSpeed-LeftWheelSpeed)/distanceBetweenWheels;
// }

// float predictMaxSpeed(){
//     return RecordedVelocity.Length()/(RightWheelSpeed+LeftWheelSpeed)/2; //linear velocity of robot body
// }

bool hasCollision(){
    return obstacle.isValid(); //if vector containing collisions is not of size 0, the current plan has crashed
}



void setRecordedVelocity(b2Vec2 vel){
    RecordedVelocity = vel;
} //useful to get the speed.


b2Vec2 getRecordedVelocity(){
    return RecordedVelocity;
}


State::Trajectory getTrajectory(){
    return trajectory;
}

void recomputeTrajectory(){
    trajectory = Trajectory(obstacle, simDuration, maxSpeed);
}

void trackObject(Object &, float, b2Vec2, b2Vec2);

void followPath(); //P-controller: takes in error, returns adjusted commands for alphabot. Take in error as angle! not point


State::simResult willCollide(b2World &, int);


float averageMaxSpeed(); //read into files and find mean/median/mode of speed to get a better more precise estimation

void noise(); //create a normal distribution for data (position etc) to find a standard deviation and generate a tolerance interval for that parameter

b2Vec2 estimateVelocityFromWheel(float max, float L, float R, float d){
    	//find angular velocity
	float angVel = max *(L-R)/d; //rad/s
	//find absolute speed
	float speed = max * (L+R)/2; //distance covered
	//find velocity vector: this is per second so no need to change this; but it is the instantaneous velocity
	b2Vec2 vel(speed*sin(angVel), speed*cos(angVel));
	return vel;
}

void findWheelSpeed(); //find the values to enter for the robot wheels

// void switchState(){} //transition function : the listener is the transition function 



private:



};