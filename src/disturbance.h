#include "settings.h"
//#include "measurement.h"
#include "robot.h"
#include <stdexcept>

struct Disturbance{ //this generates error
private:
    AffordanceIndex affordanceIndex = 0; //not using the enum because in the future we might want to add more affordances
    float angleToRobot=0;
    bool partOfObject=0;
public:
    bool valid= 0;
	b2FixtureDef fixtureDef;
    b2Transform pose = {b2Vec2(2*BOX2DRANGE, 2*BOX2DRANGE), b2Rot(M_PI)};
   // bool safeForNow=1;
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
		pose.Set(p, 0);
        valid =1;
    }    

        Disturbance(AffordanceIndex i, b2Vec2 p, float a){
        if (i>affordances.size()-1){
            throw std::invalid_argument("Not a valid affordance index\n");
        }
        else{
            affordanceIndex = i;
        }
		pose.Set(p, a);
        valid =1;
        partOfObject=1;
    }    

    void setAngle(float a){ //angle to robot
        angleToRobot =a; 
    }
    void setAngle(b2Transform t){ //angle to robot
        //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
        float angle;
        b2Vec2 thisToB;
        thisToB.x = pose.p.x-t.p.x;
        thisToB.y = pose.p.y - t.p.y;
        float cosA = (thisToB.x * cos(t.q.GetAngle())+ thisToB.y*sin(t.q.GetAngle()))/thisToB.Length();
        angleToRobot = acos(cosA);
    }


    float getAngle(b2Transform t){ //gets the angle of an Disturbance wrt to another Disturbance (robot)
        //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
        float angle;
        b2Vec2 thisToB;
        thisToB.x = pose.p.x-t.p.x;
        thisToB.y = pose.p.y - t.p.y;
        float cosA = (thisToB.x * cos(t.q.GetAngle())+ thisToB.y*sin(t.q.GetAngle()))/thisToB.Length();
        angle = acos(cosA);
        return angle;
    }

    float getAngle(b2Body* b){
        return getAngle(b->GetTransform());
    }

    float getAngle(){
        return angleToRobot;
    }

    void setPosition(b2Vec2 pos){
        pose.p.Set(pos.x, pos.y);
    }
    
    void setPosition(float x, float y){
        pose.p.Set(x, y);
    }
    
    b2Vec2 getPosition(){
        return pose.p;
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

    void setOrientation(float f){ //returns orientation (angle) of a point, in order 
        pose.q.Set(f);
        partOfObject =1;
    }

    float getOrientation(){
        return pose.q.GetAngle();
    }

    bool isPartOfObject(){
        return partOfObject;
    }

}; //sub action f


struct simResult{
    enum resultType {successful =0, crashed =1, safeForNow=2}; //successful means no collisions, finished means target reached, for later
    resultType resultCode;
    Disturbance collision;
    bool valid = 0;
    //float distanceCovered =0;
    b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    int step=0;
  //  int step=0;


    simResult(){}

    simResult(resultType code): resultCode(code){
        valid =1;
    }

    simResult(resultType code, Disturbance obst): resultCode(code), collision(obst){
        valid =1;
    }
};