#ifndef DISTURBANCE_H
#define DISTURBANCE_H
#include "settings.h"
//#include "measurement.h"
#include "robot.h"
//#include "opencv2/opencv.hpp"
#include <stdexcept>

class BodyFeatures{
    public:
    b2Transform pose {b2Transform(b2Vec2(0,0), b2Rot(0))} ;
    float halfLength=0.001; //x
    float halfWidth=0.001; //y
    float shift=0.0f;
    b2BodyType bodyType = b2_dynamicBody;
    b2Shape::Type shape = b2Shape::e_polygon;
    b2FixtureDef fixtureDef;

    BodyFeatures(){}

    BodyFeatures(b2Transform _pose):pose(_pose){}



};

struct Disturbance{ //this generates error
private:
    AffordanceIndex affordanceIndex = 0; //not using the enum because in the future we might want to add more affordances
    float angleToRobot=0;
    //bool partOfObject=0;
public:
    bool valid= 0;
    BodyFeatures bf=BodyFeatures(b2Transform(b2Vec2(2*BOX2DRANGE, 2*BOX2DRANGE), b2Rot(M_PI)));
    //b2Transform pose = {b2Vec2(2*BOX2DRANGE, 2*BOX2DRANGE), b2Rot(M_PI)};
    
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
		bf.pose.Set(p, 0);
        valid =1;
    }    

        Disturbance(AffordanceIndex i, b2Vec2 p, float a){
        if (i>affordances.size()-1){
            throw std::invalid_argument("Not a valid affordance index\n");
        }
        else{
            affordanceIndex = i;
        }
		bf.pose.Set(p, a);
        valid =1;
        //partOfObject=1;
    }    

    void setAngle(float a){ //angle to robot
        angleToRobot =a; 
    }
    void setAngle(b2Transform);


    float getAngle(b2Transform);

    float getAngle(b2Body* b){
        return getAngle(b->GetTransform());
    }

    float getAngle(){
        return angleToRobot;
    }

    void setPosition(b2Vec2 pos){
        bf.pose.p.Set(pos.x, pos.y);
    }
    
    void setPosition(float x, float y){
        bf.pose.p.Set(x, y);
    }
    
    b2Vec2 getPosition(){
        return bf.pose.p;
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
        bf.pose.q.Set(f);
       // partOfObject =1;
    }

    float getOrientation(){
        return bf.pose.q.GetAngle();
    }

    // bool isPartOfObject(){
    //     return partOfObject;
    // }

    b2Transform pose(){
        return bf.pose;
    }

    void setPose(b2Transform t){
        bf.pose=t;
    }

}; //sub action f


struct simResult{
    enum resultType {successful =0, crashed =1, safeForNow=2}; //successful means no collisions, finished means target reached, for later
    resultType resultCode= successful;
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

#endif