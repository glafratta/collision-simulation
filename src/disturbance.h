#ifndef DISTURBANCE_H
#define DISTURBANCE_H
#include "settings.h"
//#include "measurement.h"
#include "robot.h"
//#include "opencv2/opencv.hpp"
#include <stdexcept>

class Configurator;
//class StateMatcher;

class BodyFeatures{
    public:
    b2Transform pose {b2Transform(b2Vec2(0,0), b2Rot(0))} ;
    float halfLength=0.0005; //x
    float halfWidth=0.0005; //y
    float shift=0.0f;
    b2BodyType bodyType = b2_dynamicBody;
    b2Shape::Type shape = b2Shape::e_polygon;

    BodyFeatures(){}

    BodyFeatures(b2Transform _pose):pose(_pose){}

    void setHalfLength(float f){
        halfLength=f;
    }

    void setHalfWidth(float f){
        halfWidth=f;
    }

};

struct Disturbance{ //this generates error
friend Configurator;
friend struct StateMatcher;
private:
    AffordanceIndex affordanceIndex = 0; //not using the enum because in the future we might want to add more affordances
    bool valid= 0;
    bool rotation_valid=0;    

    void setOrientation(float f){ //returns orientation (angle) of a point, in order 
        rotation_valid=1;
        bf.pose.q.Set(f);
    }

    void addToOrientation(float dtheta){
    if (rotation_valid){
        setOrientation(bf.pose.q.GetAngle()+dtheta);
    }
    else{
        setOrientation(dtheta);
    }
}
protected:
    BodyFeatures bf=BodyFeatures(b2Transform(b2Vec2(2*BOX2DRANGE, 2*BOX2DRANGE), b2Rot(M_PI)));

public:

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
        bf.pose.Set(p,a);
        valid =1;
    }   

    Disturbance(BodyFeatures _bf): bf(_bf){
        valid=1;
        affordanceIndex=AVOID;
    } 

    float getAngle(b2Transform);

    float getAngle(b2Body* b){
        return getAngle(b->GetTransform());
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

    void validate(){
        valid=1;
    }


    std::pair<bool, float> getOrientation(){
    
        return std::pair<bool, float>(rotation_valid, bf.pose.q.GetAngle());
    }

    b2Transform pose(){
        return bf.pose;
    }

    void setPose(b2Transform t){
        bf.pose=t;
    }

    void setAsBox(float w, float l){
        bf.halfLength=l;
        bf.halfWidth=w;
    }

    BodyFeatures bodyFeatures(){
        return bf;
    }


    void subtractPose(b2Transform dPose){
        bf.pose.p.x-=dPose.p.x;
        bf.pose.p.y-=dPose.p.y;
        addToOrientation(-dPose.q.GetAngle());
    }

    float halfLength(){
        return bf.halfLength;
    }

    float halfWidth(){
        return bf.halfWidth;
    }

    void setOrientation(float, float);




}; //sub action f


struct simResult{
    enum resultType {successful =0, crashed =1, safeForNow=2}; //successful means no collisions, finished means target reached, for later
    resultType resultCode= successful;
    Disturbance collision;
    bool valid = 0;
    b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    int step=0;


    simResult(){}

    simResult(resultType code): resultCode(code){
        valid =1;
    }

    simResult(resultType code, Disturbance obst): resultCode(code), collision(obst){
        valid =1;
    }
};

#endif