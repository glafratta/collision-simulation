#include "disturbance.h"

BodyFeatures BodyFeatures::operator=(const BodyFeatures& bf){
    pose=bf.pose;
    halfLength=bf.halfLength;
    halfWidth=bf.halfWidth;
    shift=bf.shift;
    bodyType=bf.bodyType;
    shape=bf.shape;
    attention=bf.attention;
}


bool BodyFeatures::match(const BodyFeatures& bf){
    bool match_x=(pose.p.x-bf.pose.p.x)<D_POSE_MARGIN;
    bool match_y=(pose.p.y-bf.pose.p.y)<D_POSE_MARGIN;
    bool match_w=(halfWidth-bf.halfWidth)<D_DIMENSIONS_MARGIN;
    bool match_h=(halfLength-bf.halfLength)<D_DIMENSIONS_MARGIN;
    return match_x && match_y && match_w && match_h;
}


float Disturbance::getAngle(b2Transform t){ //gets the angle of an Disturbance wrt to another Disturbance (robot)
        //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
        float angle;
        b2Vec2 thisToB;
        thisToB.x = getPosition().x-t.p.x;
        thisToB.y = getPosition().y - t.p.y;
        float cosA = (thisToB.x * cos(t.q.GetAngle())+ thisToB.y*sin(t.q.GetAngle()))/thisToB.Length();
        angle = acos(cosA);
        return angle;
    }

void Disturbance::setOrientation(float s, float c){
    b2Rot og;
    og.s=s;
    og.c=c; 
    if (rotation_valid){
    b2Rot sup, comp, ver; //find most likely angle
        sup.s=-s;
        sup.c=c;
        comp.c=-c;
        comp.s=s;
        ver.s=-s;
        ver.c=-c;
        std::vector <b2Rot> rots={sup, comp, ver};
        for (b2Rot r:rots){
            if (fabs(r.GetAngle()-bf.pose.q.GetAngle())<fabs(og.GetAngle()-bf.pose.q.GetAngle())){
                og=r;
            }
        }
    }
    bf.pose.q.s=og.s;
    bf.pose.q.c=og.c;
    rotation_valid=1;
}