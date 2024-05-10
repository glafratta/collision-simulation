#include "disturbance.h"


// void Disturbance::setAngle(b2Transform t){ //angle to robot
//         //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
//         float angle;
//         b2Vec2 thisToB;
//         thisToB.x = getPosition().x-t.p.x;
//         thisToB.y = getPosition().y - t.p.y;
//         float cosA = (thisToB.x * cos(t.q.GetAngle())+ thisToB.y*sin(t.q.GetAngle()))/thisToB.Length();
//         angleToRobot = acos(cosA);
//     }

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