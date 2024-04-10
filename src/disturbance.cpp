#include "disturbance.h"


void Disturbance::setAngle(b2Transform t){ //angle to robot
        //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
        float angle;
        b2Vec2 thisToB;
        thisToB.x = getPosition().x-t.p.x;
        thisToB.y = getPosition().y - t.p.y;
        float cosA = (thisToB.x * cos(t.q.GetAngle())+ thisToB.y*sin(t.q.GetAngle()))/thisToB.Length();
        angleToRobot = acos(cosA);
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