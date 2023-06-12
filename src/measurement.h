#ifndef MEASUREMENT_H
#define MEASUREMENT_H
#include "robot.h"

class Measurement{
protected:
    bool valid =0;
    float value=0;
public:
    Measurement(){}

    bool isValid(){
        return valid;
    }

    float get(){
        return value;
    }

    void set(float f){
        value =f;
    }
};

class Angle: public Measurement{
    public:
    Angle(){}
    Angle(float f)
    {   value =round(f*100)/100;
        valid =1;}
};

class Distance: public Measurement{
    public:
    Distance(){}
    Distance(float f)
    {value = round(SAFE_DISTANCE +f*100)/100;
    valid =1;}
};


struct EndCriteria{
    Angle angle;
    Distance distance;
};

typedef Measurement M;
bool operator<(M &, M &);

bool operator>=(M &, M &);

#endif