#pragma once
#include <vector>
#include <stdio.h>
#include "task.h"

class Affordance{
    typedef A;
    std::vector <A> options; //can be primitives or affordances
    //can map up or down in a hierarchy (not implemented) through pointers or maps
    
};

struct Measurement{
protected:
    bool valid =0;
    float value;
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

struct Angle: protected Measurement{
    Angle(float f):
    value(f)
    {valid =1}
};

struct Distance: protected Measurement{
    Distance(float f):
    value(f)
    {valid =1}
};

struct EndCriteria{
    Angle angle;
    Distance distance;
};

//THESE MAPS REFLECT HOW DISTURBANCES ARE MAPPED TO AFFORDANCES


