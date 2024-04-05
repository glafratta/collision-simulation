#ifndef MEASUREMENT_H
#define MEASUREMENT_H
#include "graphTools.h" 

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

    void setValid(bool b){
        valid = b;
    }

    bool operator<(Measurement &);

    bool operator<=(Measurement &);

    bool operator>=(Measurement &);

    float getError(Measurement);

    float getStandardError(Measurement, float); //relative standard error

};

class Angle: public Measurement{
    public:
    Angle(){}
    Angle(float f)
    {   value =round(f*1000)/1000;
        valid =1;}
};

class Distance: public Measurement{
    public:
    Distance(){}
    Distance(float f)
    {value = round(f*1000)/1000;
    valid =1;}
};

struct EndCriteria{
    Angle angle;
    Distance distance;    //max distance, ideal
    float getError(EndCriteria); //expresses magnitude of error, not normalised
    float getStandardError(EndCriteria);
    float getStandardError(Angle, Distance);
    float getStandardError(Angle, Distance, State);
    std::vector <float> weights = {1, 1, 1}; //0:angle, 1: distance, 2: outcome (if node)
    bool hasEnd();

};


struct EndedResult{
	bool ended=0;
	float estimatedCost=0; //dot product of end criteria
    float cost=0;

    EndedResult() = default;

//    float evaluationFunction(std::vector <vertexDescriptor> plan={}, vertexDescriptor v=TransitionSystem::null_vertex());

   // float errorSquared();
};


float SignedVectorLength(b2Vec2);


#endif