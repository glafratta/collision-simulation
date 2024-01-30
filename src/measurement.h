#ifndef MEASUREMENT_H
#define MEASUREMENT_H
#include "general.h" 

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
   // float getStandardError(Node);
    bool hasEnd();

};


struct EndedResult{
	bool ended=0;
	float estimatedCost=0; //dot product of end criteria

    EndedResult() = default;

   // float errorSquared();
};

//typedef Measurement Mst;
// bool operator<(Meas &, Meas &);

// bool operator>=(Meas &, Meas &);


float SignedVectorLength(b2Vec2);


// template <class PT>
// bool isInRadius(PT point1, PT point2, float radius = 0.05){ //check if this point is within a certain radius from another given point
// 	if (point2.x <= point1.x+radius && point2.x >=point1.x-radius && point1.y <= point1.y+radius && point2.y >=point1.y-radius){
// 		return true;
// 	}
// 	else{
// 		return false;
// 	}
// }

#endif