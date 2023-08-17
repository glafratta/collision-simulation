#include "measurement.h"

// float EndedResult::errorSquared(){
//     return errorFloat*errorFloat;
// }

bool Measurement::operator<(Measurement & m2){
    bool r = false;
    if (isValid() && m2.isValid()){
        r= get()<m2.get();
    }
    else{
        r=true;
    }
    return r;
}

bool Measurement::operator<=(Measurement & m2){
    bool r = false;
    if (isValid() && m2.isValid()){
        r= get()<=m2.get();
    }
    else{
        r=true;
    }
    return r;
}

bool Measurement::operator>=(Measurement &m2){
    bool r = false;
    if (isValid() && m2.isValid()){
        r= get()>=m2.get();
    }
    else{
        r=true;
    }
    return r;
}

float Measurement::getError(Measurement m2){
    float result =0;
    if (m2.isValid() && this->isValid()){
        result= this->get()-m2.get();
    }
    return result;
}

float Measurement::getStandardError(Measurement m2){ 
    float result =0;
    if (m2.isValid()&& this->isValid()){
        float num = get()-m2.get();
        if (num ==0){
            return result;
        }
        float den = (m2.get()+get()); //normalise to the arithmetic mean, max value =2, then normalise again, max reusl =1
        if (den==0){
            return 1;
        }
        result = num/den;
    }
    return result;
}

float EndCriteria::getError(EndCriteria ec){ //not normalised
    float result =0;
    result = angle.getError(ec.angle) + distance.getError(ec.distance);
    return result;
}

float EndCriteria::getStandardError(Angle a, Distance d){ //standard error
    float result =0;
    result = weights[0]*angle.getStandardError(a)+weights[1]* distance.getStandardError(d); //max =2;
    return result/2; //return normalise
}

float EndCriteria::getStandardError(EndCriteria ec){ //standard error
    float result =0;
    //result = weights[0]*angle.getStandardError(ec.angle)+ weights[1]*distance.getStandardError(ec.distance); //max =4;
    result = getStandardError(ec.angle, ec.distance);
    return result;
}

float EndCriteria::getStandardError(Angle a, Distance d, Node n){
    float result =0;
    float outcomeError=0;
    if (n.filled){
        switch (n.outcome){
            case simResult::successful: break;
            case simResult::safeForNow: outcomeError+=1; break;
            case simResult::crashed: outcomeError+=2; break; //the max error is 6 (4 from angle and distance, 2 from outcome)
            default:break;
        }
    }
    result = getStandardError(a, d) + weights[2]*outcomeError;
    return result/3; //normalised to max value it can take
}

float SignedVectorLength(b2Vec2 v){
	float signedLength = v.Length();
	if (v.x <0){
        //printf("this task goes backwards: og length = %f\n", v.Length());
		signedLength = -signedLength;
	}
	return signedLength;
}

bool EndCriteria::hasEnd(){
    return angle.isValid() || distance.isValid();
}