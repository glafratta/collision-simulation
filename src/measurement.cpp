#include "measurement.h"

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
    float normValue = this->get();
    if (m2.isValid()&& this->isValid()){
        float num = m2.get()-get();
        if (num ==0){
            return result;
        }
        float den = (m2.get()+get())/2; //normalise to the arithmetic mean, max value =2
        if (den==0){
            return 2;
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

float EndCriteria::getStandardError(EndCriteria ec){ //standard error
    float result =0;
    result = angle.getStandardError(ec.angle)+ distance.getStandardError(ec.distance); //max =4;
    return result;
}

float EndCriteria::getStandardError(Angle a, Distance d){ //standard error
    float result =0;
    result = angle.getStandardError(a)+ distance.getStandardError(d); //max =4;
    return result;
}

// float EndCriteria::getStandardError(Node){
//     float result =0;


//     return result;
// }

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