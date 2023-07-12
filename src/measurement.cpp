#include "measurement.h"

bool Measurement::operator<(const Measurement& m2){
    bool r = false;
    if (this->isValid() && m2.isValid()){
        r= this->get()<m2.get();
    }
    else{
        r=true;
    }
    return r;
}

bool Measurement::operator>=(const Measurement& m2){
    bool r = false;
    if (this->isValid() && m2.isValid()){
        r= this->get()>=m2.get();
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
        float num = m2.get()-this->get();
        if (num ==0){
            return result;
        }
        float den = (m2.get()+this->get())/2; //normalise to the arithmetic mean, max value =2
        if (den==0){
            return 2;
        }
        result = num/den;
    }
    return result;
}

float EndCriteria::getError(EndCriteria ec){
    float result =0;
    result = this->angle.getError(ec.angle) + this->distance.getError(ec.distance);
    return result;
}

float EndCriteria::getStandardError(EndCriteria ec){ //standard error
    float result =0;
    float normAngle= this->angle;
    if (this->angle==0){
        normAngle=M_PI *2;
    }
    result = this->angle.getStandardError(ec.angle)+ this->distance.getStandardError(ec.distance); //max =4;
    return result;
}

float EndCriteria::getStandardError(Angle a, Distance d){ //standard error
    float result =0;
    float normAngle= this->angle;
    result = this->angle.getStandardError(a)+ this->distance.getStandardError(d); //max =4;
    return result;
}

float SignedVectorLength(b2Vec2 v){
	float signedLength = v.Length();
	if (v.x <0){
        //printf("this task goes backwards: og length = %f\n", v.Length());
		signedLength = -signedLength;
	}
	return signedLength;
}

bool hasEnd(){
    return angle.isValid() || distance.isValid();
}