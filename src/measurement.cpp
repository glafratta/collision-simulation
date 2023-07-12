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


float EndCriteria::getError(EndCriteria ec){
    float result =0;
    result = this->angle.getDifferenceAsFloat(ec.angle) + this->distance.getDifferenceAsFloat(ec.distance);
    return result;
}

float EndCriteria::getStandardError(EndCriteria ec){ //standard error
    float result =0;
    float normAngle= this->angle;
    if (this->angle==0){
        normAngle=M_PI *2;
    }
    result = this->angle.getDifferenceAsFloat(ec.angle)/norm + this->distance.getDifferenceAsFloat(ec.distance)/this->distance.get(); //max =2;
    return result/2;
}

float SignedVectorLength(b2Vec2 v){
	float signedLength = v.Length();
	if (v.x <0){
        //printf("this task goes backwards: og length = %f\n", v.Length());
		signedLength = -signedLength;
	}
	return signedLength;
}