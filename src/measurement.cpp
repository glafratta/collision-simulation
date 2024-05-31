#include "measurement.h"

bool Measurement::operator<(Measurement & m2){
    bool r = false;
    if (isValid() & m2.isValid()){
        r= get()<m2.get();
    }
    else{
        r=true;
    }
    return r;
}

bool Measurement::operator<=(Measurement & m2){
    bool r = false;
    if (isValid() & m2.isValid()){
        r= get()<=m2.get();
    }
    else{
        r=true;
    }
    return r;
}

bool Measurement::operator>=(Measurement &m2){
    bool r = false;
    if (isValid() & m2.isValid()){
        r= get()>=m2.get();
    }
    else{
        r=true;
    }
    return r;
}

float Measurement::getError(Measurement m2){
    float result =0;
    if (m2.isValid() & this->isValid()){
        result= this->get()-m2.get();
    }
    return result;
}

float Measurement::getStandardError(Measurement m2, float max){ 
    float result =0;
    if (m2.isValid()& this->isValid()){
        float num = get()-m2.get();
        if (num ==0){
            return result;
        }
        result = num/max;
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
    result = weights[0]*fabs(angle.getStandardError(a, MAX_ANGLE_ERROR))+weights[1]*fabs(distance.getStandardError(d, MAX_DISTANCE_ERROR)); //max =2;
    return result/2; //return normalise
}

float EndCriteria::getStandardError(EndCriteria ec){ //standard error
    float result =0;
    result = getStandardError(ec.angle, ec.distance);
    return result;
}

float EndCriteria::getStandardError(Angle a, Distance d, State n){
    float result =0, coefficient=0.3;
    if (n.filled & n.outcome== simResult::crashed){
        coefficient=1;
    }
    result = getStandardError(a, d); //+ weights[2]*outcomeError;
    return result; //normalised to max value it can take
}

float SignedVectorLength(b2Vec2 v){
	float signedLength = v.Length();
	if (v.x <0){
		signedLength = -signedLength;
	}
	return signedLength;
}

bool EndCriteria::hasEnd(){
    return angle.isValid() || distance.isValid();
}

// float EndedResult::evaluationFunction(std::vector <vertexDescriptor> plan, vertexDescriptor v){ //h(n) = error, cost is the n of D
// 	float planPriority=0.0;
//     if (!plan.empty() & u_long(v)>=0){
//         planPriority=1.0;
//     } 
//     return abs(estimatedCost) +abs(cost)+planPriority;
// 	}