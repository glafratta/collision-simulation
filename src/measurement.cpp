#include "measurement.h"

bool operator<(M & m1, M & m2){
    bool r = false;
    if (m1.isValid() && m2.isValid()){
        r= m1.get()<m2.get();
    }
    else{
        r=true;
    }
    return r;
}

bool operator>=(M & m1, M& m2){
    bool r = false;
    if (m1.isValid() && m2.isValid()){
        r= m1.get()>=m2.get();
    }
    else{
        r=true;
    }
    return r;
}

float SignedVectorLength(b2Vec2 v){
	float signedLength = v.Length();
	if (v.x <0){
        //printf("this task goes backwards: og length = %f\n", v.Length());
		signedLength = -signedLength;
	}
	return signedLength;
}