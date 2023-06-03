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