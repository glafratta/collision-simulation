#include "measurement.h"

bool operator<(M const & m1, M const & m2){
    bool r = false;
    if (m1.isValid() && m2.isValid()){
        r= m1.get()<m2.get();
    }
    else{
        r=true;
    }
    return r;
}

bool operator>=(M const & m1, M const & m2){
    bool r = false;
    if (m1.isValid() && m2.isValid()){
        r= m1.get()>=m2.get();
    }
    else{
        r=true;
    }
    return r;
}