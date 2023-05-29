#ifndef AFFORDANCE_H
#define AFFORDANCE_H
#include <vector>
#include <stdio.h>
//#include "task.h"

// typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Affordance> Affordances;
// typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Primitives> Primitives;
// typedef oost::graph_traits<Affordances>::vertex_iterator AffordanceIndex;
// typedef oost::graph_traits<Primitives>::vertex_iterator MIndex;
typedef unsigned int AffordanceIndex; //was thinking of this being a character but doesn't have to be maybe enum is fine
//typedef unsigned int MIndex;
enum InnateAffordances {NONE, AVOID, PURSUE, ATTACK, EXPLORE}; //for ease of identification

class M{
public:
    float L=0, R=0, omega=0, linearSpeed=0;
    M(){}
    M(float l, float r){
        L=l;
        R=r;
        omega = (MAX_SPEED*(R-L)/BETWEEN_WHEELS); //instant velocity, determines angle increment in willcollide
        linearSpeed = MAX_SPEED*(L+R)/2;   
    }

};

class Affordance{
public:
    AffordanceIndex ID=-1;
    std::vector <M> options; //vector of primitives, low level affordance
    Affordance(){}

    Affordance(std::vector <M> vec, AffordanceIndex i =-1): 
    options(vec), 
    ID(i)
    {}

    void initialiseMs(std::vector <M> vec){
       options = vec;
    }

    void setID(AffordanceIndex i){
        ID =i;
    }
    //can map up or down in a hierarchy (not implemented) through pointers or maps
    
};




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
};

class Angle: public Measurement{
    public:
    Angle(float f)
    {   value =f;
        valid =1;}
};

class Distance: public Measurement{
    public:
    Distance(float f)
    {value =f;
    valid =1;}
};

struct EndCriteria{
    Angle angle;
    Distance distance;
};

//THESE MAPS REFLECT HOW DISTURBANCES ARE MAPPED TO AFFORDANCES


#endif