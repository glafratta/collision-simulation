#ifndef AFFORDANCE_H
#define AFFORDANCE_H
#include <vector>
#include <stdio.h>

typedef unsigned int AffordanceIndex; //was thinking of this being a character but doesn't have to be maybe enum is fine
enum InnateAffordances {NONE, AVOID, PURSUE, ATTACK, EXPLORE}; //for ease of identification
enum Direction{LEFT, RIGHT, DEFAULT, BACK, STOP};

class Affordance{
public:
    AffordanceIndex ID=-1;
    std::set <Direction> options; //vector of primitives, low level affordance
    Affordance(){}

    Affordance(std::set <Direction> vec, AffordanceIndex i =-1):ID(i){
        for (Direction d:vec){
            options.insert(d);
        }
    }

    void initialiseMs(std::set<Direction> vec){
        for (Direction d:vec){
            options.insert(d);
        }
    }

    void setID(AffordanceIndex i){
        ID =i;
    }
    //can map up or down in a hierarchy (not implemented) through pointers or maps
    
};


#endif