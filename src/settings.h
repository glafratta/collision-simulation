#ifndef SETTINGS
#define SETTINGS
#include "affordance.h"
const Affordance None(std::vector <Direction>{DEFAULT}, 0);
const Affordance Avoid(std::vector<Direction>{LEFT, RIGHT, BACK},1);
const std::vector <Affordance> affordances{None, Avoid};

class Knowledge{
    public:
    std::vector <Direction> primitives = primitives;
    std::vector <Direction> affordances = affordances;
};


#endif