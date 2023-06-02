#ifndef SETTINGS
#define SETTINGS
#include "affordance.h"
//THE ROBOT STARTS WITH THESE FIXED SETTINGS, AND USER SETTINGS IS A WAY FOR THESE TO BE ADDED TO
// const M Straight(0.5, 0.5); //left, right
// const M Left(-0.5, 0.5);
// const M Right(0.5, -0.5);
// const M Back(-0.5, -0.5);
// const M Stop(0, 0);
//const std::vector <Direction> primitives{Straight, Left, Right, Back, Stop};
const Affordance None(std::vector <Direction>{DEFAULT}, 0);
const Affordance Avoid(std::vector<Direction>{LEFT, RIGHT, BACK, STOP},1);
const std::vector <Affordance> affordances{None, Avoid};


class UserSettings{
    std::vector <Direction> primitives = primitives;
    std::vector <Direction> affordances = affordances;
};


#endif