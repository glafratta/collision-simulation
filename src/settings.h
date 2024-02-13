#ifndef SETTINGS
#define SETTINGS
#include "affordance.h"
const Affordance None(std::vector <Direction>{DEFAULT}, 0);
const Affordance Avoid(std::vector<Direction>{LEFT, RIGHT, BACK},1);
const Affordance Pursue(std::vector <Direction> {DEFAULT, LEFT, RIGHT},2);
const std::vector <Affordance> affordances{None, Avoid, Pursue};

#endif