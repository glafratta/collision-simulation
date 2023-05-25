#include "affordances.h"
//ID = position in primitives of M
Task::M Straight(0.5, 0.5); //left, right
Task::M Left(-.5, 0.5);
Task::M Right(0.5, -.5);
Task::M Back(-.5, -.5);
Task::M Stop(0, 0);
std::vector <Task::M> primitives = {Straight, Left, Right, Back, Stop};

//all the affordances the system knows to start with
Affordance Default;
Default.options = {Straight};
Affordance Avoid;
Avoid.options = {Left, Right, Back, Stop};
std::vector <Affordance> affordances = {Default, Avoid};
