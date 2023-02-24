#include "Box2D/Box2D.h"
#include <vector>
#include <stdio.h>
#include <math.h>
#include "../src/state.cpp"

int main(){
    std::vector <b2Vec2> positions= {{0.4, 0.0}, {1.0, 0.0}, {0.1, 0.0}, {0.3, 0.02}, {0.3, -.02}, };
    //std::vector <Object> obstacles;
    int pNUmber =0;
    printf("baseline state\n");
    State state;
    b2World world({0.0f, 0.0f});
    state.willCollide(world, 0);
    printf("%s\n", state.planFile);
    printf("trajectory valid? %i\n", state.getAction().isValid());
    printf("obstacle valid? %i\n", state.obstacle.isValid());

    for (b2Vec2 p:positions){
        pNUmber++;
        b2World world({0.0f, 0.0f});
        State::Object o(ObjectType::obstacle, p);
//        printf("obstacle valid: %i\n", o.isValid());
        State state(o);
        State state2(o, State::Direction::LEFT);
        State state3(o, State::Direction::RIGHT);
        printf("%s\n", state.planFile);
        printf("obstacle at: %f, %f\n", o.getPosition().x, o.getPosition().y);
        printf("trajectory valid? %i\n", state.getAction().isValid());
        printf("obstacle in state valid? %i\n", state.obstacle.isValid());  
        printf("direction left = %i, direction right = %i\n", static_cast<int>(state2.getAction().getDirection()), static_cast<int>(state3.getAction().getDirection()));      
        State::simResult result =state.willCollide(world, pNUmber);
        printf("\n\n");


    }

    printf("test atan(0) = %f\n", atan(0.0/.45));

}
