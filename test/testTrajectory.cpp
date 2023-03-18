#include "Box2D/Box2D.h"
#include <vector>
#include <stdio.h>
#include <math.h>
#include "../src/primitive.cpp"

int main(){
    std::vector <b2Vec2> positions= {{0.4, 0.0}, {1.0, 0.0}, {0.1, 0.0}, {0.3, 0.02}, {0.3, -.02}, };
    //std::vector <Object> obstacles;
    int pNUmber =0;
    printf("baseline Primitive\n");
    Primitive dmp;
    b2World world({0.0f, 0.0f});
    dmp.willCollide(world, 0);
    printf("%s\n", dmp.planFile);
    printf("trajectory valid? %i\n", dmp.getAction().isValid());
    printf("obstacle valid? %i\n", dmp.obstacle.isValid());

    for (b2Vec2 p:positions){
        pNUmber++;
        b2World world({0.0f, 0.0f});
        Primitive::Object o(ObjectType::obstacle, p);
//        printf("obstacle valid: %i\n", o.isValid());
        Primitive dmp(o);
        Primitive dmp2(o, Primitive::Direction::LEFT);
        Primitive dmp3(o, Primitive::Direction::RIGHT);
        printf("%s\n", dmp.planFile);
        printf("obstacle at: %f, %f\n", o.getPosition().x, o.getPosition().y);
        printf("trajectory valid? %i\n", dmp.getAction().isValid());
        printf("obstacle in dmp valid? %i\n", dmp.obstacle.isValid());  
        printf("direction left = %i, direction right = %i\n", static_cast<int>(dmp2.getAction().getDirection()), static_cast<int>(dmp3.getAction().getDirection()));      
        Primitive::simResult result =dmp.willCollide(world, pNUmber);
        printf("\n\n");


    }

    printf("test atan(0) = %f\n", atan(0.0/.45));

}
