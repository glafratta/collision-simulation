#include "../src/configurator.h"
#include <stdio.h>
#include <stdlib.h>
#include<sys/stat.h>

int main(){
    //generate a grid of points
    char dirName[256];
    sprintf(dirName, "testEnvRepresentation") ;
    if (!opendir(dirName)){
        mkdir(dirName, 0777);
    }
    else{
        printf("directory opened \n");
    }

    Configurator c;
    char filename[256], straight0[256], turn1_2_0[256], straight1_2_pi4[256];
    sprintf(filename, "%s/pointGrid.txt", dirName);
    sprintf(straight0, "%s/straight0.txt", dirName);
    sprintf(turn1_2_0, "%s/turn1_2_0.txt", dirName);
    sprintf(straight1_2_pi4, "%s/straight1_2_pi4.txt", dirName);
    FILE * file = fopen(filename, "r");
    //if (file ==NULL){
        file = fopen(filename, "w+");
        b2Vec2 start; //start from bottom left
        start.x=-0.5;
        while (start.x <=0.51){
            start.y = -0.5;
            while (start.y <=0.5){
                c.current.push_back(start);
                start.y +=0.05;
                printf("%f\t%f\n", start.x, start.y);
                fprintf(file, "%.2f\t%.2f\n", start.x, start.y);
            }
        start.x +=0.05;
        //printf("%.2f\t%.2f\n", start.x, start.y);
        }

    //}
    //else{
        std::ifstream f(filename);
        b2Vec2 v;
        while (f>>v.x>>v.y){
            c.current.push_back(v);        
        }
    //}
    fclose(file);
    //write to file the bodies created s
    b2World world({0.0, 0.0}), world2({0.0, 0.0}), world3({0.0, 0.0}), world4({0.0f, 0.0f}),  world5({0.0f, 0.0f}),  world6({0.0f, 0.0f});
    c.constructWorldRepresentation(world, State::Direction::NONE, b2Transform(b2Vec2(0.0f, 0.0f),b2Rot(0)));
    FILE * s0 = fopen(straight0, "w");
    printf("world 1 bodies = %i\n", world.GetBodyCount());
    for (b2Body * b = world.GetBodyList(); b!=NULL; b = b->GetNext()){
        fprintf(s0, "%.2f\t%.2f\n", b->GetPosition().x, b->GetPosition().y);
    }
    fclose(s0);
    c.constructWorldRepresentation(world2, State::Direction::LEFT, b2Transform(b2Vec2(0.0f, 0.0f),b2Rot(0)));
    FILE * t = fopen(turn1_2_0, "w");
    printf("world2 1 bodies = %i\n", world2.GetBodyCount());
    for (b2Body * b = world2.GetBodyList(); b!=NULL; b = b->GetNext()){
        fprintf(t, "%.2f\t%.2f\n", b->GetPosition().x, b->GetPosition().y);
    }
    fclose(t);
    c.constructWorldRepresentation(world3, State::Direction::NONE, b2Transform(b2Vec2(0.0f, 0.0f),b2Rot(M_PI_4)));
    printf("world3 1 bodies = %i\n", world3.GetBodyCount());
    FILE *  pi = fopen(straight1_2_pi4, "w");
    for (b2Body * b = world3.GetBodyList(); b!=NULL; b = b->GetNext()){
        fprintf(pi, "%.2f\t%.2f\n", b->GetPosition().x, b->GetPosition().y);
    }
    fclose(pi);

    c.constructWorldRepresentation(world4, State::Direction::NONE, b2Transform(b2Vec2(0.0f, 0.0f),b2Rot(-M_PI_4)));
    printf("world4 1 bodies = %i\n", world4.GetBodyCount());
    FILE *  pi4 = fopen("testEnvRepresentation/straight-pi4.txt", "w");
    for (b2Body * b = world4.GetBodyList(); b!=NULL; b = b->GetNext()){
        fprintf(pi4, "%.2f\t%.2f\n", b->GetPosition().x, b->GetPosition().y);
    }
    fclose(pi4);

    c.constructWorldRepresentation(world5, State::Direction::NONE, b2Transform(b2Vec2(0.0f, 0.0f),b2Rot(-M_PI/6))); //-30 deg
    printf("world5 1 bodies = %i\n", world5.GetBodyCount());
    FILE *  pi6 = fopen("testEnvRepresentation/straight-pi6.txt", "w");
    for (b2Body * b = world5.GetBodyList(); b!=NULL; b = b->GetNext()){
        fprintf(pi6, "%.2f\t%.2f\n", b->GetPosition().x, b->GetPosition().y);
    }
    fclose(pi6);
    c.constructWorldRepresentation(world6, State::Direction::NONE, b2Transform(b2Vec2(0.0f, 0.0f),b2Rot(M_PI)));
    printf("world6 1 bodies = %i\n", world6.GetBodyCount());
    FILE *  pi1 = fopen("testEnvRepresentation/straight_pi.txt", "w");
    for (b2Body * b = world6.GetBodyList(); b!=NULL; b = b->GetNext()){
        fprintf(pi1, "%.2f\t%.2f\n", b->GetPosition().x, b->GetPosition().y);
    }
    fclose(pi1);
}
