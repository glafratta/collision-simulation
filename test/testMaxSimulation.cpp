#include "Box2D/Box2D.h"
#include <vector>
#include <stdio.h>
#include <math.h>
#include "../src/configurator.h"


int main(){
    //around 100 bodies
    std::vector <Point> current;
    float x=0;
    float y=0.2;
    for (int i=0; i<50; i++){
        current.push_back(Point(x, y));
        current.push_back(Point(x, -y));
        x+=0.005;
    }

    printf("current size = %i\n", current.size());

    
    int initD = 3;
    float initHz = 50;
    FILE * file = fopen("maxSimDurResult.txt", "w");
    while (true){ 
	    auto t0 =std::chrono::high_resolution_clock::now();
        State state;
        state.setHz(initHz);
        state.setSimDuration(initD);
        fprintf(file, "%f\t%f", initHz, initD);
        state.setSimDuration(3);
        Configurator c(state);
        c.NewScan(current);  
	    auto t1 =std::chrono::high_resolution_clock::now();
	    std::chrono::duration<float, std::milli>diff= t1-t0; //in seconds
        float time=float(diff.count())/1000; //express in seconds
        fprintf(file, "\t%f\n", time);
        printf("%f\t%i\t%f\n", initHz, initD, time);      
        if (time >=.15){
            if (initHz >10){
                initHz-=10.0f;
            }
            else {
                return 0;
            }
        }
        else{
            initD+=10;
        }
        printf("ended 1 loop\n");
    }
    fclose(file);

}
