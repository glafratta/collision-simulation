#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void makeLine(char * line, float angle, float speed=0.1){ //final angle
    //angular velocity is angle/10s
    float x,y;
    float hz=60.0f;
    char name[50];
    sprintf(name, "%s.txt", line);
    FILE * file;
    file=fopen(name, "w+");
    for (int i=0;i<hz*100;i++){
        y += sin(angle/hz)*speed/hz;
        x +=cos(angle/hz)*speed/hz;
        //pos[1] += (speed/hz) * cos(a);
        fprintf(file, "%.2f\t%.2f\n", x, y);
    }
    fclose(file);
}

int main(){
//straight line
makeLine("straight", 0);

//sinusoid
makeLine("sinusoid", 2*M_PI);

   
}