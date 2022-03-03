#include "../rplidar_rpi/a1lidarrpi.h"
#include "environment.h"
#include <stdio.h>
#include <stdlib.h>
#include <string> 
#include <iomanip>
#include <sstream> //for writing string into file, for checking, std::ostringstream
#include <iostream>

//TO DO: benchmark

class DataInterface : public A1Lidar::DataInterface{
public: 
    int numberOfScans=0; //for testing purposes
    Box2DEnv * box2d;
    

    void setBox2D(Box2DEnv * _box2d){
        box2d = _box2d;
    }


	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ 
        numberOfScans++;
        int obstacleIndexCount = 0;
        box2d->createMap();
        Frame * frame = box2d->frames->at(box2d->frames->size()-1);
        // std::stringstream tmp;
        // tmp << "map" << std::setw(4) << std::setfill('0') << numberOfScans << ".dat";
        // const char * filename = tmp.str().c_str();
        // std::cout<<filename<<std::endl;
        //FILE * file =fopen(filename, "w+"); //uncomment
        float x, y;
        int * row;
        int * column;
		for (A1LidarData &data:data){
			if (data.valid){ 
                x= approximate(data.x);
                y= approximate(data.y);
                //std::ostringstream stream;
                Obstacle* obstacle = new Obstacle(*(box2d->world), x, y); //doesn't check obstacles for duplicates
                //obstacle->body->SetAwake(true);
			    frame->obstacles->push_back(obstacle);
                transformForMatrix(x, y, row, column);
                *((frame->matrix.ptr(*row, *column))= 1; 
                //stream <<obstacle->body->GetPosition().x<<"\t"<<obstacle->body->GetPosition().y<<"\n";
                //std::cout<<obstacle->body->GetPosition().x<<"\t"<<obstacle->body->GetPosition().y<<"\n";
                //const char * line = stream.str().c_str();
                 //fputs(line, file);
			}
		}
        //fclose(file);
        box2d->simulate(); //simulates collision for 5 seconds
        std::cout << "velocity estimated to be "<<box2d->realVelocity.x<<", "<<box2d->realVelocity.y<<"\n";

	}


	float approximate(float a){ //more efficient with pointers?? (float a, float * ptr){assign value of a to *ptr}
		a = round(a*10)/10;
		if (a<0){
			a-=0.05;
		}
		else if (a>=0){
			a+= 0.05;
		}
        return a;
	}
    
    void transformForMatrix(float x, float y, float * i, float * j){ //transform cartesian coordinates into matrix coordinates
        //matrix is 240 bins of .1m, lidar data is in meters
        x = (int)x*10; // transform x and y in decimeters and rounds down
        y = (int)y*10;
        *i= 240/2-y; // x index: columns/2-y
        *j= 240/2+x; //y index: columns/2+x

    }
};

