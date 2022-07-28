
#include <iostream>
#include <thread>
#include "alphabot.h"
#include <unistd.h>
#include "a1lidarrpi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string> 
#include <iomanip>
#include <sstream> //for writing string into file, for checking, std::ostringstream



//the environment is sampled by the LIDAR every .2 seconds

class DataInterface : public A1Lidar::DataInterface{
char[50] folderPath;
public: 
DataInterface(char path[50]): folderPath(path){}
int iteration =0;

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
	
		////WRITE TO FILE
		char name[50];
		sprintf(name, "/%s/map%04f.dat");
		FILE * file = fopen(name, "w+");
		for (A1LidarData &data:data){
			if (data.valid){ 
				fprintf(file, "%2f\t%2f\n",data.x, data.y);
			} //to here

		}
		file.close();

	}


};





int main(int, char**) {

	//TODAYS DATE AND TIME

	time_t now =time(0);
    tm *ltm = localtime(&now);
    int y,m,d;
    y=ltm->tm_year-100;
    m = ltm->tm_mon +1;
    d=ltm->tm_mday;
    
    //PICK DIRECTORY NAME
    char dirName[256];
    int copynumber=1;
    while (true){
        DIR *dir; 
        char tmp[256];
        sprintf(tmp, "maps%02i-%02i-%02i_%i",d, m, y, copynumber);
        dir = opendir(tmp);
        if (dir ==NULL){
            sprintf(dirName, "%s", tmp);
            std::filesystem::create_directory(dirName);
            break;
        } //if the directory DOESN'T return a void pointer (= exists already)
        else if (dir!=NULL){
            copynumber++;                                                           //increase the number at the end of the name
            closedir(dir);
        }

    }



	AlphaBot motors; //records data for 2 seconds
	A1Lidar lidar;
	DataInterface dataInterface;
	lidar.registerInterface(&dataInterface);
	lidar.start();
	motors.start();
	motors.setRightWheelSpeed(0.0); //possible solution for the gpioinitialise is macros?
	motors.setLeftWheelSpeed(0.0);
	motors.setRightWheelSpeed(0.5f); //possible solution for the gpioinitialise is macros?
	motors.setLeftWheelSpeed(0.5f);
	do {}
	while (!getchar());
	motors.stop();
	lidar.stop();


	//make lidar and alphabod type classes pointers, can create threads for processes that start and stop lidar and robot separately
	



	
}
	
	
