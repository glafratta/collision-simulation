#include "Box2D/Box2D.h"
#include "../src/configurator.h"
#include <thread>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip> 
#include <sstream> //for writing string into file, for checking, std::ostringstream
#include <ncurses.h>
#include <ctime>
#include <dirent.h>
#include <filesystem>
#include "CppTimer.h"


// b2Vec2 estimateDisplacementFromWheels(float max, float L, float R, float d, float t=0.2){
//     b2Vec2 vel;
//     	//find angular velocity
// 	float angVel = max *(L-R)/d; //rad/s
// 	//find absolute speed
// 	float speed = max * (L+R)/2; //distance covered
// 	//find velocity vector: this is per second so no need to change this; but it is the instantaneous velocity
// 	//vel= b2Vec2(speed*sin(angVel), speed*(-cos(angVel)));
//     //from the derivation
//     vel.x = (d/2)* sin(d*t/(L-R))
// 	return vel;
// }
float getHypothenuse(float x, float y){
    float d = sqrt(x*x + y*y);
    return d;
}


class DataInterface {
public:
int mapCount = 0;
FILE * velocity;
char fileName[256];
//for debugging
Configurator * c;
char * folder;
bool fileExists=1;
float m;
char filePath[256];

    DataInterface(Configurator * _c, float _m){ //reads data from folder
		c = _c;
        m=_m;
        sprintf(fileName, "/tmp/velocity%.1fm.txt", m);
        	velocity = fopen(fileName, "w");
            fclose(velocity);
		// std::filesystem::path folderPath(_folder);
		// if (exists(folderPath)){
		// 	if (is_directory(folderPath)){
		// 		folder = _folder;
        //         c->folder = _folder;
		// 	}
		// 	else{
		// 		printf("not a directory");
		// 	}
		// }
		// else{
		// 	printf("%s doesn't exist", _folder);
		// }


	}

	void newScanAvail(){ //uncomment sections to write x and y to files	
        c->timeElapsed = .2;
        mapCount++;
        b2Vec2 coord;
        //float totalTime =0;
		//char filePath[256];
        char folderName[256];
        sprintf(folderName,"%s", c->folder);
		sprintf(filePath, "%s%s%04d.dat", folderName, c->getReadMap(),mapCount);
        //currentFile = filePath;
        printf("%s\n", filePath);
        std::ifstream map(filePath);
        if (!map){
            fileExists=0;
            printf("file doesn't exist!\n");
            return;
        }
        std::vector <cv::Point2f> current;
        printf("only including points closer than %f meters\n", m);
        int ignoredCount = 0;
        int counted =0;
		while (map>>coord.x>>coord.y){
			if (coord.Length()<=m){
                current.push_back(cv::Point2f(coord.x, coord.y));
                //printf("%f\t%f\n", coord.x, coord.y);
                counted++;
            }
		    
            else{
                ignoredCount++;
            }
	    }
        printf("%i ignored, %i counted\n", ignoredCount, counted);
        map.close();
        c->addIteration();
        b2Vec2 vel = c->GetRealVelocity(current).vector;
        velocity = fopen(fileName, "a");
        fprintf(velocity, "%f\t%f\t%f\n", vel.x, vel.y, vel.Length());
        fclose(velocity);
		
    }
	




};


// class TimerDI: public CppTimer{
//     DataInterface di;
//     public:
//     TimerDI(DataInterface & _di): di(_di){}
//     void timerEvent(){
//         di.newScanAvail();
//     }
// };

//END THREAD DEBUGGING

int main(int argc, char** argv) {
    //DATA INTERCFACE
    FILE * errorLogAffine = fopen("errorLogAffine.txt", "w+");
    fprintf(errorLogAffine, "Meters\tError\n");
    for (float m=12.0; m>0;m-=.5){
        printf("m= %.1f\n", m);
        Configurator conf;
        conf.setReadMap("map");
        conf.setFolder(argv[1]);
        int countFiles=1;
        char fileName[256];
        sprintf(fileName, "%smap%04i.dat", argv[1], countFiles);
        std::ifstream file(fileName);
        while (file){
            countFiles++;
            sprintf(fileName, "%smap%04i.dat", argv[1], countFiles);
            //printf("filename = %s\n", fileName);
            file = std::ifstream(fileName);
        }
        DataInterface dataInterface(&conf, m);
        DataInterface * diPtr;
        diPtr = &dataInterface;
        for  (int i=0; i<countFiles; i++){
            dataInterface.newScanAvail();
        }
        // TimerDI lidar(dataInterface);
        // lidar.startms(200);
        // std::this_thread::sleep_for(std::chrono::milliseconds(200*countFiles));
        fprintf(errorLogAffine, "%f\t%f\n", m, conf.affineTransError);
        // lidar.stop();
    }
    fclose(errorLogAffine);


    // FINISH BENCHMARKING
    // auto end = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    // printf("scanned in %f seconds\n", elapsed.count()* 1e-9);




}


