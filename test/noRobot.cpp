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

class DataInterface {
public:
int iteration = 0;
Configurator * box2d;
char * folder;
    DataInterface(Configurator * _box2d, char * _folder){
		box2d = _box2d;
		std::filesystem::path folderPath(_folder);
		if (exists(folderPath)){
			if (is_directory(folderPath)){
				folder = _folder;
                box2d->folder = _folder;
			}
			else{
				printf("not a directory");
			}
		}
		else{
			printf("%s doesn't exist", _folder);
		}


	}

	void newScanAvail(){ //uncomment sections to write x and y to files		
        iteration++;
		char filePath[256];
		sprintf(filePath, "%s/transmap%04d.dat", folder, iteration);
        printf("%s\t", filePath);
		std::ifstream file(filePath);

        float x, y;
		//box2d->previous = box2d->current;
		box2d->current.clear();
		while (file>>x>>y){
			box2d->current.push_back(cv::Point2f(x, y));

			
		}
		file.close();
       // printf("current: %i, previous: %i\n", box2d->current.size(), box2d->previous.size());
        box2d->NewScan();

		
		

	}




};

class StepCallback{
    float L,R;
    Configurator * box2d;
public:
    StepCallback(Configurator * c): box2d(c){}
    void step(){
        box2d->controller();
        L=box2d->leftWheelSpeed;
        R= box2d->rightWheelSpeed;
        printf("R= %f, L = %f\n,", R, L);

    }
};

void printVectorToFile(std::vector<cv::Point2f> _v, int iteration, bool isCurrent, char dirName[256]){
    char type[15];
    if (isCurrent==true){
        sprintf(type,"current");
    }
    else if (isCurrent ==false){
        sprintf(type,"previous");
    }
    char name[256];
    sprintf(name,"%s/%s%04d.txt", dirName, type, iteration);
    printf("creating file at: %s\n", name);
    FILE * file = fopen(name, "w+");
    for (int i=0; i<_v.size(); i++){
        fprintf(file,"cv::Point(%f, %f)\n", _v[i].x, _v[i].y);


    }
    fclose(file);
}


// Box2DPlanningInterface::Box2DPlanningInterface(){
//     actionsAvailable = {GO, LEFT, RIGHT};
// }

// Box2DPlanningInterface::actionToPlan(Action action){
//     switch (action)
// }


int main(int argc, char** argv) {
    //BENCHMARK
//	auto begin = std::chrono::high_resolution_clock::now();


    //HOW MANY FILES IN DIRECTORY
    DIR *dp;
    int i = 0;
    struct dirent *ep;     
    dp = opendir(argv[1]);

    if (dp != NULL)
    {
        while (ep = readdir(dp)){
            i++;
        }
        closedir(dp);
    }
    else{
        printf("Couldn't open the directory%s", argv[1]);
    }


    //CREATE A FOLDER TO STORE THE RESULTS FOR THIS RUN

    // //TODAYS DATE AND TIME
    // time_t now =time(0);
    // tm *ltm = localtime(&now);
    // int y,m,d;
    // y=ltm->tm_year-100;
    // m = ltm->tm_mon +1;
    // d=ltm->tm_mday;
    
    // //PICK DIRECTORY NAME
    // char dirName[256];
    // int copynumber=1;
    // while (true){
    //     DIR *dir; 
    //     char tmp[256];
    //     sprintf(tmp, "LoRNoTimer%02i-%02i-%02i_%i",d, m, y, copynumber);
    //     dir = opendir(tmp);
    //     if (dir ==NULL){
    //         sprintf(dirName, "%s", tmp);
    //         std::filesystem::create_directory(dirName);
    //         break;
    //     } //if the directory DOESN'T return a void pointer (= exists already)
    //     else if (dir!=NULL){
    //         copynumber++;                                                           //increase the number at the end of the name
    //         closedir(dir);
    //     }

    // }
                                //should exit when dir is null pointer

    //DATA INTERCFACE
    State desiredState;
    Configurator box2d(desiredState);
    StepCallback cb(&box2d);
    DataInterface dataInterface(&box2d, argv[1]);
                //iterate through files
    //b2Vec2 velocity = {0,0};
    for (int j=0; j<i-1;j++){
        dataInterface.newScanAvail();
        cb.step();

    }

    // FINISH BENCHMARKING
    // auto end = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    // printf("scanned in %f seconds\n", elapsed.count()* 1e-9);




}


