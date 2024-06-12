#include "callbacks.h"
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


    char filePrefix[5];
    sprintf(filePrefix, "map");
    int fileCount =1;
    char file[256];
    FILE *f;
    while ( sprintf(file, "%s%s%04i.dat", argv[1], filePrefix, fileCount)>-1 &&(f = fopen(file, "r"))!=NULL){   
        fileCount++;
        memset(file, 0, sizeof(file));
        fclose(f);
    }
    printf("\n%i files\n", fileCount);

    //DATA INTERCFACE

    bool timerOff=atoi(argv[2]);
    Task controlGoal;
    //controlGoal.init();
    Configurator configurator(controlGoal, 1, timerOff);
    configurator.planning=1;
    if (argc>3){
		configurator.setSimulationStep(atof(argv[3]));
    }
    ConfiguratorInterface configuratorInterface;
    configurator.registerInterface(&configuratorInterface);
    DataInterface dataInterface(&configuratorInterface); 
    dataInterface.folder = argv[1];
    StepCallback cb(&configurator);

    if (!timerOff){
        TimerDI lidar(dataInterface);
        TimerStep motors(cb);
        lidar.startms(200);
        motors.startms(100);
        configurator.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(200*fileCount)); //simulates lidar
        lidar.stop();
        motors.stop();
        configurator.stop();
    }
    else if (timerOff){
                // b2Vec2 velocity = {0,0};
        //for (int i=0; i<40;i++){
        while  (dataInterface.newScanAvail()){
            configurator.running=1;           
            //configurator.Spawner(configuratorInterface.data, configuratorInterface.data2fp);
            if (configuratorInterface.isReady()){
                if (configuratorInterface.data2fp != configurator.data2fp){
                    //printf("confINt data size = %i\n", configuratorInterface.data.size());
                    configurator.data2fp=configuratorInterface.data2fp;
                    configurator.Spawner();
                    // configuratorInterface.data.clear();
                    // configuratorInterface.data2fp.clear();
                    // configuratorInterface.ready=0;
			}
            }
            cb.step();
            cb.step();
        }
        configurator.running=0;
    }



    // FINISH BENCHMARKING
    // auto end = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    // printf("scanned in %f seconds\n", elapsed.count()* 1e-9);




}