#include "../src/configurator.h"


int main(int argc, char**argv){
    assert(argc>1);
    time_t now =time(0);
	tm *ltm = localtime(&now);
    int y, m, d, h, min, iteration=0;
    y=ltm->tm_year-100;
    m = ltm->tm_mon +1;
    d=ltm->tm_mday;
    h = ltm->tm_hour;
    min = ltm->tm_min;
    char dirDump[50];
    sprintf(dirDump, "velocityData");
    if (!opendir(dirDump)){
        mkdir(dirDump, 0777);
    }
    char filePath[256];
    sprintf(filePath, "%s/vel%02i%02i%02i_%02i%02i.txt", dirDump, d,m,y,h, min);
    FILE * f = fopen(filePath, "w");
    fprintf(f, "Vector\tAffineResult\nNOTE: %s", argv[2]);
    fclose(f);
    Configurator c;
    char folder[256];
    sprintf(folder, "%s", argv[1]);
    std::vector <Point> data, prevData;
    while (1){
        iteration++;
            char readPath[256];
            sprintf(readPath, "%smap%04d.dat", folder, iteration);
            std::ifstream file(readPath);

            float x, y, x2, y2;
            //box2d->previous = box2d->current;
            //box2d->current.clear();
            while (file>>x>>y){
                x = round(x*1000)/1000;
                y = round(y*1000)/1000;
                Point p(x,y);
                //Point pp = *(&p-1);
                if (p.r<1.0 && p!=*(&p-1)){
                    //current.push_back(cv::Point2f(x, y));
                    data.push_back(p);
                }
                
            }
            file.close();
            Configurator::getVelocityResult r =c.GetRealVelocity(data, prevData);
            prevData = data;
            data.clear();
            f = fopen(filePath, "a");
            fprintf(f,"%f,%f\t%f, %f\n", r.vector.x, r.vector.y, r.affineResult.x, r.affineResult.y);
            fclose(f);

    }



}