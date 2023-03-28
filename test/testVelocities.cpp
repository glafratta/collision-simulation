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
    fprintf(f, "%s\nW=window 0.2h, x>0 w\nCorrected(r, angle)\tRaw Affine(r, angle)\tCorrected_W\tRaw Affine_W\nNOTE: %s\n", argv[1], argv[2]);
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
    if (!(f=fopen(readPath, "r"))){
        return 0;
    }
    else {
        fclose(f);
    }
    

        float x, y, x2, y2;
        Point p1;
            //box2d->previous = box2d->current;
            //box2d->current.clear();
            while (file>>x>>y){
                x = round(x*1000)/1000;
                y = round(y*1000)/1000;
                Point p(x,y);
                if (p!=p1){
                data.push_back(p);
            }
            p1=p;
                
            }
            file.close();
            c.timeElapsed=0.2;
            Configurator::getVelocityResult r =c.GetRealVelocity(data, prevData);
            Configurator::getVelocityResult r_w =c.GetVelocityFromReference(data, prevData);           
            prevData = data;
            data.clear();
            f = fopen(filePath, "a");
            fprintf(f,"%.3f,   %.3f pi\t\t%.3f,   %.3f pi\t\t%.3f,    %.3f pi\t\t%.3f,   %.3f pi\n", r.vector.Length(), atan(r.vector.y/r.vector.x)/M_PI, r.affineResult.Length(), 
            atan(r.affineResult.y/r.affineResult.x)/M_PI, r_w.vector.Length(), atan(r_w.vector.y/r_w.vector.x)/M_PI, r_w.affineResult.Length(), 
            atan(r_w.affineResult.y/r_w.affineResult.x)/M_PI);
            fclose(f);
            // c.getDMP()->setRecordedVelocity(r.vector);
            // c.applyController(1, *c.getDMP())

    }



}