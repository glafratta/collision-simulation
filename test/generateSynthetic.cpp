#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "../src/primitive.h"
#include <fstream>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
//#include <dirent.h>



float sanityCheck(char * str){
    char tmpStr[50];
    char exp[50], prstr[256];
    char * ptr = &tmpStr[0];
    float result;
    int c=0;
    for (char a= *(str); a!=NULL; a = *(str++)){
        if (a!='e'){
            *(ptr+c)=a;
            c++;
        }
        else{
           ptr = &exp[0];
           c=0; 
        }
    }
    printf("%f * 10^%f\n", tmpStr, exp);
    result = atof(tmpStr)*pow(10, atoi(exp));
    sprintf(prstr, "%s", str);
    printf("original = %s, new %f = %f\n", prstr, result);
    return result;
    //sprintf(tmpStr, "%.2f", tmpStr);
}

int main(int argc, char ** argv){
    if (get_current_dir_name()!= "test" || get_current_dir_name()!="test/"){
        assert(chdir("test")>-1);
    }
    printf("cwd = %s\n", get_current_dir_name);
    char fileName[256], folderName[256], fileRoot[256];
    int nMaps;
    bool defaultSet;
    float linVel =0.0625, angVel=0;
    printf("Insert name of file you want to generate data from: \n");
    std::cin>>fileName;
    printf("How many synthetic maps do you want?\n");
    std::cin>>nMaps;
    printf("Default settings v=0.0625, w=0. Continue?\n");
    std::cin>>defaultSet;
    if (!defaultSet){
        printf("What linear velocity do you want? (max .125)");
        std::cin>>linVel;
        assert(linVel<=0.125);
        printf("What angular velocity do you want? in fraction of PI (max .5)");
        std::cin>>angVel;
        angVel *=M_PI;
        assert(angVel<=0.5);
    }
    int count=0;
    for (char a:fileName){
        if (a!='.'){
           // sprintf(fileRoot, "%c", a);
           fileRoot[count] = a;
            printf("%c", a);
            count++;
        }
        else{
            break;
        }
    }
    printf("\n");
    sprintf(folderName,"Synth_%s", fileRoot);
    int success;
    if (chdir(folderName)==-1){
        mkdir(folderName, 0777);
        // success =chdir(folderName);
        // if (success<=0){
        //     printf("couldn't make directory\n");
        // }
    }
    else{
        chdir("..");
    }
    printf("saving in folder %s\n", folderName);
    

    // //TODAYS DATE AND TIME
    // time_t now =time(0);
    // tm *ltm = localtime(&now);
    // int y,m,d;
    // y=ltm->tm_year-100;
    // m = ltm->tm_mon +1;
    // d=ltm->tm_mday;
    // char dirName[256];
    // sprintf(dirName, "%02i%02i%02i_%02i%02i", d,m,y, ltm->tm_hour, ltm->tm_min);
    // mkdir(dirName, 0777);

    std::vector <cv::Point2f> points;
    float x,y;
    char tmpX[50], tmpY[256];
    std::ifstream file(fileName);
    char debugFile[256];
    sprintf(debugFile, "%s_debug.txt", fileRoot);
    FILE * dump = fopen(debugFile, "w+");
    while(file>>tmpX>>tmpY){
        x = sanityCheck(tmpX);
        y= sanityCheck(tmpY);
        points.push_back(cv::Point2f(x,y));
        fprintf(dump, "%f\t%f\n", x, y);
    }
    file.close();
    fclose(dump);

    Primitive pr;
    float theta =0;
    float xShift=0;
    float yShift=0;
    float DeltaW = angVel/pr.hz;
    float DeltaV = linVel/pr.hz;
    float DeltaX = -DeltaV *cos(DeltaW);
    float DeltaY = -DeltaV*sin(DeltaW);
    printf("deltaW = %f, deltaV = %f, deltaX = %f, deltay = %f\n", DeltaW, DeltaV, DeltaX, DeltaY);
    cv::Mat transformMatrix = (cv::Mat_<double>(2,3)<<cos(DeltaW), -sin(DeltaW), DeltaX, sin(DeltaW), cos(DeltaW), DeltaY);
    for (int j=0; j<=nMaps; j++){ //create 40 data points corresponding to 40 transformations
        char filePath[256];
        sprintf(filePath,"%s/%s_%04i.txt",folderName, fileRoot, j);
        FILE *  f=fopen(filePath, "w+");
        //printf("xshift = %f, yshift = %f, theta = %f", xShift, yShift, theta);
        // cv::Mat transformMatrix = (cv::Mat_<double>(2,3)<<cos(theta), -sin(theta), xShift, sin(theta), cos(theta), yShift);
        //create return vector which containes the transformation
        std::vector <cv::Point2f> returnVector(points.size());
        printf("return vector size = %i\n", returnVector.size());
        cv::transform(points, returnVector, transformMatrix);
        for (cv::Point2f p:returnVector){
            fprintf(f, "%f\t%f\n", p.x, p.y);
        }
        fclose(f);
        // theta +=DeltaW;
        // xShift+=DeltaX;
        // yShift+=DeltaY;
        points = returnVector;
}
}
    
    
    

