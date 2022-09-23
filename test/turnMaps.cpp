#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <math.h>

int main(int argc, char** argv){
    if (sizeof(argv)<=sizeof(int)){
        printf("select a txt input file\n");
        return -1;
    }
    std::vector <cv::Point2d> vec;
    std::ifstream file(argv[1]);
    double x, y;
    while (file>>x>>y){
        vec.push_back(cv::Point2d(x,y)); //fill vector
        //printf("%2f, %2f\n", x, y);
    }
    file.close();


    //apply affine transfomration
    std::vector <cv::Point2d> transformedVec(vec.size());
    double turningAngle =M_PI_2;
    cv::Mat transformMatrix = (cv::Mat_<double>(2,3)<<cos(turningAngle), -sin(turningAngle), 0, sin(turningAngle), cos(turningAngle), 0);
    cv::transform(vec, transformedVec,transformMatrix);//transforms input

    //save to file

    char name[25];
    sprintf(name,"%strans%s", argv[1]);
    FILE * out=fopen(name, "w+");
    for (cv::Point2d p:transformedVec){
        fprintf(out,"%f\t%f\n", p.x, p.y);
    }
    fclose(out);


    //see if it works correctly
}