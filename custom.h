#include "configurator.h"
#include "libcam2opencv.h"
#include "a1lidarrpi.h"
#include "alphabot.h"
#include "CppTimer.h"
#include <stdio.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#define _USE_MATH_DEFINES
//const float LEFT_WHEEL_WEIGHT =.9;
std::vector <BodyFeatures> WorldBuilder::processData(CoordinateContainer points){
    int count =0;
    std::vector <BodyFeatures> result;
    for (Pointf p: points){
        if (count%2==0){
            BodyFeatures feature;
            feature.pose.p = getb2Vec2(p); 
            result.push_back(feature);  
        }
        count++;
    }
    return result;
}


class LidarInterface : public A1Lidar::DataInterface{
ConfiguratorInterface * ci;
// CoordinateContainer coordinates = {};
// CoordinateContainer coordinates2fp = {};
public: 
    int mapCount =0;

    LidarInterface(ConfiguratorInterface * _ci): ci(_ci){}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
	    //ci->setReady(0);
		if (ci == NULL){
			printf("null pointer to ci\n");
			return;
		}
		//ci->ready=0;
		ci->data.clear();
		ci->data2fp.clear();
		mapCount++;
		//std::vector <Point> current;
		Pointf p, p2f;
		FILE *f;
		char name[256];
		sprintf(name,"/tmp/map%04i.dat", mapCount);
		printf("%s\n", name);
		f=fopen(name, "w");
		// coordinates.clear();
		// coordinates2fp.clear();
		for (A1LidarData &data:data){
			if (data.valid&& data.r <LIDAR_RANGE){
				//DATA IS ROUNDED AND DUPLICATES ARE ELIMINATED
				float x = round(data.x*1000)/1000;
				float y = round(data.y*1000)/1000;
				float x2 = round(data.x*100)/100;
				float y2 = round(data.y*100)/100;
				p= (Pointf(x, y));
				p2f=Pointf(x2, y2);
				//coordinates.insert(p);
				//coordinates2fp.insert(p2f);
				ci->data.insert(p);
				ci->data2fp.insert(p2f);
				if (ci->debugOn){
					fprintf(f, "%.2f\t%.2f\n", p2f.x, p2f.y);
				}
            }
		}
		fclose(f);
		printf("added data to lidar containers, coord = %i, coord2fp = %i\n", ci->data.size(), ci->data2fp.size());


		if (!ci->data.empty()){
			ci->setReady(1);
		}
		ci->iteration++;
		printf("added data to interface containers\n");

	}


};

class Callback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
    //int iteration=0;
    //int confIteration=0;
    Configurator * c;
    float L=0;
	float R=0;

public:
int ogStep=0;

Callback(Configurator *conf): c(conf){
}
void step( AlphaBot &motors){
	printf("g size=%i, current vertex=%i\n", c->transitionSystem.m_vertices.size(), c->currentVertex);
	c->printPlan();
	if (c ==NULL){
		printf("null pointer to configurator in stepcallback\n");
	}
	if (c->getIteration() <=0){
		return;
	}
	if (c->planVertices.empty()){
		motors.setRightWheelSpeed(0);
 	   motors.setLeftWheelSpeed(0);		
	}
	//c->trackTaskExecution(*(c->getTask()));	
    ExecutionError ee =c->trackTaskExecution(*c->getTask());
    if (c->getTask()->motorStep>0){
        Task::Action action= c->getTask()->getAction();
        c->getTask()->correct(action, MOTOR_CALLBACK);
    }
	EndedResult er = c->controlGoal.checkEnded();
	if (er.ended){
		c->controlGoal.change =1;
	}

	printf("tracking, d is x =%f, y=%f\n", c->controlGoal.disturbance.pose().p.x, c->controlGoal.disturbance.pose().p.y);
	c->changeTask(c->getTask()->change,  ogStep);
    motors.setRightWheelSpeed(c->getTask()->getAction().getRWheelSpeed()); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(c->getTask()->getAction().getLWheelSpeed());
	printf("og step: %i ,R=%f\tL=%f, conf iteration = %i\n", ogStep, c->getTask()->getAction().getRWheelSpeed(), c->getTask()->getAction().getLWheelSpeed(), c->getIteration());
}
};

struct CameraCallback: Libcam2OpenCV::Callback {
    CameraCallback(Configurator * _c):c(_c){}

	virtual void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) {
        if (NULL==c->ci){
            return;
        }
        //c->ci->visual_field=frame;
		// std::vector <cv::Point2f> corners=c->imgProc.get_corners();
		// cv::Mat previousFrame = c->imgProc.get_previous();
		cv::Vec2d optic_flow=c->imgProc.opticFlow(frame);
		c->getTask()->correct.update(optic_flow[0]); //for now just going straight
    }
    private:
    Configurator * c;
};

float Configurator::taskRotationError(){
    float error=0, theta_left=0, theta_right=0; //difference in rotation between left and right visual field
    //get Left
    // if (ci->.empty()){
    //     return error;
    // }
    // cv::Mat left_vf= imgProc.cropLeft(ci->visual_field);
    // //get Right
    // cv::Mat right_vf= imgProc.cropRight(ci->visual_field);
    // //get optic flow
    // b2Vec2 left_optic_flow= imgProc.opticFlow(left_vf, corners_left, previous_grey_left);
    // b2Vec2 right_optic_flow= imgProc.opticFlow(right_vf, corners_right, previous_grey_right);
	// printf("L optic flow=%f, %f\t R optic flow= %f, %f\n", left_optic_flow.x, left_optic_flow.y, right_optic_flow.x, right_optic_flow.x);
    // theta_left=atan(left_optic_flow.y/left_optic_flow.x);
    // theta_right =atan(right_optic_flow.y/right_optic_flow.x);
    // if ((theta_left<0 & theta_right>0)||(theta_left>0 & theta_right<0)){
    //     printf("optic flows diverge\n");
    // }
    // //get difference between optic flows = error
    // error = theta_left-theta_right;
	//error=imgProc.opticFlow(ci->visual_field, imgProc.corners(), imgProc.previous()).x;
    return error;
}