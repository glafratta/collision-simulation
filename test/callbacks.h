#ifndef CALLBACKS_H
#define CALLBACKS_H

#include <thread>
#include "configurator.h"
#include "CppTimer.h"
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
#define _USE_MATH_DEFINES


void printGraph(TransitionSystem& g){
    boost::print_graph(g);
}

Remember remember;
Visited visited;

template <typename Predicate> 
void printEdges(TransitionSystem& g, Predicate p){
    auto es = boost::edges(g);
    for (auto ei=es.first; ei!=es.second;ei++){
        if (!p(*ei)){
            printf("%i->%i, direction=%i,probability=%f, step=%i\n", (*ei).m_source, (*ei).m_target, g[(*ei)].direction, g[*ei].probability, g[*ei].step);
        }
    }
}

void printEdges(TransitionSystem& g){
    auto es = boost::edges(g);
    for (auto ei=es.first; ei!=es.second;ei++){
        printf("%i->%i, direction=%i,probability=%f, step=%i\n", (*ei).m_source, (*ei).m_target, g[(*ei)].direction, g[*ei].probability, g[*ei].step);
    }
}

void print_forget(TransitionSystem& g){
    Remember p;
    printEdges(g, p);
}

float print_belowP(TransitionSystem& g, float p){
    auto es = boost::edges(g);
    float ct=0;
    for (auto ei=es.first; ei!=es.second;ei++){
        if (g[*ei].probability<p){
            ct++;
            //printf("%i->%i, direction=%i,probability=%f\n", (*ei).m_source, (*ei).m_target, g[(*ei).m_target].direction, g[*ei].probability);
        }
    }
    return ct/g.m_vertices.size();
}

void getVisited(TransitionSystem& g, vertexDescriptor cv){
    auto es = boost::edges(g);
    float ct=0;
    for (auto ei=es.first; ei!=es.second;ei++){
        if ((g[(*ei).m_source].visited()|| (*ei).m_source==0 || (*ei).m_source==cv)& g[(*ei).m_target].visited()){
            ct++;
            //printf("%i->%i, direction=%i,probability=%f\n", (*ei).m_source, (*ei).m_target, g[(*ei).m_target].direction, g[*ei].probability);
        }
    }
}

class DataInterface {
public:
int iteration = 0;
ConfiguratorInterface * ci;

char * folder;
    DataInterface(ConfiguratorInterface * _ci): ci(_ci){}

	bool newScanAvail(){ //uncomment sections to write x and y to files		
        iteration++;
    	ci->ready=0;
      //  ci->updatePCProc();
	//	ci->data.clear();
		ci->data2fp.clear();
		char filePath[256];
        char folderName[256];
        sprintf(folderName,"%s", folder);
		sprintf(filePath, "%smap%04d.dat", folderName, iteration);
        printf("%s\n", filePath);
        FILE *f;
        if (!(f=fopen(filePath, "r"))){
            ci->stop=1;
            return false;
        }
        else {
            fclose(f);
        }
        // std::vector<Pointfff > data;
		std::ifstream file(filePath);
        float x2, y2;
      //  Pointf  p1, p2_1;
		while (file>>x2>>y2){
         //   x = round(x*1000)/1000;
			//y = round(y*1000)/1000;
            x2 = round(x2*100)/100;
			y2 = round(y2*100)/100;
           // Pointf  p(x,y);
            Pointf  p2(x2,y2);
           // ci->data.insert(p);
            ci->data2fp.insert(p2);
		}
		file.close();
        ci->setReady(1);
        ci->iteration++;
        return true;
		


	}
};

class StepCallback{
    float L=0,R=0;
    Configurator * c;
    int ogStep=0;
public:

    StepCallback()=default;

    StepCallback(Configurator * _c): c(_c){}
    void step(){
        if (!c->running ){
            printf("not runing\n");
            return;
        }
        printf("graph size = %i\n", c->transitionSystem.m_vertices.size());
        ExecutionError ee =c->trackTaskExecution(*c->getTask());
        Task::Action action= c->getTask()->getAction();
        c->getTask()->correct(action, c->getTask()->motorStep);
        EndedResult er = c->controlGoal.checkEnded(b2Transform(b2Vec2(0,0), b2Rot(0)), UNDEFINED, true);//true
	    // if (c->getTask()->motorStep==0 && c->planVertices.empty() & c->transitionSystem.m_vertices.size()>2){ //&& (c->transitionSystem[c->movingEdge].step==0 || c->getIteration()<2)
        //     er.ended=1;
        // }
        printf(" task step =%i, vertices empty = %i, direction stop=%i, is it i =%i\n", c->getTask()->motorStep==0,  c->planVertices.empty(), (c->transitionSystem[c->movingEdge].step==0), c->getIteration()<2);
        if (c->controlGoal.disturbance.isValid()){
            printf("distance from goal=%f\n", c->controlGoal.disturbance.getPosition().Length());
        }
        if (er.ended &( c->getTask()->motorStep<1 & c->transitionSystem[c->movingEdge].direction!=STOP && c->planVertices.empty() && c->getIteration()>1)){ //& c->getTask()->motorStep<1
           if (!er.ended){
                printf("task step = %i\n", c->getTask()->motorStep);
           }
            Disturbance new_goal(PURSUE, c->controlGoal.start.p, c->controlGoal.start.q.GetAngle());
		    c->controlGoal = Task(new_goal, UNDEFINED);
            b2Vec2 v = c->controlGoal.disturbance.getPosition() - b2Vec2(0,0);
            printf("new control goal start: %f, %f, %f, distance = %f, valid =%i\n", c->controlGoal.start.p.x, c->controlGoal.start.p.y, c->controlGoal.start.q.GetAngle(), v.Length(), c->controlGoal.disturbance.isValid());
            printf("GOAL REACHED ");
            if (c->getTask()->direction==STOP){
                printf("but stopping");
            }
        	FILE * f = fopen(c->statFile, "a+");
            fprintf(f, "!");
            fclose(f)l

	    }
	    c->planVertices =c->changeTask(c->getTask()->change,  ogStep, c->planVertices);
        L=c->getTask()->getAction().getLWheelSpeed();
        R= c->getTask()->getAction().getRWheelSpeed();
        printf("\n og step=%i, L=%f, R=%f\n", ogStep, L, R);
    }
};

// std::vector <BodyFeatures> WorldBuilder::processData(CoordinateContainer points){
//     int count =0;
// 	//buildType =2;
//     std::vector <BodyFeatures> result;
//     for (Pointf p: points){
//         if (count%2==0){
//             BodyFeatures feature;
//             feature.pose.p = getb2Vec2(p); 
//             result.push_back(feature);  
//         }
//         count++;
//     }
//     return result;
// }

std::vector <BodyFeatures> WorldBuilder::processData(CoordinateContainer points){
    std::vector <BodyFeatures> result;
    std::vector <Pointf> ptset= set2vec(points);
    std::pair<bool,BodyFeatures> feature= getOneFeature(ptset);
    if (feature.first){
        result.push_back(feature.second);
    }
    return result;
}


//FOR THREAD DEBUGGING

class TimerStep: public CppTimer{
    StepCallback sc;
    public:
    TimerStep(StepCallback & _step): sc(_step){}
    void timerEvent(){
        sc.step();
    }
};

class TimerDI: public CppTimer{
    DataInterface di;
    public:
    TimerDI(DataInterface & _di): di(_di){}
    void timerEvent(){
        if (!di.newScanAvail()){
            this->stop();
        }
    }
};

// float Configurator::taskRotationError(){
//     printf("previous coord=%i\n", pcProc.previous.size());
// 	b2Transform velocity = b2Transform(currentTask.getAction().getTransform());
//     float theta_error=0, theta=0;
// 	if (currentTask.action.getOmega()==0){
// 		float dataRange=0.25;
// 	 	velocity= pcProc.affineTransEstimate(std::vector <Pointf>(ci->data2fp.begin(), ci->data.end()), currentTask.action, MOTOR_CALLBACK, dataRange);
//         theta_error = atan(velocity.p.y/velocity.p.x)- currentTask.action.getOmega();
// 	}
// 	else{
//         //displacement will be garbage
//         if (currentTask.disturbance.isValid()){
//             orientation prev_orientation= currentTask.disturbance.getOrientation();
//             std::vector<Pointf> nb =pcProc.setDisturbanceOrientation(currentTask.disturbance, ci->data2fp);
//             theta =subtract(currentTask.disturbance.getOrientation(), prev_orientation).second;
//             theta_error=(theta -currentTask.action.getOmega()*MOTOR_CALLBACK);
//             ExecutionError ee;
//             if (auto it=errorMap.find(transitionSystem[currentVertex].ID); it!=errorMap.end()){
//                 ee.setR(it->second.r());
//             }
//             ee.setTheta(theta_error);
//             errorMap.insert_or_assign(transitionSystem[currentVertex].ID, ee);
//             velocity.q.Set(theta/MOTOR_CALLBACK);
//             //get orientation
//             printf("prev orientation = %f, current orientation = %f, or error=%f, neighbours= %i, theta=%f\n", prev_orientation.second, currentTask.disturbance.pose().q.GetAngle(), theta_error, nb.size(), theta);
//         }
// 		//velocity = b2Transform(currentTask.getAction().getTransform()); //open loop
// 	}
// 	currentTask.action.setRec(SignedVectorLength(velocity.p), velocity.q.GetAngle());
//     // ExecutionError exErr;
//     // if (auto it=errorMap.find(transitionSystem[currentVertex].ID); it!=errorMap.end()){
//     //     exErr= it->second;
//    // }
//     //exErr.setTheta(error);
//     //errorMap.insert_or_assign(g[currentVertex].ID, exErr);
//     return theta_error;
// }

 #endif