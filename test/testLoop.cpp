#include "callbacks.h"

class AltCallback{
    float L=0,R=0;
    Configurator * c;

    vertexDescriptor start;
public:
        int ogStep=0;
        
    AltCallback(Configurator * _c): c(_c){
        start=c->movingEdge.m_target;
    }
    void step(){
        ExecutionError ee =c->trackTaskExecution(*c->getTask());
        Task::Action action= c->getTask()->getAction();
        c->getTask()->correct(action, c->getTask()->motorStep);
        EndedResult er = c->controlGoal.checkEnded();
	    if (er.ended){
            Disturbance new_goal(PURSUE, c->transitionSystem[start].endPose.p);
		    c->controlGoal = Task(new_goal, UNDEFINED);
            printf("goal reached\n");
	    }
        if (er.ended || start==c->movingVertex){
		    start=c->currentVertex;
	    }
	    c->planVertices =c->changeTask(c->getTask()->change,  ogStep, c->planVertices);
        L=c->getTask()->getAction().getLWheelSpeed();
        R= c->getTask()->getAction().getRWheelSpeed();
        printf("L=%f, R=%f\n", L, R);
    }
};


int main(int argc, char** argv){
    //we imagine that we have executed a plan and then the robot is instructed to go back on its steps
	Disturbance target(PURSUE, b2Vec2(BOX2DRANGE, 0));    
    Task controlGoal(target, DEFAULT);
    bool debug=0;
    Configurator conf(controlGoal, debug);
    conf.simulationStep=0.5;
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    AltCallback cb(&conf);
    ci.data.emplace(Pointf(-1.0, -1.0));
    ci.data2fp.emplace(Pointf(-1.0, -1.0));
    conf.Spawner(ci.data, ci.data2fp);
    cb.step();
    conf.planVertices =conf.changeTask(1, cb.ogStep, conf.planVertices);
    conf.updateGraph(conf.transitionSystem, ExecutionError(), controlGoal.disturbance.pose());
    cb.step();
    conf.Spawner(ci.data, ci.data2fp);
    printGraph(conf.transitionSystem);
   // conf.planner(conf.transitionSystem, src);
    conf.printPlan();
    conf.planVertices =conf.changeTask(1, cb.ogStep, conf.planVertices);

}