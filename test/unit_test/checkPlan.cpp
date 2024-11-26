#include "../callbacks.h"

int main(int argc, char** argv){
    bool debug=0;
    Disturbance target1;
    vertexDescriptor solution=TransitionSystem::null_vertex();
    if (argc>2){
        solution=vertexDescriptor(atoi(argv[2]));
    }
    else{
        throw;
    }
    if (argc>3){
        if (atoi(argv[3])==1){
            target1= Disturbance(PURSUE, b2Vec2(1.0,0), 0);    
        }
    }
    Task goal(target1,DEFAULT);
    Configurator conf(goal);
    conf.simulationStep=0.27;
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    DataInterface di(&ci);
    if (argc>1){
        di.folder=argv[1];
        di.newScanAvail();          
    }
    conf.data2fp = ci.data2fp;
    conf.addIteration();
    b2World world(b2Vec2(0,0));
    boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    conf.dummy_vertex(conf.currentVertex);
    conf.explorer(conf.currentVertex, conf.transitionSystem, *conf.getTask(), world);
    std::vector <vertexDescriptor> plan=conf.planner(conf.transitionSystem, conf.currentVertex);
    conf.planVertices=plan;
    int og=0;
    conf.changeTask(true, og, conf.planVertices);
    std::vector <vertexDescriptor> options_src;
    State state_tmp;
    int steps= atoi(argv[4])-5;
    float distanceTraversed = MOTOR_CALLBACK*conf.getTask()->action.getLinearSpeed()*steps;
    b2Transform shift;
    shift.q.Set(MOTOR_CALLBACK*conf.getTask()->action.getOmega()*steps);
    shift.p.x= cos(shift.q.GetAngle())*distanceTraversed;
    shift.p.y= sin(shift.q.GetAngle())*distanceTraversed;
    conf.applyAffineTrans(shift, conf.transitionSystem);    
    if (argc>4){
        di.iteration=atoi(argv[4]);
        di.newScanAvail();          
        conf.data2fp = ci.data2fp;
    }
    bool plan_works= conf.checkPlan(world,conf.planVertices, conf.transitionSystem);
    return !plan_works;
}