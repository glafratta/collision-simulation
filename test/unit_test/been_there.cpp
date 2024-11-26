#include "../callbacks.h"

int main(int argc, char**argv){
    Disturbance target=Disturbance(PURSUE, b2Vec2(1.0,0), 0);
    Task goal(target,DEFAULT);
   Configurator conf(goal);
    conf.setSimulationStep(0.27);
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    DataInterface di(&ci);
    di.folder=argv[1];
    di.newScanAvail();          
    conf.data2fp = ci.data2fp;
    conf.addIteration();
    b2World world(b2Vec2(0,0));    conf.addIteration();
    conf.dummy_vertex(conf.currentVertex);
    conf.explorer(conf.currentVertex, conf.transitionSystem, *conf.getTask(), world);
    std::vector <vertexDescriptor> plan=conf.planner(conf.transitionSystem, conf.currentVertex);
    b2Transform shift= b2Transform(b2Vec2(1,0), b2Rot(0));
    conf.controlGoal.disturbance.setPose(-shift);
    boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    conf.applyAffineTrans(shift, conf.transitionSystem);
    conf.addIteration();
    std::pair <bool, vertexDescriptor> result=conf.been_there(conf.transitionSystem, target);
    vertexDescriptor solution=conf.currentVertex;
    if (!result.first){
        return 1;
    }
    if (result.second!=solution){
        return int(result.second);
    }
    return 0;
}