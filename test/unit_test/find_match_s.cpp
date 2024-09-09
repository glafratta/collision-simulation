#include "../callbacks.h"

int main(int argc, char** argv){
    bool debug=0;
    Disturbance target1;
    if (argc>2){
        if (atoi(argv[2])==1){
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
    std::vector <vertexDescriptor> options_src;
    std::vector <vertexDescriptor>* options_ptr=&options_src;
    State s_temp;
    if (argc>4){
        di.folder=argv[3];
        di.iteration=atoi(argv[4]);
        di.newScanAvail();          
        conf.data2fp = ci.data2fp;
    }
    else{
        conf.findMatch(s_temp,conf.transitionSystem, NULL, UNDEFINED, StateMatcher::DISTURBANCE, options_ptr);
    }
    if (options_src.empty()){
        return 1;
    }
    if (options_src[0]!=2){
        return 1;
    }
    return 0;
}