#include "../callbacks.h"


int main(int argc, char** argv){
    Task goal=Task();
    Configurator conf(goal);
    conf.simulationStep=std::max(ROBOT_HALFLENGTH, ROBOT_HALFWIDTH)*2;
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    conf.data2fp.emplace(Pointf(atof(argv[1]), atof(argv[2])));
    auto v1 = boost::add_vertex(conf.transitionSystem);
    auto e1 = boost::add_edge(conf.movingVertex, v1, conf.transitionSystem);
    b2World world(b2Vec2(0.0, 0.0));
    conf.worldBuilder.buildWorld(world, conf.data2fp, b2Transform(b2Vec2(0,0), b2Rot(0)),DEFAULT);
    Task task(Disturbance(), DEFAULT);
    simResult result=task.willCollide(world, 1);
    gt::fill(result, &conf.transitionSystem[v1], &conf.transitionSystem[e1.first]);
    int total_step=result.step, counted_step=0;
    std::vector <vertexDescriptor> split =conf.splitTask(v1, conf.transitionSystem, conf.transitionSystem[e1.first].direction, task.start);
    for (vertexDescriptor v:split){ 
        auto ie = gt::inEdges(conf.transitionSystem, v, DEFAULT)[0];
        counted_step+=conf.transitionSystem[ie].step;
    }
    printf("total=%i, counted%i\n", total_step, counted_step);
    if (counted_step!=total_step){
    }
    return counted_step!=total_step;
}