#include "callbacks.h"

int main(int argc, char** argv){
    if (argc<2){
        return 0;
    }
    //we imagine that we know a priori that there will be an invisible obstacle behind
    //the D that is avoided by node 26. The robot thinks that by transitioning through states
    //29-30 it will avoid the D successfully but actually not
	Disturbance target(PURSUE, b2Vec2(BOX2DRANGE, 0));    
    Task controlGoal(target, DEFAULT);
    bool debug=0;
    Configurator conf(controlGoal, debug);
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    DataInterface lidar(&ci);
    lidar.folder=argv[1];
    lidar.newScanAvail();
    conf.Spawner(ci.data, ci.data2fp);
    vertexDescriptor v1, v0=24, v1_old=25, src=1;    
    conf.planner(conf.transitionSystem, src);
    printGraph(conf.transitionSystem);
    conf.printPlan();
    if (conf.transitionSystem.m_vertices.size()<30){
        printf("data not suitable, size=%i\n", conf.transitionSystem.m_vertices.size());
        return 0;
    }
    //manually inserting a disturbance

    conf.transitionSystem[v0].options={DEFAULT};
    std::pair <State, Edge> sk;
    simResult sr;
    sr.resultCode=simResult::crashed;
    float x=conf.transitionSystem[v0].disturbance.pose().p.x;
    float y=conf.transitionSystem[v1_old].endPose.p.y;
    b2Vec2 d_pos(x, y);
    sr.collision= Disturbance(AVOID, d_pos);
    sr.endPose= b2Transform(b2Vec2(conf.transitionSystem[26].disturbance.pose().p.x,
        conf.transitionSystem[v1_old].endPose.p.y+.05), 
        b2Rot(conf.transitionSystem[30].endPose.q.GetAngle()));
    gt::fill(sr, &sk.first, &sk.second);
    sk.first.direction=DEFAULT;
    bool topDown=1;
    std::pair<edgeDescriptor, bool> e= conf.addVertex(v0, v1, conf.transitionSystem, Disturbance(), sk.second, topDown);
	if (!e.second){
        printf("what??\n");
    }
    gt::set(e.first, sk, conf.transitionSystem, v1==conf.currentVertex, conf.errorMap, conf.getIteration());
	gt::adjustProbability(conf.transitionSystem, e.first);
    conf.Spawner(ci.data, ci.data2fp);
    printGraph(conf.transitionSystem);
    conf.planner(conf.transitionSystem, src);
    conf.printPlan();
}