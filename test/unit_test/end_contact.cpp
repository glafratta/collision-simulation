#include "../callbacks.h"



int main(int argc, char** argv){
    Configurator conf;
    b2World world(b2Vec2(0,0));
    //boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    b2Transform start=b2Transform(b2Vec2(0,0), b2Rot(0)), d_pose=b2Transform(b2Vec2(0, -.35), b2Rot(0));
    conf.data2fp.emplace(getPointf(d_pose.p));
    BodyFeatures bf(d_pose);
    bf.halfLength=0.02;
    bf.halfWidth=0.05;
    bf.attention=1;
    Disturbance obstacle(bf); 
    Disturbance goal(PURSUE, b2Vec2(0.5,0.5));
    obstacle.validate();
    int expected=100;
    Task task;
    if (argc==1){
        task=Task(obstacle, DEFAULT, start,true);
        expected=abs((ROBOT_HALFWIDTH+(ROBOT_HALFWIDTH+ROBOT_BOX_OFFSET_X))+(start.p.x-d_pose.p.x) + bf.halfWidth)*HZ/task.action.getLinearSpeed();
        conf.worldBuilder.buildWorld(world, conf.data2fp, task.start, task.direction, task.disturbance,0.15, WorldBuilder::PARTITION);
    }
    else if (argc==2){
        task=Task(goal, DEFAULT, start,true);
        expected=abs((start.p.x-goal.bf.pose.p.x))*HZ/task.action.getLinearSpeed();
    }
    debug_draw(world, 8);
    simResult sr= task.willCollide(world, 69, true);
    //linear speed =0.1
    if (sr.step!=expected){
        printf("step=%i, expected=%i\n", sr.step, expected);
        debug::print_pose(sr.endPose);
        return 1;
    }
    return 0;
}