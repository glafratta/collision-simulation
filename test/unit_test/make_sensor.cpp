#include "../callbacks.h"


int main(int argc, char** argv){
    Configurator conf;
    b2World world(b2Vec2(0,0));
    //boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    b2Transform start, d_pose=b2Transform(b2Vec2(0.68, 0), b2Rot(0));
    conf.data2fp.emplace(getPointf(d_pose.p));
    BodyFeatures bf(d_pose);
    bf.halfLength=0.045;
    bf.halfWidth=0.02;
    bf.attention=1;
    Disturbance obstacle(bf); 
    Task task(obstacle, DEFAULT, start,true);
    if (obstacle.vertices().empty()){
        return 2;
    }
    if (argc>3){
        start.p.x=atof(argv[1]);
        start.p.y=atof(argv[2]);
        start.q.Set(atof(argv[3]));
    }
    if (task.disturbance.vertices().empty()){
        return 3;
    }
    Robot robot(&world);
    robot.body->SetTransform(start.p, start.q.GetAngle());
    conf.worldBuilder.buildWorld(world, conf.data2fp, task.start, task.direction, task.disturbance,0.15, WorldBuilder::PARTITION);
    task.makeRobotSensor(robot.body);
    debug_draw(world, atoi(argv[4]));
    char name_v[256];
    sprintf(name_v, "/tmp/debug_disturbance_v_%i.txt", atoi(argv[4]));
    FILE * f= fopen(name_v, "w");
    auto v=obstacle.vertices();
    for (b2Vec2 p:v){
        fprintf(f, "%f\t%f\n", p.x, p.y);
    }
    fclose(f);

    return 0;
}