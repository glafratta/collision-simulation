#include "../callbacks.h"

bool debug_draw(b2World & w, int file){
    char name_v[256], name_s[256], name_d[256];
    sprintf(name_v, "/tmp/debug_chassis_%i.txt",file);
    sprintf(name_s, "/tmp/debug_sensor_%i.txt",file);
    sprintf(name_d, "/tmp/debug_disturbance_%i.txt",file);
    FILE * f_v=fopen(name_v, "w");
    FILE * f_s=fopen(name_s, "w");
    FILE * f_d=fopen(name_d, "w");
    for (auto b=w.GetBodyList(); b; b=b->GetNext()){
        if (b->GetUserData().pointer){
            b2Fixture* fixture=b->GetFixtureList();
            b2PolygonShape * poly=(b2PolygonShape*)fixture->GetShape();
            int ct= poly->m_count;
            for (int i=0; i<ct; i++){
                b2Vec2* v=poly->m_vertices+i;
                b2Vec2 world_point= b->GetWorldPoint(*v);
                if (b->GetUserData().pointer==ROBOT_FLAG){
                    if (fixture->IsSensor()){
                        fprintf(f_s, "%f\t%f\n",  world_point.x, world_point.y);            
                    }
                    else{
                        fprintf(f_v, "%f\t%f\n",  world_point.x, world_point.y);            
                    }
                }
                else if (b->GetUserData().pointer==DISTURBANCE_FLAG){
                    fprintf(f_d, "%f\t%f\n",  world_point.x, world_point.y);            
                }
            }
        }
    }
    fclose(f_v);
    fclose(f_s);
    fclose(f_d);
}


int main(int argc, char** argv){
    Configurator conf;
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
    //boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    conf.dummy_vertex(conf.currentVertex);
    b2Transform start;
    Task task(DEFAULT);
    conf.worldBuilder.buildWorld(world, conf.data2fp, task.start, task.direction, task.disturbance,0.15, WorldBuilder::PARTITION);
    simResult sim= task.willCollide(world, conf.getIteration());
    Disturbance obstacle=sim.collision; 
    if (argc>4){
        start.p.x=atof(argv[2]);
        start.p.y=atof(argv[3]);
        start.q.Set(atof(argv[4]));
    }
    Task task2(obstacle, DEFAULT, start,true);
    Robot robot(&world);
    task2.makeRobotSensor(robot.body);
    debug_draw(world, atof(argv[5]));
    return 0;
}