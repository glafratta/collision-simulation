#ifndef WORLDBUILDER_H
#define WORLDBUILDER_H
#include "sensor.h"



class WorldBuilder{
    int bodies=0;
    public:
    int iteration=0;
    bool debug =0;
    char bodyFile[100];
    float simulationStep=BOX2DRANGE;
    //int buildType=0;
    std::pair <CoordinateContainer, bool> salientPoints(b2Transform, CoordinateContainer, std::pair <Pointf, Pointf>); //gets points from the raw data that are relevant to the task based on bounding boxes
                                                                                                                                        //std::pair<points, obstaclestillthere>
    void makeBody(b2World&, BodyFeatures);

    std::vector <BodyFeatures> processData(CoordinateContainer);

    bool checkDisturbance(Pointf, bool&,Task * curr =NULL, float range=0.025);

    std::vector <BodyFeatures> getFeatures(CoordinateContainer , b2Transform, Direction , float);

    void buildWorld(b2World&,CoordinateContainer, b2Transform, Direction,  Disturbance disturbance=Disturbance());

    std::pair <Pointf, Pointf> bounds(Direction, b2Transform t, float boxLength); //returns bottom and top of bounding box

    std::pair<bool,BodyFeatures> getOneFeature(std::vector <Pointf>);//gets bounding box of points

    b2Vec2 averagePoint(CoordinateContainer, Disturbance &, float rad = 0.025); //finds centroid of a poitn cluster, return position vec difference

    int getBodies(){
        return bodies;
    }

    void resetBodies(){
        bodies =0;
    }

    bool occluded(CoordinateContainer, Disturbance);
};
#endif