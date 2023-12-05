#include "task.h"

// class BoxFeatures{
//     private:
//     float halfWindowWidth =.1;
//     public:
    

// };

class BodyFeatures{
    public:
    b2Transform pose {b2Transform(b2Vec2(0,0), b2Rot(0))} ;
    float halfLength=0.001; //x
    float halfWidth=0.001; //y
    b2BodyType bodyType = b2_dynamicBody;
    b2Shape::Type shape = b2Shape::e_polygon;

    BodyFeatures(){}
};


class WorldBuilder{
    int bodies=0;
    public:
    int iteration=0;
    bool debug =0;
    char bodyFile[100];
    float simulationStep=BOX2DRANGE;
    int buildType=0;
    std::pair <CoordinateContainer, bool> salientPoints(b2Transform, CoordinateContainer, std::pair <Point, Point>, Task*curr=NULL, CoordinateContainer * dCloud=NULL); //gets points from the raw data that are relevant to the task based on bounding boxes
                                                                                                                                        //std::pair<points, obstaclestillthere>
    void makeBody(b2World&, BodyFeatures);

    std::vector <BodyFeatures> processData(CoordinateContainer);

    bool checkDisturbance(Point, bool&,Task * curr =NULL);

    std::pair<bool, b2Vec2> buildWorld(b2World&,CoordinateContainer, b2Transform, Direction, Task*curr=NULL, CoordinateContainer * dCloud = NULL);

    std::pair <Point, Point> bounds(Direction, b2Transform t, float boxLength, Task* curr=NULL); //returns bottom and top of bounding box

    b2Vec2 averagePoint(CoordinateContainer, Disturbance &, float rad = 0.025); //finds centroid of a poitn cluster, return position vec difference

    int getBodies(){
        return bodies;
    }

    void resetBodies(){
        bodies =0;
    }
};
