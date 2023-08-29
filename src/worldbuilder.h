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
    std::pair <CoordinateContainer, bool> salientPoints(b2Transform, CoordinateContainer, std::pair <Point, Point>, Task*curr=NULL); //gets points from the raw data that are relevant to the task based on bounding boxes

    void makeBody(b2World&, BodyFeatures);

    std::vector <BodyFeatures> processData(CoordinateContainer);

    void checkDisturbance(Point, bool&,Task * curr =NULL);

    bool buildWorld(b2World&,CoordinateContainer, b2Transform, Direction, Task*curr=NULL, bool discrete =0);

    std::pair <Point, Point> bounds(Direction, b2Transform t, float boxLength, Task* curr=NULL); //returns bottom and top of bounding box


};

class AlternateBuilder:WorldBuilder{
    public:
    std::vector <BodyFeatures> processData(CoordinateContainer); //makes every other body
};
