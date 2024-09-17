#ifndef WORLDBUILDER_H
#define WORLDBUILDER_H
#include "sensor.h"

class WorldBuilder{
    int bodies=0;
    public:
        struct CompareCluster{
        CompareCluster()=default;

        bool operator()(const cv::Point2f & p1, const cv::Point2f & p2){ //distances of centre from start
            bool result=false;
            if (fabs(atan2(p1.y, p1.x))< fabs(atan2(p2.y, p2.x)) && length(p1)<= length(p2)){
                result=true;
            }
            return result;
        }
    };

    int iteration=0;
    bool debug =0;
    char bodyFile[100];
    float simulationStep=BOX2DRANGE;
    //int buildType=0;
    std::pair <CoordinateContainer, bool> salientPoints(b2Transform, CoordinateContainer, std::pair <Pointf, Pointf>); //gets points from the raw data that are relevant to the task based on bounding boxes
                                                                                                                                        //std::pair<points, obstaclestillthere>
    void makeBody(b2World&, BodyFeatures);

    std::vector <BodyFeatures> processData( CoordinateContainer, const b2Transform&);

    bool checkDisturbance(Pointf, bool&,Task * curr =NULL, float range=0.025);

    std::vector <BodyFeatures> getFeatures(CoordinateContainer , b2Transform, Direction , float, float halfWindowWidth=.15);

    std::vector <BodyFeatures> buildWorld(b2World&,CoordinateContainer, b2Transform, Direction,  Disturbance disturbance=Disturbance(), float halfWindowWidth=.15);

    std::pair <Pointf, Pointf> bounds(Direction, b2Transform t, float boxLength, float halfWindowWidth); //returns bottom and top of bounding box

    template <typename Pt>
    std::pair<bool,BodyFeatures> getOneFeature(std::vector <Pt>nb){//gets bounding box of points
    float  l=(0.0005*2), w=(0.0005*2) ;
    float x_glob=0.0f, y_glob=0.0f;
    // cv::Rect2f rect(x_loc,y_loc,w, h);
    // b2Transform pose;
    std::pair <bool, BodyFeatures> result(0, BodyFeatures());
    if (nb.empty()){
        return result;
    }
    CompareX compareX;
    CompareY compareY;
    //Pointf maxx, minx, miny, maxy;
	typename std::vector<Pt>::iterator maxx=std::max_element(nb.begin(), nb.end(), compareX);
	typename std::vector<Pt>::iterator miny=std::min_element(nb.begin(), nb.end(), compareY);
	typename std::vector<Pt>::iterator minx=std::min_element(nb.begin(), nb.end(), compareX);
	typename std::vector<Pt>::iterator maxy=std::max_element(nb.begin(), nb.end(), compareY);
    if (minx->x!=maxx->x){
        w= fabs((*maxx).x-(*minx).x);
    }
    if (miny->y!=maxy->y){
        l=fabs((*maxy).y-(*miny).y);
    }
    x_glob= ((*maxx).x+(*minx).x)/2;
    y_glob= ((*maxy).y+(*miny).y)/2;
    result.second.halfLength=l/2;
    result.second.halfWidth=w/2;
    result.second.pose.p=b2Vec2(x_glob, y_glob);
    result.first=true;
}
    std::vector <std::vector<cv::Point2f>> feature_clusters( std::vector <cv::Point2f>, std::vector <cv::Point2f>&);

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