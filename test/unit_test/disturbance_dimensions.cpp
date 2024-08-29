#include "../callbacks.h"


int main(int argc, char** argv){
    //generate points
    std::vector <Pointf> pts;
    float x=0, y=-0.05;
    for (int i=0; i<11; i++){
        pts.push_back(Pointf(x, y));
        y+=0.01;
    }
    WorldBuilder wb;
    std::pair<bool,BodyFeatures> feature= wb.getOneFeature(pts);
    feature.second.halfLength=round(feature.second.halfLength*1000)/1000;
    feature.second.halfWidth=round(feature.second.halfWidth*1000)/1000;

    if (feature.second.halfLength -0.05>0.001){
        throw std::logic_error("wrong half length\n");
    }
    if (feature.second.halfWidth==0){
        throw std::logic_error("wrong half width\n");
    }
    if (feature.second.pose.p.x !=0 || feature.second.pose.p.y!=0){
        throw std::logic_error("wrong pose\n");
    }
    if (!feature.first){
        throw std::logic_error("feature does not exist\n");
    }
    return 0;
}