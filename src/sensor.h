#ifndef SENSOR_H
#define SENSOR_H
#include "task.h"

class Pointf: public cv::Point2f{
	public: 

	Pointf(){}

	Pointf(float _x, float _y){
		x=_x;
		y=_y;
	}
    
    Pointf operator+(const Pointf p){
			Pointf result;
			result.x = x + p.x;
			result.y = y+ p.y;
			return result;
	}

};

template<>
struct cv::traits::Depth<Pointf> {enum {value = Depth<cv::Point2f>::value};};

template<>
struct cv::traits::Type<Pointf> {enum {value = CV_MAKETYPE(cv::traits::Depth<Pointf>::value, 2)};};

float length(cv::Point2f const& p);

float angle(cv::Point2f const&);

bool operator <(Pointf const &, Pointf const&);

bool operator >(const Pointf&,  const Pointf&);

typedef std::set<Pointf> CoordinateContainer;

struct comparator{
    bool operator() ( cv::Point2f a, cv::Point2f b ){ //
        return a.y <=b.y;
	}
}; 



b2Vec2 getb2Vec2(cv::Point2f );

Pointf Polar2f(float, float);

template <typename T>
std::vector<T> set2vec(std::set<T>);

class SensorTools{
    std::vector <Pointf> previous;
    public:
    SensorTools(){};

    b2Transform affineTransEstimate(std::vector <Pointf>, Task::Action, float timeElapsed=0.2, float range=1.0);

};

#endif