#ifndef GENERAL_H
#define GENERAL_H

#include <vector>
#include <map>
#include <set>
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
//#include <boost/graph/depth_first_search.hpp>
#include </usr/include/boost/container/map.hpp>
#include "task.h"

struct Edge{
	Direction direction;
	//int stepDuration =0;
	float distanceCovered=0;
};

struct Node{
	Task::Disturbance disturbance; //error signal
	b2Transform endPose; 
	float distanceSoFar =0; 
	Task::simResult::resultType outcome;
	//int step=0; //error signal
	std::vector <Direction> options;
	int predecessors =0;
	int nodesInSameSpot =0;
	int totDs=0; //error signal
};

typedef b2Transform Transform;
bool operator!=(Transform const &, Transform const &);
bool operator==(Transform const &, Transform const &);

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Node, Edge> CollisionGraph;
typedef boost::graph_traits<CollisionGraph>::vertex_iterator vertexIterator; 
typedef boost::graph_traits<CollisionGraph>::vertex_descriptor vertexDescriptor;
typedef boost::graph_traits<CollisionGraph>::edge_descriptor edgeDescriptor;
typedef boost::graph_traits<CollisionGraph>::edge_iterator edgeIterator;
typedef std::pair <Task::Disturbance, Direction> TaskSummary;
typedef std::vector <TaskSummary> Plan;

struct Point{
	float x=0;
	float y=0;
	float r=0;
	float phi=0;
	bool valid =0;

	Point(){}

	Point(float _x, float _y): x(_x), y(_y){
		r= sqrt(x*x+y*y);
		phi = atan(y/x);
		valid =1;
	}

	Point(b2Vec2 v): x(v.x), y(v.y){
		r= sqrt(x*x+y*y);
		phi = atan(y/x);
		valid =1;
	}

	Point(float _x, float _y, float _r, float _phi): x(_x), y(_y), r(_r), phi(_phi){
		valid =1;
	}

	void operator=(const Point &p){
		x = p.x;
		y= p.y;
		r= p.r;
		phi = p.phi;
	}

	bool operator==(Point &p){
		return (x == p.x && y == p.y);
	}

	bool operator!=(Point &p){
		if (*this == p){
			return false;
		}
		else if (!(*this ==p)){
			return true;
		}
	}

	Point operator+(const Point &p){
		Point result;
		result.x = x + p.x;
		result.y = y+ p.y;
		result.phi = phi +p.phi;
		result.r = r+p.r;
		return result;
	}


	bool isInRadius(b2Vec2 point, float radius = 0.05){ //check if this point is within a certain radius from another given point
		if (this->x <= point.x+radius && this->x >=point.y-radius && this->y <= point.y+radius && this->y >=point.y-radius){
			return true;
		}
		else{
			return false;
		}
	}

	b2Vec2 getb2Vec2(){
		return b2Vec2(x,y);
	}

	void polarInit(float radius, float angle){
		r= radius;
		phi = angle;
		x = radius *cos(angle);
		y = radius *sin(angle);
	}


};

struct comparator{
    bool operator() ( Point a, Point b ){ //
        return a.y <=b.y;
	}
}; 

typedef Point P;
bool operator<(P const &, P const &);

bool operator>(P const &, P const &);

bool operator==(P const &, P const &);

typedef std::set<Point> CoordinateContainer;

typedef b2Transform DeltaPose;

#endif