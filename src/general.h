#ifndef GENERAL_H
#define GENERAL_H

#include <vector>
#include <map>
#include <set>
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include </usr/include/boost/container/map.hpp>
#include "state.h"

struct Edge{
	State::Direction direction;
	State::simResult::resultType outcome;
	//int stepDuration =0;
	float distanceCovered=0;
};

struct Node{
	State::Object obstacle;
	std::vector <State::Direction> options;
    //b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
	b2Transform endPose;
	float costSoFar =0;

	//the cost of a (state, motor plan, command?, task) is informed by 
	// 1. the cost of the previous
	// 2. whether or not it is baseline (if not baseline robot doesn't move)
	// 3. how far it allows the robot to go
	// 4. whether or not it runs the robot into an obstacle (safe distance)
	//best possible cost = 0
	void updateCost(float prevCost, Edge  e){
		float normDistanceCost = 1-e.distanceCovered /BOX2DRANGE;
		costSoFar = prevCost + normDistanceCost ; //arbitrary weight of 0.1 to task cost 
	}
	
};


typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, State, State::simResult> Graph;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Node, Edge> CollisionTree;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, State, int> G;

typedef boost::graph_traits<Graph>::vertex_iterator vertexIterator; 
typedef boost::graph_traits<Graph>::vertex_descriptor vertexDescriptor;
typedef boost::graph_traits<Graph>::edge_descriptor edgeDescriptor;
typedef boost::graph_traits<Graph>::edge_iterator edgeIterator;



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
		if (this->x < point.x+radius && this->x >point.y-radius && this->y < point.y+radius && this->y >point.y-radius){
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

class Pruner : public boost::default_dfs_visitor{
	public:
	// Graph &graph;

	// Pruner(Graph &g){
	// 	graph = (g);
	// }

	void discover_vertex(vertexDescriptor v, const Graph & g){
		//graph = g;
		printf("%i\n", v);
	}

	void start_vertex(vertexDescriptor v, const Graph & g){
		//graph = g;
		printf("start %i\n", v);
	}

	void finish_vertex(vertexDescriptor v, const Graph & g){
		//graph = g;
		printf("finish %i\n", v);
	}
	// void discover_vertex(vertexDescriptor v, const Gg & g){
	// 	//graph = g;
	// 	printf("%i\n", v);
	// }

	// void start_vertex(vertexDescriptor v, const Gg & g){
	// 	//graph = g;
	// 	printf("start %i\n", v);
	// }

	// void finish_vertex(vertexDescriptor v, const Gg & g){
	// 	//graph = g;
	// 	printf("finish %i\n", v);
	// }

};

//Median filter?? kernel of size 5cm 
//1. take point, make it center of kernel, median filter only applies to that kernel or it'll fuck all the data up
//test here
//need to set the extremes of the kernel and centre and that'll be one poitn
// typedef std::map <float, Point> map;
// class MedianFilter{
// 	struct Kernel{
// 		const float size = 0.03;
// 		Point centre;
// 		std::vector <map> buffer;
// 	};
// 	Kernel kernel;
// public:
//     void filterFloat(Point p){ //value gets modified in the input vector
// 		//if centre of kernel not valid it means that this is the first point in the kernel
// 		if (kernel.centre.valid){
// 			if (p.isInRadius(kernel.centre, kernel.size)){
// 				buffer.push_back(map(p.r, p)); //they should be organised from smaller to larger
// 				//can organise points 	
// 			}
// 		}
// 		else{
// 			kernel.centre = p;
// 		}
// 		//check that this is the current kernel

// 		//add point to kernel
//         buffer.erase(buffer.begin());
//         std::vector <float> tmp = buffer;
//         std::sort(tmp.begin(), tmp.end());
//         int index = int(buffer.size()/2)+1;
//         c = tmp[index];
//         //printf("median value at index: %i, value = %f\n", index, tmp[index]);
//     }
//     void applyToPoint(P &p){
//         filterFloat(p.x, bufferX);
//         filterFloat(p.y, bufferY);
//     }

// };
#endif