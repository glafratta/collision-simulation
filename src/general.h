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
#include "primitive.h"

struct Edge{
	Primitive::Direction direction;
	//int stepDuration =0;
	float distanceCovered=0;
};

struct Node{
	Primitive::Object obstacle;
	std::vector <Primitive::Direction> options;
    //b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
	b2Transform endPose;
	float distanceSoFar =0; //just negative of the total distance
	int predecessors =0;
	int nodesInSameSpot =0;
	int totObstacles=0;
	Primitive::simResult::resultType outcome;
	
};

typedef b2Transform Transform;
bool operator!=(Transform const &, Transform const &);



typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Primitive, Primitive::simResult> Graph;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Node, Edge> CollisionGraph;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Node, Edge> Tree;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Primitive, int> G;

typedef boost::graph_traits<CollisionGraph>::vertex_iterator vertexIterator; 
typedef boost::graph_traits<CollisionGraph>::vertex_descriptor vertexDescriptor;
typedef boost::graph_traits<CollisionGraph>::edge_descriptor edgeDescriptor;
typedef boost::graph_traits<CollisionGraph>::edge_iterator edgeIterator;



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
		//printf("%f < x = %f <%f , %f <y= %f<%f\n", point.x-radius, point.x, point.x+radius, point.y-radius, point.y, point.y+radius );
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

template <class P>
P operator+=(P const &, P const &);



class Pruner : public boost::default_dfs_visitor{
	// public:
	// // Graph &graph;

	// // Pruner(Graph &g){
	// // 	graph = (g);
	// // }

	// void discover_vertex(vertexDescriptor v, const Graph & g){
	// 	//graph = g;
	// 	printf("%i\n", v);
	// }

	// void start_vertex(vertexDescriptor v, const Graph & g){
	// 	//graph = g;
	// 	printf("start %i\n", v);
	// }

	// void finish_vertex(vertexDescriptor v, const Graph & g){
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
// template <typename V, typename G, typename E>
// void add_vertex(V &src, V & v1, G &g){ //to be used with directed graph
// 	if (g[src].option.size()>0){
// 		v1 = boost::add_vertex(g);
// 		edgeDescriptor e = boost::add_edge(src, v1,g).first;
// 		g[e].direction = g[src].options[0];
// 		g[src].options.erase(g[src].options.begin());
// 		g[src].out_edges.push_back(e);
// 	}
// }

// template <typename V, typename G, typename E>
// V nextPrimitive(V v, G g, Primitive s, b2World &w, std::vector <V> & _leaves, int iteration){
// 	//INIT 
// 	V v1 = v;
// 	Primitive::simResult result;
// 	edgeDescriptor inEdge;
	
// 	//EVALUATE THE PRESENT PRIMITIVE
// 	if (v.predecessors.size()>0){
// 		result = s.willCollide(w, iteration, v.predecessors[-1].endPose.p, v.predecessors[-1].endPose.q.GetAngle());
// 		inEdge.distanceCovered = result.distanceCovered;

// 	}



// }

#endif