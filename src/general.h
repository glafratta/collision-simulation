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
#include "state.h"

struct Node{
	State::Object obstacle;
	std::vector <State::Direction> options;
    //b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
	b2Transform endPose;
};

struct Edge{
	State::Direction direction;
	State::simResult::resultType outcome;
	//int stepDuration =0;
	float distanceCovered=0;
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

	Point(){}

	Point(float _x, float _y): x(_x), y(_y){
		r= sqrt(x*x+y*y);
		phi = atan(y/x);
	}

	Point(b2Vec2 v): x(v.x), y(v.y){
		r= sqrt(x*x+y*y);
		phi = atan(y/x);
	}

	Point(float _x, float _y, float _r, float _phi): x(_x), y(_y), r(_r), phi(_phi){}

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

	bool isInSquare(b2Vec2 point, float radius = 0.05){
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
#endif