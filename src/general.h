 #ifndef GENERAL_H
#define GENERAL_H
#include <vector>
#include <map>
#include <set>
#include "state.h"
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, State, State::simResult> Graph;
//typedef std::pair<State &, State&> Edge; 
typedef boost::graph_traits<Graph>::vertex_iterator vertexIterator; 
typedef boost::graph_traits<Graph>::vertex_descriptor vertexDescriptor;
typedef boost::graph_traits<Graph>::edge_descriptor edgeDescriptor;
typedef boost::graph_traits<Graph>::edge_iterator edgeIterator;


State::Direction getOppositeDirection(State::Direction d){
    switch (d){
        case State::Direction::LEFT: return State::Direction::RIGHT;break;
        case State::Direction::RIGHT: return State::Direction::LEFT;break;
        default:
        return State::Direction::NONE;
    }
}

bool isFullLength(vertexDescriptor v, Graph &g, float length=0){
	//length = stepdur/hz *linvel
    if (in_degree(v,g)<=0 && length < g[v].box2dRange){
        return false;
    }
    else if (length >=g[v].box2dRange){
        return true;
    }
    else{
        edgeDescriptor inEdge= in_edges(v, g).first.dereference();
        length += g[inEdge].stepDuration/g[v].hz * g[v].getAction().getLinearSpeed();
        vertexDescriptor newV = source(inEdge, g);
        return isFullLength(newV, g, length);
    }

}

#endif