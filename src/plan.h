#ifndef PLAN_H
#define PLAN_H
#include <vector>
#include <map>
#include "state.h"
#include <utility>

class Tree{
	public:

class Node{
public:
    enum nodeType {ROOT, LEFT, RIGHT, PREV, NOT_SET};
    std::map <nodeType, State *>  map{{NOT_SET, NULL}, {NOT_SET, NULL}, {NOT_SET, NULL}}; //L, R, P
    State * state;
    int index =0;

    void addEdge(nodeType t, State * s){ 
        map.emplace(t, s);
    }

};
    
    std::vector <std::vector <Node>> nodes;
	float timeDesired =0;
	float timeAvoid =0;

	int getStepDuration(){
		int result =0;
		if (!nodes.empty()){
			for (Node & node:nodes[-1]){
				State *state = node.state;
				while (node.map.find(Node::nodeType::PREV)->second!=NULL);{
				result += node.state->stepDuration;
				state= node.map.find(Node::nodeType::PREV)->second;
			}
		}
		return result;
	}
	}

	int getObstacleCount(){
		int result =0;
	for (Node & node:nodes[-1]){
				State * state = node.state;
				while (node.map.find(Node::nodeType::PREV)->second!=NULL);{
				if (state->getObstacle().isValid());
				state= node.map.find(Node::nodeType::PREV)->second;
			}
		}
		return result;
	}

	bool full(int maxDuration, int maxObstacle){
		return (getStepDuration()>= maxDuration) || (getObstacleCount()>=maxObstacle);
	}
};

#endif