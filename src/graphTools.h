#ifndef GENERAL_H
#define GENERAL_H
#include <set>
//#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp> //LMEDS
#include <vector>
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <map>
#include <boost/property_map/property_map.hpp> //property map
//#include <boost/variant/get.hpp> //get function
#include <boost/graph/copy.hpp>
#include <utility>
#include "disturbance.h"

const float NAIVE_PHI=10.0;
// enum M_CODES {THREE_M=3, FOUR_M=4};

// enum GRAPH_CONSTRUCTION {BACKTRACKING, A_STAR, A_STAR_DEMAND, E};
enum VERTEX_LABEL {UNLABELED, MOVING, ESCAPE, ESCAPE2};

typedef std::vector <float> DistanceVector;

typedef std::vector <bool> MatchVector;

struct Edge{
	Direction direction=DEFAULT;
	float probability=1.0;
	int step=0;
	int it_observed=-1; //last iteration where this edge was observed

	Edge()=default;

	Edge(Direction d):direction(d){}

	float weighted_probability(int it){
		float result=0;
		if (it_observed>=0){
			result=probability*float(it_observed)/float(it);
		}
		return result;
	}
};


struct State{
	Disturbance disturbance; //disturbance encounters
	b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0)); 
	simResult::resultType outcome;
	std::vector <Direction> options;
	int nodesInSameSpot =0;
	bool filled =0;
	int nObs=0;
	State* ID=this;
	float phi=NAIVE_PHI; //arbitrarily large phi
	VERTEX_LABEL label=VERTEX_LABEL::UNLABELED;
	//Direction direction=DEFAULT;

	
	State()=default;

	//State(Direction d): direction(d){}

	bool visited(){
		return phi<=1.0;
	}

	void resetVisited(){
		phi=10.0;
	}

};


typedef b2Transform Transform;
bool operator!=(Transform const &, Transform const &);
bool operator==(Transform const &, Transform const &);
void operator-=(Transform &, Transform const&);

typedef std::pair<bool, float> orientation;
orientation subtract(orientation, orientation);

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::bidirectionalS, State, Edge> TransitionSystem;
typedef boost::graph_traits<TransitionSystem>::vertex_iterator vertexIterator; 
typedef boost::graph_traits<TransitionSystem>::vertex_descriptor vertexDescriptor;
typedef boost::graph_traits<TransitionSystem>::edge_descriptor edgeDescriptor;
typedef boost::graph_traits<TransitionSystem>::edge_iterator edgeIterator;

struct Connected{
	Connected(){}
	Connected(TransitionSystem * ts): g(ts){}
	
	bool operator()(const vertexDescriptor& v)const{
	 	bool in= boost::in_degree(v, *g)>0;
		bool out =boost::out_degree(v, *g)>0;
	 	return in || out;
	}
private:
TransitionSystem * g;
};

struct MoreLikely{
	bool operator()(Edge e1, Edge e2){//const
		return e1.probability >e2.probability;
	}
};


struct Remember{
	Remember(){}
	Remember(TransitionSystem* ts):g(ts){}

	bool operator()(const edgeDescriptor& e){//const
		if ((*g)[e].probability<FORGET_THRESHOLD){ //filter signal
		 	return false;
		 }
		return true;
	}

	private: 
	TransitionSystem *g;
	//std::set <edgeDescriptor> forget;
};

struct Visited{ //for debug
	Visited(){}
	Visited(TransitionSystem * ts):g(ts){}

	bool operator()(const vertexDescriptor&v)const{
		return (*g)[v].visited();
	}
	private:
	TransitionSystem *g;
};

struct is_not_v{
	is_not_v(){}
	//CurrentV(TransitionSystem * ts): g(ts){}
	is_not_v(vertexDescriptor _cv): cv(_cv){}
	bool operator()(edgeDescriptor e){
		return e.m_target!=cv;
	}	

	private:
	vertexDescriptor cv;
};

struct ExecutionError{

	ExecutionError(){}

	ExecutionError(float fr, float ft){
		_r=fr;
		_theta=ft;
	}

	float r(){
		return _r;
	}

	float theta(){
		return _theta;
	}

	void setTheta(float f){
		_theta=f;
	}

	void setR(float f){
		_r=f;
	}
	private:
	float _r=0;
	float _theta=0;
};

typedef std::pair<vertexDescriptor, std::vector<vertexDescriptor>> Frontier;

struct ComparePhi{

	ComparePhi(){}

	bool operator()(const std::pair<State*, Frontier>& p1, const std::pair<State*, Frontier>& p2) const{
		return (*p1.first).phi<(*p2.first).phi;
	}
};


namespace gt{

	void fill(simResult, State* s=NULL, Edge* e=NULL);

	int simToMotorStep(int);

	int distanceToSimStep(float, float){

	void update(edgeDescriptor,  std::pair <State, Edge>, TransitionSystem&, bool, std::unordered_map<State*, ExecutionError>&, int); //returns disturbance rror based on expected vs observed D

	void set(edgeDescriptor,  std::pair <State, Edge>, TransitionSystem&, bool, std::unordered_map<State*, ExecutionError>&, int);

	std::pair< bool, edgeDescriptor> getMostLikely(TransitionSystem&,std::vector<edgeDescriptor>, int);

	std::vector <edgeDescriptor> outEdges(TransitionSystem&, vertexDescriptor, Direction); //returns a vector containing all the out-edges of a vertex which have the specified direction

	std::vector <edgeDescriptor> inEdges(TransitionSystem&, const vertexDescriptor&, const Direction & d = UNDEFINED); //returns a vector containing all the in-edges of a vertex which have the specified direction

	Disturbance getExpectedDisturbance(TransitionSystem&, vertexDescriptor, Direction, int);

	std::pair <bool,edgeDescriptor> visitedEdge(const std::vector <edgeDescriptor>&, TransitionSystem&, vertexDescriptor cv=TransitionSystem::null_vertex());

	void adjustProbability(TransitionSystem&, edgeDescriptor);

	std::pair <edgeDescriptor, bool> add_edge(vertexDescriptor, vertexDescriptor, TransitionSystem&, int); //wrapper around boost function, disallows edges to self

}



typedef boost::filtered_graph<TransitionSystem, boost::keep_all, Connected> FilteredTS;
typedef boost::filtered_graph<TransitionSystem, boost::keep_all, Visited> VisitedTS;


struct StateMatcher{
        std::vector <float> weights; //disturbance, position vector, angle
		//assume mean difference 0
		//std::vector <float> SDvector={0.03, 0.03, 0, 0.08, 0.08, M_PI/6};//hard-coded standard deviations for matching
		
		struct Error{
			const float endPosition=0.05;//0.05;
			const float angle= M_PI/6;
			const float dPosition= 0.065;//0.065; 
			const float affordance =0;
		}error;

		float mu=0.001;
	    StateMatcher(){}

		void initOnes(){
			for (auto i=weights.begin(); i!= weights.end(); i++){
				*i=1.0;
			}
		}

		DistanceVector getDistance(State, State);  //DO NOT TRY TO LEARN DISTRIBUTION

		float sumVector(DistanceVector);

		bool isPerfectMatch(DistanceVector, float endDistance=0); // is this the same state?

		bool isPerfectMatch(State, State, State* src=NULL); //first state: state to find a match for, second state: candidate match

		std::pair<bool, vertexDescriptor> isPerfectMatch(TransitionSystem, vertexDescriptor, Direction, State); //find match amoung vertex out edges
		
		void ICOadjustWeight(DistanceVector, DistanceVector); //simple ICO learning rule

		//std::pair <bool, float> distance_target_s(b2Transform, b2Transform);

		std::pair <bool, vertexDescriptor> soft_match(TransitionSystem&, b2Transform);
	private:

	const float COEFFICIENT_INCREASE_THRESHOLD=0.0;
};

template <class I>
bool check_vector_for(const std::vector <I>& vector, const I& item){
	for (I _item:vector){
		if (_item==item){
			return true;
		}
	}
	return false;
}
#endif