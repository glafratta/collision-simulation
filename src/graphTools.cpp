#include "graphTools.h"

void gt::fill(simResult sr, State* s, Edge* e){
	if (NULL!=s){
		s->disturbance = sr.collision;
		s->endPose = sr.endPose;
		s->outcome = sr.resultCode;
		s->filled=true;
	}
	if (NULL!=e){
		e->step = gt::simToMotorStep(sr.step);
	}
}

int gt::simToMotorStep(int simStep){
	return std::floor(simStep/(HZ*MOTOR_CALLBACK)+0.5);
}

// simResult State::getSimResult(){
// 	simResult result;
// 	result.collision= disturbance;
// 	result.endPose=endPose;
// 	//result.step= step;
// 	result.valid = filled;
// 	result.resultCode=outcome;
// 	return result;
// }

void gt::update(edgeDescriptor e, std::pair <State, Edge> sk, TransitionSystem& g, bool current, std::unordered_map<State*, float>& errorMap){
	if (e==edgeDescriptor()){
		return;
	}
	float result=0;
	if (!current){
		g[e].step = sk.second.step;
	}
	else if (g[e.m_target].direction==DEFAULT& g[e.m_target].disturbance.isValid()){
		result=g[e.m_target].disturbance.getPosition().x-sk.first.disturbance.getPosition().x;
		errorMap.insert_or_assign(g[e.m_target].ID, result);
	}
	g[e.m_target].disturbance = sk.first.disturbance;
	if(sk.first.label==g[e.m_target].label){
		g[e.m_target].endPose = sk.first.endPose;
	}
	g[e.m_target].options = sk.first.options;
	g[e.m_target].nObs++;
	if (!g[e.m_target].visited()){
		g[e.m_target].phi=sk.first.phi;
	}
	//adjustProbability(g, e);
}

void gt::set(edgeDescriptor e, std::pair <State, Edge> sk, TransitionSystem& g, bool current, std::unordered_map<State*, float>& errorMap){
	update(e, sk, g, current, errorMap);
	g[e.m_target].outcome = sk.first.outcome;
	//g[e.m_target].label=sk.first.label;
}

std::vector <edgeDescriptor> gt::outEdges(TransitionSystem&g, vertexDescriptor v, Direction d){
	std::vector <edgeDescriptor> result;
	auto es = boost::out_edges(v, g);
	for (auto ei = es.first; ei!=es.second; ++ei){
		if (g[(*ei).m_target].direction == d){
			result.push_back(*ei);
		}
	}
	return result;
}

std::pair< bool, edgeDescriptor> gt::getMostLikely(TransitionSystem& g, std::vector <edgeDescriptor> oe){
	std::pair< bool, edgeDescriptor> mostLikely(false, edgeDescriptor());
	float prob=0;
	for (edgeDescriptor e:oe){
		if (g[e].probability>prob){
			mostLikely.second=e;
			prob=g[e].probability;
		}
	}
	mostLikely.first=!oe.empty();
	return mostLikely;
}


Disturbance gt::getExpectedDisturbance(TransitionSystem& g, vertexDescriptor v, Direction d){
	std::vector<edgeDescriptor> oe=outEdges(g, v, d);
	Disturbance result=Disturbance();
	if (oe.empty()){
		return result;
	}
	std::pair<bool,edgeDescriptor> mostLikely=getMostLikely(g, oe);
	if (mostLikely.first){
		result=g[mostLikely.second.m_target].disturbance;
	}
	return result;

}
edgeDescriptor gt::visitedEdge(std::vector <edgeDescriptor> es, TransitionSystem& g){
	for (edgeDescriptor e:es){
		if (g[e.m_source].visited() & g[e.m_target].visited()){
			return e;
		}
	}
}


void gt::adjustProbability(TransitionSystem &g, edgeDescriptor e){
	auto es= out_edges(e.m_source, g);
	float totObs=0;
	std::vector <edgeDescriptor> sameTask;
	//find total observations
	for (auto ei= es.first; ei!=es.second; ei++){
		if (g[((*ei).m_target)].direction==g[e.m_target].direction){
			totObs+=g[(*ei).m_target].nObs;
			sameTask.push_back(*ei);
			//g[*ei].probability=g[e.m_target].nObs/g[e.m_source].nObs;
		}
	}
	//adjust
	for (edgeDescriptor ed: sameTask){
		g[ed].probability=g[ed.m_target].nObs/totObs;
	}
}

std::pair <edgeDescriptor, bool> gt::add_edge(vertexDescriptor u, vertexDescriptor v, TransitionSystem& g){
	std::pair <edgeDescriptor, bool> result(edgeDescriptor(), false);
	if (u==v){
		return result;
	}
	result =boost::add_edge(u, v, g);
	return result;
}



DistanceVector StateMatcher::getDistance(State s1, State s2){
	DistanceVector result(6);
	result[0]= s1.disturbance.getPosition().x - s2.disturbance.getPosition().x; //disturbance x
	result[1]= s1.disturbance.getPosition().y - s2.disturbance.getPosition().y; //disturbance y
	result[2]= s1.disturbance.getAffIndex()-s2.disturbance.getAffIndex(); //disturbance type
	result[3]= s1.endPose.p.x-s2.endPose.p.x; //endpose x
	result[4]=s1.endPose.p.y-s2.endPose.p.y; //endpose y
	result[5]=s1.endPose.q.GetAngle()-s2.endPose.q.GetAngle(); //endpose angle
	return result;
}


float StateMatcher::sumVector(DistanceVector vec){
	float result=0;
	for (float i:vec){
		result+=abs(i);
	}
	return result;
}

bool StateMatcher::isPerfectMatch(DistanceVector vec, float endDistance){
	float coefficient=1.0;
	if (endDistance>COEFFICIENT_INCREASE_THRESHOLD){
		coefficient+=(endDistance-COEFFICIENT_INCREASE_THRESHOLD)/(BOX2DRANGE-COEFFICIENT_INCREASE_THRESHOLD);
		//coefficient*=scale;
	}
    bool result =false;
	bool positionMatch = b2Vec2(vec[3], vec[4]).Length()<(error.endPosition*coefficient);
	bool angleMatch = fabs(vec[5])<error.angle;
	bool disturbanceMatch =b2Vec2(vec[0], vec[1]).Length()<(error.dPosition*coefficient);
	bool affordanceMatch = vec[2]==error.affordance;
    if (positionMatch &&  disturbanceMatch&& affordanceMatch &&angleMatch){ //match position and disturbance
        result=true;
    }
    return result;
}
bool StateMatcher::isPerfectMatch(State s, State candidate, State *src){
	DistanceVector  distance = getDistance(s, candidate);
	float stray=0;
	if (src!=NULL & s.label!=UNDEFINED){
		float ds= (src->endPose.p -candidate.endPose.p).Length();
		b2Vec2 ref(src->endPose.p.x+ds*cos(src->endPose.q.GetAngle()), src->endPose.p.y+ ds*sin(src->endPose.q.GetAngle()));
		stray=(s.endPose.p-src->endPose.p).Length();
	}
    return isPerfectMatch(distance, s.endPose.p.Length()) & (stray<error.endPosition|| s.label==candidate.label);
}


std::pair<bool, vertexDescriptor> StateMatcher::isPerfectMatch(TransitionSystem g, vertexDescriptor src, Direction d, State s){
    std::pair<bool, vertexDescriptor> result(false, TransitionSystem::null_vertex());
	auto edges= boost::out_edges(src, g);
	for (auto ei=edges.first; ei!=edges.second; ++ei){
		if (g[(*ei).m_target].direction==d & isPerfectMatch(s, g[ei.dereference().m_source])){
			result.first=true;
			result.second=(*ei).m_target;
			break;
		}
	}
    return result;
}


void StateMatcher::ICOadjustWeight(DistanceVector E, DistanceVector dE){
	for (int i=0; i<weights.size();i++){
		//float weight= weights[i];
		weights[i]+=mu*E[i]*dE[i];
	}
}

bool operator!=(Transform const &t1, Transform const& t2){
	return t1.p.x != t2.p.x || t1.p.y != t2.p.y || t1.q.GetAngle() != t2.q.GetAngle();
}

bool operator==(Transform const &t1, Transform const& t2){
	return (t1.p.x == t2.p.x) && (t1.p.y == t2.p.y) && (t1.q.GetAngle() == t2.q.GetAngle());
}

void operator-=(Transform & t1, Transform const&t2){
	t1.p.x-=t2.p.x;
	t1.p.y-=t2.p.y;
	t1.q.Set(t1.q.GetAngle()-t2.q.GetAngle());
}

