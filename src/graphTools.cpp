#include "graphTools.h"

std::pair<State, Edge> gt::fill(simResult sr){
	std::pair <State, Edge> result;
	// if (sr.collision.isValid()){
	// 	result.first.totDs++;
	// }
	result.first.disturbance = sr.collision;
	result.first.endPose = sr.endPose;
	result.first.outcome = sr.resultCode;
	result.second.step = gt::simToMotorStep(sr.step);
	//nObs++;
	result.first.filled=true;
	return result;
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

void gt::update(edgeDescriptor e, std::pair <State, Edge> sk, TransitionSystem& g, bool current, std::unordered_map<Edge*, float>& errorMap){
	float result=0;
	if (!current){
		g[e].step = sk.second.step;
	}
	else if (g[e].direction==DEFAULT& g[e.m_target].disturbance.isValid()){
		result=g[e.m_target].disturbance.pose.p.x-sk.first.disturbance.pose.p.x;
		//if (auto it =errorMap.find(&g[e.m_target]); it !=errorMap.end()){
		//	it->second= result;
		//}
		//else{
		errorMap.insert_or_assign(g[e].ID, result);
	//	}
	}
	g[e.m_target].disturbance = sk.first.disturbance;
	g[e.m_target].endPose = sk.first.endPose;
	g[e.m_target].options = sk.first.options;
}

void gt::set(edgeDescriptor e, std::pair <State, Edge> sk, TransitionSystem& g, bool current, std::unordered_map<Edge*, float>& errorMap){
	update(e, sk, g, current, errorMap);
	g[e.m_target].outcome = sk.first.outcome;
}

DistanceVector StateMatcher::getDistance(State s1, State s2){
	DistanceVector result(6);
	result[0]= s1.disturbance.pose.p.x - s2.disturbance.pose.p.x; //disturbance x
	result[1]= s1.disturbance.pose.p.y - s2.disturbance.pose.p.y; //disturbance y
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

bool StateMatcher::isPerfectMatch(DistanceVector vec){
    bool result =false;
	bool positionMatch = b2Vec2(vec[3], vec[4]).Length()<error.endPosition;
	bool angleMatch = fabs(vec[5])<error.angle;
	bool disturbanceMatch =b2Vec2(vec[0], vec[1]).Length()<error.dPosition;
	bool affordanceMatch = vec[2]==error.affordance;
    if (positionMatch &&  disturbanceMatch&& affordanceMatch &&angleMatch){ //match position and disturbance
        result=true;
    }
    return result;
}

bool StateMatcher::isPerfectMatch(State s1, State s2){
	DistanceVector  distance = getDistance(s1, s2);
    return isPerfectMatch(distance);
}


std::pair<bool, vertexDescriptor> StateMatcher::isPerfectMatch(TransitionSystem g, vertexDescriptor src, Direction d, State s){
    std::pair<bool, vertexDescriptor> result(false, TransitionSystem::null_vertex());
	auto edges= boost::out_edges(src, g);
	for (auto ei=edges.first; ei!=edges.second; ++ei){
		if (g[*ei].direction==d & isPerfectMatch(s, g[ei.dereference().m_source])){
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


