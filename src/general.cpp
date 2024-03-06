#include "general.h"

void State::fill(simResult result){
	if (result.collision.isValid()){
		totDs++;
	}
	disturbance = result.collision;
	endPose = result.endPose;
	outcome = result.resultCode;
	step = std::floor(result.step/(HZ*MOTOR_CALLBACK)+0.5);
	//nObs++;
	filled=true;
}

simResult State::getSimResult(){
	simResult result;
	result.collision= disturbance;
	result.endPose=endPose;
	result.step= step;
	result.valid = filled;
	result.resultCode=outcome;
	return result;
}

void State::set(State tmp){
	if (tmp.disturbance.isValid()& tmp.disturbance.getAffIndex()==AVOID){
		totDs++;
	}
	disturbance = tmp.disturbance;
	endPose = tmp.endPose;
	outcome = tmp.outcome;
	step = std::floor(tmp.step/(HZ*MOTOR_CALLBACK)+0.5);
	options = tmp.options;
	//nObs++;
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
	bool positionMatch = b2Vec2(vec[3], vec[4]).Length()<SDvector[4];
	bool angleMatch = fabs(vec[5])<SDvector[5];
	bool disturbanceMatch =b2Vec2(vec[0], vec[1]).Length()<SDvector[0];
	bool affordanceMatch =vec[2]==SDvector[2];
    if (positionMatch &&  disturbanceMatch&& affordanceMatch &&angleMatch){ //match position and disturbance
        result=true;
    }
    return result;
}

bool StateMatcher::isPerfectMatch(State s1, State s2){
	DistanceVector  distance = getDistance(s1, s2);
    return isPerfectMatch(distance);
}


std::pair<bool, vertexDescriptor> StateMatcher::isPerfectMatch(CollisionGraph g, vertexDescriptor src, Direction d, State s){
    std::pair<bool, vertexDescriptor> result(false, -1);
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

// void StateMatcher::match(State tmp, State& state){
// 	int nObs = state.nObs;
// 	state=tmp;
// 	state.nObs= nObs+1;
// }

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

bool operator<(P const & p1, P const &p2){
	return std::tie(p1.phi, p1.r)< std::tie(p2.phi, p2.r);
}

bool operator>(P const &p1, P const & p2){
	return p2<p1;

}

bool operator==(P const &p1, P const & p2){
	return (p1.x == p2.x && p1.y == p2.y);
}

void operator-=(Transform & t1, Transform const&t2){
	t1.p.x-=t2.p.x;
	t1.p.y-=t2.p.y;
	t1.q.Set(t1.q.GetAngle()-t2.q.GetAngle());
}
