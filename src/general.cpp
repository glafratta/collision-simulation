#include "general.h"

void State::fill(simResult result){
	if (result.collision.isValid()){
		totDs++;
	}
	disturbance = result.collision;
	endPose = result.endPose;
	outcome = result.resultCode;
	step = std::floor(result.step/(HZ*MOTOR_CALLBACK)+0.5);
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

DistanceVector StateMatcher::getDistance(State s1, State s2){
	DistanceVector result;
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
    if (b2Vec2(vec[4], vec[5]).Length()<SDvector[4] && b2Vec2(vec[0], vec[1]).Length()<SDvector[0] && vec[3]==SDvector[3]){ //match position and disturbance
        result=true;
    }
    return result;
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
