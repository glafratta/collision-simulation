#include "general.h"

void Node::fill(Task::simResult result){
	if (result.collision.isValid()){
		totDs++;
	}
	disturbance = result.collision;
	endPose = result.endPose;
	//distanceSoFar = g[srcVertex].distanceSoFar + (round(result.distanceCovered*100))/100; //rounding to 2 decimals to eliminate floating point errors
	outcome = result.resultCode;
}

bool operator!=(Transform const &t1, Transform const& t2){
	//printf("%f != %f = %i\t %f != %f = %i\t %f != %f = %i\n",t1.p.x, t2.p.x, t1.p.x != t2.p.x,t1.p.y, t2.p.y, t1.p.y != t2.p.y, t1.q.GetAngle(), t2.q.GetAngle(), t1.q.GetAngle() != t2.q.GetAngle() );
	return t1.p.x != t2.p.x || t1.p.y != t2.p.y || t1.q.GetAngle() != t2.q.GetAngle();
}

bool operator==(Transform const &t1, Transform const& t2){
	//printf("%f != %f = %i\t %f != %f = %i\t %f != %f = %i\n",t1.p.x, t2.p.x, t1.p.x != t2.p.x,t1.p.y, t2.p.y, t1.p.y != t2.p.y, t1.q.GetAngle(), t2.q.GetAngle(), t1.q.GetAngle() != t2.q.GetAngle() );
	return (t1.p.x == t2.p.x) && (t1.p.y == t2.p.y) && (t1.q.GetAngle() == t2.q.GetAngle());
}

bool operator<(P const & p1, P const &p2){
	return std::tie(p1.r, p1.phi)< std::tie(p2.r, p2.phi);
}

bool operator>(P const &p1, P const & p2){
	return p2<p1;

}

bool operator==(P const &p1, P const & p2){
	return (p1.x == p2.x && p1.y == p2.y);
}
