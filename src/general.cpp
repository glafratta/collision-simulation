#include "general.h"


void Node::fill(simResult result){
	if (result.collision.isValid()){
		totDs++;
	}
	disturbance = result.collision;
	endPose = result.endPose;
	//distanceSoFar = g[srcVertex].distanceSoFar + (round(result.distanceCovered*100))/100; //rounding to 2 decimals to eliminate floating point errors
	outcome = result.resultCode;
	step = std::floor(result.step/(HZ*MOTOR_CALLBACK)+0.5);
	filled=true;
}

float Node::evaluationFunction(float weight){ //h(n) = error, cost is the n of D
	return abs(error) +abs(cost);
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
	return std::tie(p1.phi, p1.r)< std::tie(p2.phi, p2.r);
}

bool operator>(P const &p1, P const & p2){
	return p2<p1;

}

bool operator==(P const &p1, P const & p2){
	return (p1.x == p2.x && p1.y == p2.y);
}
