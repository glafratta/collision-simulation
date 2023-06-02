#include "general.h"

bool operator!=(Transform const &t1, Transform const& t2){
	//printf("%f != %f = %i\t %f != %f = %i\t %f != %f = %i\n",t1.p.x, t2.p.x, t1.p.x != t2.p.x,t1.p.y, t2.p.y, t1.p.y != t2.p.y, t1.q.GetAngle(), t2.q.GetAngle(), t1.q.GetAngle() != t2.q.GetAngle() );
	return t1.p.x != t2.p.x || t1.p.y != t2.p.y || t1.q.GetAngle() != t2.q.GetAngle();
}

// V operator-(V const & v1, V const & v2){
// 	V v;
// 	v.x = v1.x - v2.x;
// 	v.y = v1.y - v2.y;
// 	return v;
// }


// P operator+=(P const & p1, P const & p2){
// 	P result;
// 	result.x = p.
// }

// P& P::operator-(P const & p){
// 	P result;
// 	result.x = this->x-p.x;
// 	result.y = this->y-p.y;
// 	return result;
// }

bool operator<(P const & p1, P const &p2){
	// bool ret=0;
	// if (p1.r< p2.r){
	// 	ret= true;
	// }
	// else if (p1.r == p2.r & p1.phi<p2.phi){
	// 	ret = true;
	// }
	// return ret;
	return std::tie(p1.r, p1.phi)< std::tie(p2.r, p2.phi);
}

bool operator>(P const &p1, P const & p2){
	return p2<p1;

}
