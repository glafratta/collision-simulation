#include "sensor.h"

float length(cv::Point2f const& p){
	return sqrt(pow(p.x,2)+ pow(p.y, 2));
}


float angle(cv::Point2f const& p){
	return atan2(p.y, p.x);
}

b2Vec2 getb2Vec2(cv::Point2f p){
	return b2Vec2(p.x,p.y);

}

Pointf getPointf(b2Vec2 v){
	return Pointf(v.x, v.y);
}

Pointf Polar2f(float radius, float angle){
	float x = radius *cos(angle);
	float y = radius *sin(angle);
	return Pointf(x,y);
}

template <typename T>
std::vector<T> set2vec(std::set<T> s){
    std::vector <T> vec;
    for (T t:s){
        vec.emplace_back(t);
    }
    return vec;
}

// template <typename T>
// std::set<T> vec2set(std::vector<T> vec){
//     std::set <T> set;
//     for (T t:vec){
//         set.emplace_back(t);
//     }
//     return set;
// }

bool operator <(Pointf const & p1, Pointf const& p2){
	float a1 = angle(p1);
	float l1=length(p1);
	float a2=angle(p2);
	float l2=length(p2); 
	return std::tie(a1, l1)< std::tie(a2, l2);
}	

bool operator >(const Pointf& p1,  const Pointf& p2){
	return p2<p1;
}



b2Transform SensorTools::affineTransEstimate(std::vector <Pointf> current, Task::Action a,float timeElapsed, float range){
        b2Transform result;
        std::vector <Pointf> previousTmp = previous;
        previous=current;
		if (previousTmp.empty() || current.empty()){
			return result;
		}
	 	float theta = a.getOmega()* timeElapsed;
		result.p ={a.getLinearSpeed()*cos(theta),a.getLinearSpeed()*sin(theta)};
		result.q.Set(a.getOmega());
		int diff = current.size()-previousTmp.size(); //if +ve,current is bigger, if -ve, previous is bigger

		if(diff>0){
            for (int i=0; i<abs(diff); i++){
					previousTmp.push_back(previousTmp[0]); //before it was [-1]
			}

		}
		else if (diff<0){
         for (int i=0; i<abs(diff); i++){
                current.push_back(current[0]);
            }
		}
	//use partial affine transformation to estimate displacement
	cv::Mat transformMatrix =cv::estimateAffinePartial2D(previousTmp, current, cv::noArray(), cv::LMEDS);
	if (!transformMatrix.empty()){
		result.p.x= -(transformMatrix.at<double>(0,2))/timeElapsed;
		result.p.y = -(transformMatrix.at<double>(1,2))/timeElapsed;
		result.q.Set(acos(transformMatrix.at<double>(0,0))/timeElapsed);
		float posAngle=0; 
        float tan = atan(result.p.y/result.p.x);//atan2 gives results between pi and -pi, atan gives pi/2 to -pi/2
		if (result.p.y !=0 && result.p.x !=0 && tan < MAX_OMEGA*timeElapsed){
			posAngle =tan;
		}
		if (result.p.Length()>MAX_SPEED){
			result.p.x = a.getLinearSpeed() *cos(posAngle);
			result.p.y = a.getLinearSpeed() *sin(posAngle);
		}

	}
	return result;

}