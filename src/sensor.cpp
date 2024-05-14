#include "sensor.h"

bool Pointf::isin(Pointf tl, Pointf br){
	bool result= this->x>tl.x & this->x<br.x & this->y>br.y& this->y<tl.y;
	return result;
}


float length(cv::Point2f const& p){
	return sqrt(pow(p.x,2)+ pow(p.y, 2));
}


float angle(cv::Point2f const& p){
	return atan2(p.y, p.x);
}

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

b2Vec2 getb2Vec2(cv::Point2f p){
	return b2Vec2(p.x,p.y);

}

Pointf getPointf(b2Vec2 v){
	return Pointf(v.x, v.y);
}

// template <typename T>
// cv::Point2f getPoint2f(T p){
// 	cv::Point2f result(p.x, p.y);
// 	return result;
// }

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
// std::vector<cv::Point2f> set2vec_cv(std::set<T> s){
//     std::vector <cv::Point2f> vec;
//     for (T t:s){
//         vec.emplace_back(getPoint2f(t));
//     }
//     return vec;
// }

// template <typename T>
// std::set<T> vec2set(std::vector<T> vec){
//     std::set <T> set;
//     for (T t:vec){
//         set.emplace_back(t);
//     }
//     return set;
// }





b2Transform PointCloudProc::affineTransEstimate(std::vector <Pointf> current, Task::Action a,float timeElapsed, float range){
        b2Transform result;
        std::vector <Pointf> previousTmp = previous;
		if (previousTmp.empty() || current.empty() || previousTmp==current){
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

std::vector <Pointf> PointCloudProc::neighbours(b2Vec2 pos, float radius, std::vector <Pointf> data){ //more accurate orientation
	std::vector <Pointf> result= std::vector<Pointf>();
	//cv::Rect2f rect(pos.x-radius, pos.y-radius, radius*2, radius*2);//tl, br, w, h
	float x1=pos.x-radius, x2=pos.x+radius, y1=pos.y-radius, y2=pos.y+radius;
	Pointf br(std::max(x1, x2), std::min(y1, y2));
	Pointf tl(std::min(x1, x2), std::max(y1, y2));
	if (data.empty()){
		data = previous;
	}
	for (Pointf p: data){
		if (p.isin(tl, br) & p!=getPointf(pos)){
			result.push_back(p);
		}
	}
	// if (!result.empty()){
	// 	CompareY compareY;
	// 	std::sort(result.begin(), result.end(), compareY);
	// }
	return result;
}

std::pair <bool, b2Vec2> PointCloudProc::findOrientation(std::vector<Pointf> vec){
	std::pair <bool, b2Vec2>result(false, b2Vec2());
	if (vec.size()<6){
		return result;
	}
	int count=0;
	float sumY=0, sumX=0;
	b2Vec2 avgVec;
	CompareY compareY;
	std::vector <Pointf> vec_copy(vec);
	//std::sort(vec.begin(), vec.end(), compareY);
	//for (int i=0; i<vec.size()-1; i++){
	Pointf p;
	std::vector<Pointf>::iterator pIt=vec.end();
	while(!vec.empty()){
		result.first=true;
		pIt =std::min_element(vec.begin(), vec.end(), compareY);
		p=*(pIt);
		vec_copy.push_back(p);
		vec.erase(pIt);
		auto pItNext=std::min_element(vec.begin(), vec.end(), compareY);
		Pointf p_next=*pItNext;
		// Pointf p=vec[i];
		// Pointf p_next = vec[i+1];
		float deltaY =p_next.y- p.y;
		float deltaX = p_next.x - p.x;
		count+=1;
		sumY+=deltaY;
		sumX+=deltaX;
	}
	//if (count>0){
	avgVec.y = sumY/count;
	avgVec.x = sumX/count;
	avgVec.Normalize();
	result.second=avgVec;
	return result;
}


std::pair <bool, cv::Vec4f> PointCloudProc::findOrientationCV(std::vector<Pointf> vec){
	std::pair <bool, cv::Vec4f>result(false, 0);
	if (vec.size()<6){
		return result;
	}
	result.first=true;
	cv::Vec4f line; //vx, vy, x0, y0 -> (vx, vy) normalised collinear vector 
							    // -> (x0, y0) a point on the line
	cv::fitLine(vec, line, cv::DIST_L2, 0, 0.1, 0.1);
	result.second=line;
	//result.second=atan(line[1]/ line[0]);
	return result;
}

std::vector<Pointf> PointCloudProc::setDisturbanceOrientation(Disturbance& d, CoordinateContainer data){
	std::vector <Pointf> v;
	if (!data.empty()){
		v=set2vec(data);
	}
	else{
		//v=previous;
		v=std::vector<Pointf>(previous);
	}
	std::vector <Pointf> nb=std::vector<Pointf>(neighbours(d.getPosition(), NEIGHBOURHOOD,v));
	//cv::Rect2f rect =worldBuilder.getRect(nb);
	//std::pair<bool, cv::Vec4f> orientation =findOrientationCV(nb);
	std::pair<bool, b2Vec2> orientation =findOrientation(nb);	
	float dtheta=0;
	if (orientation.first){
//		d.setOrientation(orientation.second[1], orientation.second[0]);
		d.setOrientation(orientation.second.y, orientation.second.x);
	}
	return nb;

}



cv::Mat ImgProc::cropLeft(cv::Mat mat){
		float w=mat.size().width;
		float h=mat.size().height;
		cv::Mat result=mat(cv::Range(0, h), cv::Range(0, w/2));
        return result;
}

cv::Mat ImgProc::cropRight(cv::Mat mat){
		float w=mat.size().width;
		float h=mat.size().height;
		cv::Mat result=mat(cv::Range(0, h), cv::Range(w/2, w));
        return result;
}

b2Vec2 ImgProc::opticFlow(const cv::Mat& frame, std::vector <cv::Point2f>& corners, cv::Mat& previousFrame_grey){
	    b2Vec2 optic_flow;
		cv::Mat frame_grey;
        std::vector <cv::Point2f> new_corners;
        std::vector <uchar> status;
        std::vector<float> err;
        cv::cvtColor(frame, frame_grey, cv::COLOR_RGB2GRAY);
        if (it%60==0){ //resample corners every 2 seconds (30fps)
            //corners.clear();
            cv::goodFeaturesToTrack(frame_grey, corners, gfp.MAX_CORNERS, gfp.QUALITY_LEVEL, gfp.MIN_DISTANCE);
            printf("GFT, corners size=%i\n", corners.size());
        }
        if (!corners.empty()){
            cv::calcOpticalFlowPyrLK(previousFrame_grey, frame_grey, corners, new_corners, status, err); //no flags: error is L1 distance between points /tot pixels
            printf("LK\n");
        }
        else{
            status=std::vector<uchar>(corners.size(), 1);
        }

        std::vector <cv::Point2f> good_corners;
        //if (it==1){
        int i=0;
        printf("pre-fill in status, new corners size =%i\n", new_corners.size());
        for (i; i<new_corners.size();i++){
            if (status[i]==1){
                good_corners.push_back(new_corners[i]); //og corners
            }
			//float RADIUS=5;
            //cv::circle(frame, corners[i], RADIUS, cv::Scalar(0,0,255));
        }
        printf("good corners = %i, new corners %i\n", good_corners.size(),i);
        if (!corners.empty()&!new_corners.empty()){ //corners are ordered from strongest to weakest
           	optic_flow.x=corners[0].x-new_corners[0].x;
            optic_flow.y=corners[0].y-new_corners[0].y;
			
        }

        printf("updated %i\n", it);
        previousFrame_grey=frame_grey.clone();
        corners=good_corners;
        it++;
		return optic_flow;

}

std::vector <cv::Point2f> ImgProc::corners(){
	return corners_left;
}

cv::Mat ImgProc::previous(){
	return previous_grey_left;
}