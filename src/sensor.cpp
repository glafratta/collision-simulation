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



b2Transform PointCloudProc::affineTransEstimate(std::vector <Pointf> current, Task::Action a,float timeElapsed, float range){
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

std::vector <Pointf> PointCloudProc::neighbours(b2Vec2 pos, float radius){ //more accurate orientation
	std::vector <Pointf> result;
	cv::Rect2f rect(pos.x-radius, pos.y-radius, radius*2, radius*2);//tl, br, w, h
	auto br=rect.br();
	auto tl=rect.tl();
	for (Pointf p: previous){
		if (p.inside(rect) & p!=getPointf(pos)){
			result.push_back(p);
		}
	}
	return result;
}

std::pair <bool, float> PointCloudProc::findOrientation(std::vector<Pointf> vec){
	int count=0;
	float sumY=0, sumX=0;
	float avgY=0, avgX=0;
	std::pair <bool, float>result(false, 0);
	vec.shrink_to_fit();
	for (Pointf p:vec){
	//cv::Rect2f rect(pos.x-radius, pos.y+radius, radius, radius);//tl, br, w, h
	//if (p.inside(rect)){
		std::set <Pointf>set=vec2set(vec);
		auto pIt =set.find(p);
		CoordinateContainer::iterator pItNext = pIt++;
		if (pIt!=set.end()){
			float deltaY =pItNext->y- pIt->y;
			float deltaX = pItNext->x - pIt->x;
			result.first=true; //is there a neighbouring point?
			count+=1;
			sumY+=deltaY;
			sumX+=deltaX;
		}

	//}
	}
	avgY = sumY/count;
	avgX = sumX/count;
	result.second=atan(avgY/avgX);
	return result;
}

std::vector<Pointf> PointCloudProc::setDisturbanceOrientation(Disturbance& d){
	std::vector <Pointf> nb=neighbours(d.getPosition(), NEIGHBOURHOOD);
	//cv::Rect2f rect =worldBuilder.getRect(nb);
	std::pair<bool, float> orientation =findOrientation(nb);
	// result.collision.bf.halfLength=rect.width/2;
	// result.collision.bf.halfLength=rect.height/2;
	d.setOrientation(orientation.second);
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
        //QImage::Format f= QImage::Format_Grayscale8;
        if (it%60==0){ //resample corners every 2 seconds (30fps)
            corners.clear();
            cv::goodFeaturesToTrack(frame_grey, corners, gfp.MAX_CORNERS, gfp.QUALITY_LEVEL, gfp.MIN_DISTANCE);
            printf("GFT\n");
        }
        if (it>0 & !corners.empty()){
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
        for (i; i<corners.size();i++){
            if (status[i]==1){
                good_corners.push_back(new_corners[i]); //og corners
            }
			float RADIUS=5;
            cv::circle(frame, corners[i], RADIUS, cv::Scalar(0,0,255));
        }
        if (!corners.empty()&!new_corners.empty()){ //corners are ordered from strongest to weakest
           	optic_flow.x=corners[0].x-new_corners[0].x;
            optic_flow.y=corners[0].y-new_corners[0].y;
			
        }
        printf("good corners = %i, corners %i\n", good_corners.size(),i);

        printf("updated %i\n", it);
        previousFrame_grey=frame_grey.clone();
        corners=good_corners;
        it++;
		return optic_flow;

}