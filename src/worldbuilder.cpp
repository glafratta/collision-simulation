#include "worldbuilder.h"

std::pair<Pointf, Pointf> WorldBuilder::bounds(Direction d, b2Transform start, float boxLength, float halfWindowWidth){
    //float halfWindowWidth=0.15; //wa .1
    std::pair <Pointf, Pointf>result;
    if (d ==LEFT || d==RIGHT){
        boxLength =ROBOT_HALFLENGTH -ROBOT_BOX_OFFSET_X; //og 16 cm
        result.first =Pointf(start.p.x-boxLength, start.p.y-boxLength);
        result.second =Pointf(start.p.x+boxLength, start.p.y+boxLength);
    }
    else{
        Pointf positionVector, radiusVector, maxFromStart, top, bottom; 
        std::vector <Pointf> bounds;
        radiusVector = Polar2f(boxLength, start.q.GetAngle());
        maxFromStart = Pointf(start.p.x, start.p.y) + radiusVector;
        //FIND THE BOUNDS OF THE BOX
        b2Vec2 unitPerpR(-sin(start.q.GetAngle()), cos(start.q.GetAngle()));
        b2Vec2 unitPerpL(sin(start.q.GetAngle()), -cos(start.q.GetAngle()));
        bounds.push_back(Pointf(start.p.x, start.p.y)+Pointf(unitPerpL.x *halfWindowWidth, unitPerpL.y*halfWindowWidth));
        bounds.push_back(Pointf(start.p.x, start.p.y)+Pointf(unitPerpR.x *halfWindowWidth, unitPerpR.y*halfWindowWidth));
        bounds.push_back(maxFromStart+Pointf(unitPerpL.x *halfWindowWidth, unitPerpL.y*halfWindowWidth)); 
        bounds.push_back(maxFromStart+Pointf(unitPerpR.x *halfWindowWidth, unitPerpR.y*halfWindowWidth));
        CompareY compareY;
        std::sort(bounds.begin(), bounds.end(), compareY); //sort bottom to top
        result.first = bounds[0]; //bottom 
        result.second = bounds[3]; //top  
    }
    return result;
    }

// template <typename Pt>
// std::pair<bool,BodyFeatures> WorldBuilder::getOneFeature(std::vector <Pt>nb){//gets bounding box of points
//     float  l=(0.0005*2), w=(0.0005*2) ;
//     float x_glob=0.0f, y_glob=0.0f;
//     // cv::Rect2f rect(x_loc,y_loc,w, h);
//     // b2Transform pose;
//     std::pair <bool, BodyFeatures> result(0, BodyFeatures());
//     if (nb.empty()){
//         return result;
//     }
//     CompareX compareX;
//     CompareY compareY;
//     //Pointf maxx, minx, miny, maxy;
// 	typename std::vector<Pt>::iterator maxx=std::max_element(nb.begin(), nb.end(), compareX);
// 	typename std::vector<Pt>::iterator miny=std::min_element(nb.begin(), nb.end(), compareY);
// 	typename std::vector<Pt>::iterator minx=std::min_element(nb.begin(), nb.end(), compareX);
// 	typename std::vector<Pt>::iterator maxy=std::max_element(nb.begin(), nb.end(), compareY);
//     if (minx->x!=maxx->x){
//         w= fabs((*maxx).x-(*minx).x);
//     }
//     if (miny->y!=maxy->y){
//         l=fabs((*maxy).y-(*miny).y);
//     }
//     x_glob= ((*maxx).x+(*minx).x)/2;
//     y_glob= ((*maxy).y+(*miny).y)/2;
//     result.second.halfLength=l/2;
//     result.second.halfWidth=w/2;
//     result.second.pose.p=b2Vec2(x_glob, y_glob);
//     result.first=true;
// }

std::vector <std::vector<cv::Point2f>> WorldBuilder::feature_clusters( std::vector <cv::Point2f> points,std::vector <cv::Point2f> &centers){
    
    const int MAX_CLUSTERS=3, attempts =3, flags=cv::KMEANS_PP_CENTERS;
    cv::Mat bestLabels;
    cv::TermCriteria termCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0);
    cv::kmeans(points, MAX_CLUSTERS, bestLabels, termCriteria, attempts, flags, centers);
    std::vector <std::vector<cv::Point2f>>result(centers.size());

    for (int i=0; i<bestLabels.rows; i++){ //bestlabel[i] gives the index
            auto index=bestLabels.at<int>(i, 0);
            result[index].push_back(points[i]);
    }
    return result;
}

std::vector <BodyFeatures> WorldBuilder::processData_kmeans( CoordinateContainer pts, const b2Transform& start){
    std::vector <BodyFeatures> result;
    std::vector <cv::Point2f> points, centers;
    for (Pointf p:pts){
        points.push_back(cv::Point2f(float(p.x), float(p.y)));
    }
    std::vector<std::vector<cv::Point2f>> clusters=feature_clusters(points, centers);
    for (int c=0; c<clusters.size(); c++){
        if (std::pair<bool,BodyFeatures>feature=getOneFeature(clusters[c]); feature.first){
            feature.second.pose.q.Set(start.q.GetAngle());
            result.push_back(feature.second);
        }
    }
    return result;

}

void WorldBuilder::makeBody(b2World&w, BodyFeatures features){
	b2Body * body;
	b2BodyDef bodyDef;
	b2FixtureDef fixtureDef;
	bodyDef.type = features.bodyType;	
    bodyDef.position.Set(features.pose.p.x, features.pose.p.y); 
	body = w.CreateBody(&bodyDef);
    switch(features.shape){
        case b2Shape::e_polygon: {
            b2PolygonShape fixture; 
            fixtureDef.shape = &fixture;             
            fixture.SetAsBox(features.halfWidth, features.halfLength); 
            body->CreateFixture(fixtureDef.shape, features.shift);
            break;
        }
        case b2Shape::e_edge:{ //straight edge
            b2EdgeShape fixture; 
            fixtureDef.shape = &fixture; 
            fixture.m_vertex1 =features.pose.p - b2Vec2(features.halfLength*features.pose.q.c, features.halfWidth*features.pose.q.s);
            fixture.m_vertex2 =features.pose.p + b2Vec2(features.halfLength*features.pose.q.c, features.halfWidth*features.pose.q.s);
	        body->CreateFixture(fixtureDef.shape, features.shift);
            break;
        }
        case b2Shape::e_circle:{
            b2CircleShape fixture;
            fixtureDef.shape = &fixture; 
            fixture.m_radius = features.halfLength;
	        body->CreateFixture(fixtureDef.shape, features.shift);
            break;
        }
        default:
        throw std::invalid_argument("not a valid shape\n");break;
    }
    if (features.attention){
        body->GetUserData().pointer=reinterpret_cast<uintptr_t>(DISTURBANCE_FLAG);
    }

	bodies++;
}

std::pair <CoordinateContainer, bool> WorldBuilder::salientPoints(b2Transform start, CoordinateContainer current, std::pair<Pointf, Pointf> bt){
    std::pair <CoordinateContainer, bool> result(CoordinateContainer(), 0);
    float qBottomH=0, qTopH=0, qBottomP=0, qTopP=0, mHead=0, mPerp=0, ceilingY=0, floorY=0, frontX=0, backX=0;
    if (sin(start.q.GetAngle())!=0 && cos(start.q.GetAngle())!=0){
        //FIND PARAMETERS OF THE LINES CONNECTING THE a
        mHead = sin(start.q.GetAngle())/cos(start.q.GetAngle()); //slope of heading direction
        mPerp = -1/mHead;
        qBottomH = bt.first.y - mHead*bt.first.x;
        qTopH = bt.second.y - mHead*bt.second.x;
        qBottomP = bt.first.y -mPerp*bt.first.x;
        qTopP = bt.second.y - mPerp*bt.second.x;
        for (Pointf p: current){
            ceilingY = mHead*p.x +qTopH;
			floorY = mHead*p.x+qBottomH;
			float frontY= mPerp*p.x+qBottomP;
			float backY = mPerp*p.x+qTopP;
            if (p.y >=floorY && p.y<=ceilingY && p.y >=frontY && p.y<=backY){
                result.first.insert(p);
            }
            // if (checkDisturbance(p, result.second, curr) & NULL != dCloud){
            //     dCloud->insert(p);
            // }
        }

    }
    else{
        ceilingY = std::max(bt.second.y, bt.first.y); 
        floorY = std::min(bt.second.y, bt.first.y); 
        frontX = std::min(bt.second.x, bt.first.x);
        backX = std::max(bt.second.x, bt.first.x);
        for (Pointf p: current){
            if (p.y >=floorY && p.y<=ceilingY && p.x >=frontX && p.x<=backX){
                result.first.insert(p);
            }
            // if (checkDisturbance(p, result.second, curr) & NULL != dCloud){
            //     dCloud->insert(p);
            // }      
        }
    }
    return result;
}


std::vector <BodyFeatures> WorldBuilder::getFeatures(CoordinateContainer current, b2Transform start, Direction d, float boxLength, float halfWindowWidth, bool kmeans){
    std::vector <BodyFeatures> features;
    std::pair<Pointf, Pointf> bt = bounds(d, start, boxLength, halfWindowWidth);
    std::pair <CoordinateContainer, bool> salient = salientPoints(start,current, bt);
    if (salient.first.empty()){
        return features;
    }
    if (!kmeans){
        features =processData(salient.first, start);
    }
    else{
        features=processData_kmeans(salient.first, start);
    }
    return features;
}



 std::vector <BodyFeatures> WorldBuilder::buildWorld(b2World& world,CoordinateContainer current, b2Transform start, Direction d, Disturbance disturbance, float halfWindowWidth, bool kmeans){
  //  std::pair<bool, b2Vec2> result(0, b2Vec2(0,0));
    float boxLength=simulationStep-ROBOT_BOX_OFFSET_X;
    std::vector <BodyFeatures> features=getFeatures(current, start, d, boxLength, halfWindowWidth);
    // if (occluded(current, disturbance)){
    //     salient.first.emplace(getPointf(disturbance.getPosition()));
    //     features.push_back(disturbance.bodyFeatures());
    // }
    bool has_D=false;
    if (disturbance.getAffIndex()==AVOID && d==DEFAULT){
        for (BodyFeatures f: features){
            if (f.match(disturbance.bf)){
                has_D=true;
                f.attention=true;
            }
        }
        if (!has_D){
            features.push_back(disturbance.bf);
            
        }
    }
    for (BodyFeatures f: features){
        makeBody(world, f);
    }
	FILE *file;
	if (debug){
		file = fopen(bodyFile, "a+");
		for (b2Body * b = world.GetBodyList(); b!=NULL; b= b->GetNext()){
			fprintf(file, "%f\t%f\n", b->GetPosition().x, b->GetPosition().y);
		}
		fclose(file);
	}
    //result.first = salient.second;
    return features;
   // return result;
}

bool WorldBuilder::checkDisturbance(Pointf p, bool& obStillThere, Task * curr, float range){
    bool result=0;
	if (NULL!=curr){ //
        if (!curr->disturbance.isValid()){
            return result;
        }
        cv::Rect2f rect(curr->disturbance.getPosition().x-range, curr->disturbance.getPosition().y+range, range*2, range*2);
		if (p.inside(rect)){
			obStillThere =1;
            result =1;
		}
	}
    return result;
}

b2Vec2 averagePoint(CoordinateContainer c, Disturbance & d, float rad = 0.025){
    b2Vec2 result(0,0), centroid(0,0);
    for (Pointf p: c){
       centroid.x+=p.x;
       centroid.y +=p.y; 
    }
    centroid.x/=c.size();
    centroid.y/=c.size();
    result = d.getPosition()- centroid;
    d.setPosition(centroid);
    return result;
}

bool WorldBuilder::occluded(CoordinateContainer cc, Disturbance expectedD){
    bool result=false;
    if (!expectedD.isValid()){
        return result;
    }
    cv::Rect2f rect;
    std::vector <Pointf> occluding; 
    for (Pointf p:cc){
        Pointf tl(expectedD.getPosition().x-expectedD.halfWidth(), expectedD.getPosition().y+expectedD.halfWidth());
        Pointf br(expectedD.getPosition().x+expectedD.halfWidth(), expectedD.getPosition().y-expectedD.halfWidth());
        // rect=cv::Rect2f(expectedD.getPosition().x+expectedD.halfWidth(), 
        //                 expectedD.getPosition().y-expectedD.halfWidth(), 
        //                 expectedD.getPosition().x+expectedD.halfWidth(), 
        //                 expectedD.halfLength()*2);
        // if (p.inside(rect)){
        if (p.isin(tl, br)){
            occluding.push_back(p);
        }
    }
    if (occluding.empty()){
        return result;
    }
    CompareY compareY;
    std::vector<Pointf>::iterator miny=std::min_element(occluding.begin(), occluding.end(), compareY);
    std::vector<Pointf>::iterator maxy=std::max_element(occluding.begin(), occluding.end(), compareY);
    float length = (*maxy).y-(*miny).y;
    if (length>=rect.height*0.75 & occluding.size()>= rect.height*75){
        result=true;
    }
    //to finish
    return result;
}