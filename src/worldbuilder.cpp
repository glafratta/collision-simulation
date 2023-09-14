#include "worldbuilder.h"

std::pair<Point, Point> WorldBuilder::bounds(Direction d, b2Transform start, float boxLength,Task* curr ){
    float halfWindowWidth=0.1;
    std::pair <Point, Point> result;
    if (d !=DEFAULT & d!=BACK){
        boxLength =ROBOT_HALFLENGTH -ROBOT_BOX_OFFSET_X; //og 16 cm
        result.first =Point(start.p.x-boxLength, start.p.y-boxLength);
        result.first =Point(start.p.x+boxLength, start.p.y+boxLength);
    }
    else{
        Point positionVector, radiusVector, maxFromStart, top, bottom; 
        std::vector <Point> bounds;
        radiusVector.polarInit(boxLength, start.q.GetAngle());
        //printf("radius vector = x=%f, y=%f\n", radiusVector.x, radiusVector.y);
        maxFromStart = Point(start.p) + radiusVector;
        //printf("max from start length = %f\n", maxFromStart.r);
        //FIND THE BOUNDS OF THE BOX
        b2Vec2 unitPerpR(-sin(start.q.GetAngle()), cos(start.q.GetAngle()));
        b2Vec2 unitPerpL(sin(start.q.GetAngle()), -cos(start.q.GetAngle()));
        bounds.push_back(Point(start.p)+Point(unitPerpL.x *halfWindowWidth, unitPerpL.y*halfWindowWidth));
        bounds.push_back(Point(start.p)+Point(unitPerpR.x *halfWindowWidth, unitPerpR.y*halfWindowWidth));
        bounds.push_back(maxFromStart+Point(unitPerpL.x *halfWindowWidth, unitPerpL.y*halfWindowWidth)); 
        bounds.push_back(maxFromStart+Point(unitPerpR.x *halfWindowWidth, unitPerpR.y*halfWindowWidth));
        comparator compPoint;
        std::sort(bounds.begin(), bounds.end(), compPoint); //sort bottom to top
        result.first = bounds[0]; //bottom
        result.second = bounds[3]; //top  
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
            fixture.SetAsBox(features.halfLength, features.halfWidth); 
	        body->CreateFixture(fixtureDef.shape, 0.0f);
            break;
        }
        case b2Shape::e_edge:{ //straight edge
            b2EdgeShape fixture; 
            fixtureDef.shape = &fixture; 
            fixture.m_vertex1 =features.pose.p - b2Vec2(features.halfLength*features.pose.q.c, features.halfWidth*features.pose.q.s);
            fixture.m_vertex2 =features.pose.p + b2Vec2(features.halfLength*features.pose.q.c, features.halfWidth*features.pose.q.s);
	        body->CreateFixture(fixtureDef.shape, 0.0f);
            break;
        }
        case b2Shape::e_circle:{
            b2CircleShape fixture;
            fixtureDef.shape = &fixture; 
            fixture.m_radius = features.halfLength;
	        body->CreateFixture(fixtureDef.shape, 0.0f);
            break;
        }
        default:
        throw std::invalid_argument("not a valid shape\n");break;
    }
	//b2PolygonShape fixture; //giving the point the shape of a box

	bodies++;
	// body->CreateFixture(fixtureDef.shape, 0.0f);
}

std::pair <CoordinateContainer, bool> WorldBuilder::salientPoints(b2Transform start, CoordinateContainer current, std::pair <Point, Point> bt, Task *curr){
    std::pair <CoordinateContainer, bool> result(CoordinateContainer(), 0);
    CoordinateContainer dCloud;
    float qBottomH=0, qTopH=0, qBottomP=0, qTopP=0, mHead=0, mPerp=0, ceilingY=0, floorY=0, frontX=0, backX=0;
    if (sin(start.q.GetAngle())!=0 && cos(start.q.GetAngle())!=0){
        //FIND PARAMETERS OF THE LINES CONNECTING THE a
        mHead = sin(start.q.GetAngle())/cos(start.q.GetAngle()); //slope of heading direction
        mPerp = -1/mHead;
        qBottomH = bt.first.y - mHead*bt.first.x;
        qTopH = bt.second.y - mHead*bt.second.x;
        qBottomP = bt.first.y -mPerp*bt.first.x;
        qTopP = bt.second.y - mPerp*bt.second.x;
        for (Point p: current){
            ceilingY = mHead*p.x +qTopH;
			floorY = mHead*p.x+qBottomH;
			float frontY= mPerp*p.x+qBottomP;
			float backY = mPerp*p.x+qTopP;
            if (p.y >=floorY && p.y<=ceilingY && p.y >=frontY && p.y<=backY){
                result.first.insert(p);
            }
            if (checkDisturbance(p, result.second, curr)){
                dCloud.insert(p);
            }
        }

    }
    else{
        ceilingY = std::max(bt.second.y, bt.first.y); 
        floorY = std::min(bt.second.y, bt.first.y); 
        frontX = std::min(bt.second.x, bt.first.x);
        backX = std::max(bt.second.x, bt.first.x);
        for (Point p: current){
            if (p.y >=floorY && p.y<=ceilingY && p.x >=frontX && p.x<=backX){
                result.first.insert(p);
            }
            if (checkDisturbance(p, result.second, curr)){
                dCloud.insert(p);
            }        
        }
    }
    return result;
}

  std::pair<bool, b2Vec2> WorldBuilder::buildWorld(b2World& world,CoordinateContainer current, b2Transform start, Direction d, Task*curr, bool discrete){
    //sprintf(bodyFile, "/tmp/bodies%04i.txt", iteration);
    // if (iteration=0){
    //     FILE *f = fopen(bodyFile, "w+");
    //     fclose(f);
    // }
    std::pair<bool, b2Vec2> result(0, b2Vec2(0,0));
    float boxLength=BOX2DRANGE;
    if (discrete){
        boxLength = DISCRETE_RANGE;
    }
    if(d==BACK){
        float x = start.p.x - (SAFE_DISTANCE+ ROBOT_HALFLENGTH)* cos(start.q.GetAngle());
        float y = start.p.y - (SAFE_DISTANCE+ROBOT_HALFLENGTH)* sin(start.q.GetAngle());
        start = b2Transform(b2Vec2(x, y), b2Rot(start.q.GetAngle()));
    }
    std::pair <Point, Point> bt = bounds(d, start, boxLength, curr);
    std::pair <CoordinateContainer, bool> salient = salientPoints(start,current, bt, curr);
    // if (NULL!=curr){
    //     result.second = averagePoint(&&&.first, curr->disturbance);
    // }
    std::vector <BodyFeatures> features =processData(salient.first);
    for (BodyFeatures f: features){
        makeBody(world, f);
    }
	FILE *f;
	if (debug){
		f = fopen(bodyFile, "a+");
		for (b2Body * b = world.GetBodyList(); b!=NULL; b= b->GetNext()){
			fprintf(f, "%f\t%f\n", b->GetPosition().x, b->GetPosition().y);
		}
		fclose(f);
	}
    result.first = salient.second;
    return result;
}

bool WorldBuilder::checkDisturbance(Point p, bool& obStillThere, Task * curr){
    bool result=0;
	if (NULL!=curr){ //
		if (p.isInRadius(curr->disturbance.getPosition())){
			obStillThere =1;
            result =1;
		}
	}
    return result;
}

b2Vec2 averagePoint(CoordinateContainer c, Disturbance & d, float rad = 0.025){
    b2Vec2 result(0,0), centroid(0,0);
    for (Point p: c){
       centroid.x+=p.x;
       centroid.y +=p.y; 
    }
    centroid.x/=c.size();
    centroid.y/=c.size();
    result = d.getPosition()- centroid;
    d.setPosition(centroid);
    return result;
}
    
// std::vector <BodyFeatures> AlternateBuilder::processData(CoordinateContainer points){
//     int count =0;
//     std::vector <BodyFeatures> result;
//     for (Point p: points){
//         if (count%2==0){
//             BodyFeatures feature;
//             feature.pose.p = p.getb2Vec2(); 
//             result.push_back(feature);  
//         }
//         count++;
//     }
//     return result;
// }
