#include "task.h"

b2Fixture * GetSensor(b2Body * body){
	for (b2Fixture * f=body->GetFixtureList(); f;f=f->GetNext()){
		if (f->IsSensor()){
			return f;
		}
	}
	return NULL;
}

b2Body * GetDisturbance(b2World * w){
	for (b2Body * b=w->GetBodyList();b;b=b->GetNext()){
		if (b->GetUserData().pointer==DISTURBANCE_FLAG){
			return b;
		}
	}
	return NULL;
}


bool overlaps(b2Body * robot, b2Body * disturbance){
	b2Fixture * sensor=GetSensor(robot);
	if (sensor==NULL){
		return true;
	}
	if (disturbance==NULL){
		return true;
	}
	b2AABB aabb=sensor->GetAABB(0);
	b2Shape * d=disturbance->GetFixtureList()->GetShape();
	b2Transform robot_pose=robot->GetTransform(), d_pose= disturbance->GetTransform();
	b2AABB aabb_shape, aabb_zero;
	sensor->GetShape()->ComputeAABB(&aabb_shape, robot_pose,0);
	sensor->GetShape()->ComputeAABB(&aabb_shape, b2Transform_zero,0);
	return b2TestOverlap(sensor->GetShape(), 0, d, 0,robot_pose, d_pose);
}

bool overlaps(b2Body * robot, Disturbance * disturbance){
	b2Fixture * sensor=GetSensor(robot);
	if (sensor==NULL){
		return true;
	}
	if (disturbance==NULL || disturbance->getAffIndex()!= AVOID ){
		return true;
	}
	b2AABB aabb=sensor->GetAABB(0);
	// b2Shape * d=disturbance->GetFixtureList()->GetShape();
	b2Transform robot_pose=robot->GetTransform(), d_pose= disturbance->pose();
	b2AABB aabb_shape, aabb_zero, aabb_d;
	sensor->GetShape()->ComputeAABB(&aabb_shape, robot_pose,0);
	sensor->GetShape()->ComputeAABB(&aabb_shape, b2Transform_zero,0);
	b2PolygonShape d_shape;
	d_shape.SetAsBox(disturbance->bf.halfWidth, disturbance->bf.halfLength, b2Vec2(0,0), 0);
	d_shape.ComputeAABB(&aabb_d, disturbance->pose(), 0);
	//create AABB with disturbance vertices
	//test overlap
	return b2TestOverlap(sensor->GetShape(), 0, &d_shape, 0,robot_pose, d_pose);
}

simResult Task::willCollide(b2World & _world, int iteration, b2Body * robot, bool debugOn, float remaining, float simulationStep){ //CLOSED LOOP CONTROL, og return simreult
		simResult result=simResult(simResult::resultType::successful);
		result.endPose = start;
		if (action.L==0 & action.R==0){
			return result;
		}
		Listener listener(&disturbance);
		//Query query(&disturbance);
		b2Body * d_body=GetDisturbance(&_world);
		int _count=_world.GetBodyCount();
		_world.SetContactListener(&listener);	
		FILE * robotPath;
		if (debugOn){
			sprintf(planFile, "/tmp/robot%04i.txt", iteration);
			robotPath = fopen(planFile, "a");
		}
		float theta = start.q.GetAngle();
		b2Vec2 instVelocity = {0,0};
		//robot->SetTransform(start.p, theta);
		//makeRobotSensor(robot.body);
		int stepb2d=0;
		float traj_error=0;
		for (stepb2d; stepb2d < (HZ*remaining); stepb2d++) {//3 second
			// if (direction==DEFAULT){
			// 	traj_error=remainder(robot.body->GetTransform().q.GetAngle(), M_PI_2);
			// 	correct.update(traj_error);
			// 	correct(action, stepb2d);
			// 	printf("angle =%f,error =%f\n", robot.body->GetTransform().q.GetAngle(),traj_error);
			// }
			instVelocity.x = action.getLinearSpeed()*cos(theta);
			instVelocity.y = action.getLinearSpeed()*sin(theta);
			robot->SetLinearVelocity(instVelocity);
			robot->SetAngularVelocity(action.getOmega());
			robot->SetTransform(robot->GetPosition(), theta);
			if (debugOn){
				fprintf(robotPath, "%f\t%f\n", robot->GetPosition().x, robot->GetPosition().y); //save predictions/
			}
			bool out_x= fabs(robot->GetTransform().p.x)>=(BOX2DRANGE-0.001);
			bool out_y= fabs(robot->GetTransform().p.y)>=(BOX2DRANGE-0.001);
			bool out=(out_x || out_y );
			bool overlap=overlaps(robot, &disturbance);
			if (!overlap){
				disturbance.invalidate();
			}
			if (bool ended=checkEnded(robot->GetTransform(), direction, false, robot).ended; ended || out){ //out
				bool keep_going_out_x=(fabs(robot->GetTransform().p.x+instVelocity.x) >fabs(robot->GetTransform().p.x))&&out_x;
				bool keep_going_out_y=(fabs(robot->GetTransform().p.y+instVelocity.y) >fabs(robot->GetTransform().p.y))&&out_y;
				if (ended){
					break;
				}
				if (keep_going_out_x || keep_going_out_y){
					break;
				}
			}
			_world.Step(1.0f/HZ, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
			theta += action.getOmega()/HZ; //= omega *t
			if (listener.collisions.size()>0){ //
				int index = int(listener.collisions.size()/2);
				Disturbance collision = Disturbance(listener.collisions[index]);
				//b2Vec2 distance = collision.getPosition()-robot.body->GetTransform().p;
				result = simResult(simResult::resultType::crashed, collision);
				//stepb2d=0;
				break;
			}
		}
		result.endPose = robot->GetTransform();
		result.step=stepb2d;
		for (b2Body * b = _world.GetBodyList(); b; b = b->GetNext()){
			_world.DestroyBody(b);
		}
		if (debugOn){
			fclose(robotPath);
		}
		int bodies = _world.GetBodyCount();
		return result;
	
}


void Task::Correct::operator()(Action & action, int step){
	float tolerance = 0.1; //tolerance in radians/pi = just under 2 degrees degrees
	if (action.getOmega()!=0){ //only check every 2 sec, og || motorstep<1
		//printf("returning\n");
		return;
	}
	//printf("error buffer sum = %f, i=%f\n", p(), get_i());
	if (fabs(get_i())>tolerance){//& step>correction_rate
		float p_correction= ((p()/bufferSize)*kp)/2; //do not increase one wheel speed too much
		float i_correction= (get_i()*ki)/2; //do not increase one wheel speed too much
		float d_correction= (get_d()*kd)/2; //do not increase one wheel speed too much
			action.R -= p_correction+ i_correction; 
		 	action.L+= p_correction+ i_correction;
		if (action.L>1.0){
		action.L=1.0;
		}
		if (action.R>1.0){
			action.R=1;
		}
		if (action.L<(-1.0)){
			action.L=-1;
		}
		if (action.R<(-1.0)){
			action.R=-1;
		}
	
	}
}

float Task::Correct::errorCalc(Action a, double x){
	float result=0;
	if (a.getOmega()!=0){
		return result;
	}
	else{
		return sin(a.getOmega())*MOTOR_CALLBACK-float(x); //-ve error if robot goes R, +ve error if goes L
	}	

}


float Task::Correct::update(float e){
	float p0=p();
	p_buffer.erase(p_buffer.begin());
	p_buffer.push_back(e);
	float p1=p();
	mf.buffer.erase(mf.buffer.begin());
	mf.buffer.push_back(p1);
	d=p1-p0;
	i+=e;
	return p1;
}


Direction Task::H(Disturbance ob, Direction d, bool topDown){
	if (ob.isValid()){
        if (ob.getAffIndex()==int(InnateAffordances::AVOID)){ //REACTIVE BEHAVIOUR
            if (d == Direction::DEFAULT & !topDown){ //REACTIVE BEHAVIOUR
                if (ob.getAngle(start)<0){//angle formed with robot at last safe pose
                    d= Direction::LEFT; //go left
                }
                else if (ob.getAngle(start)>0){ //angle formed with robot at last safe pose
                    d= Direction::RIGHT; //
                }   
                else{
                    int c = rand() % 2;
                    d = static_cast<Direction>(c);

                }
            }
        }
		else if (ob.getAffIndex()==int(InnateAffordances::PURSUE)){
			if (d == Direction::DEFAULT & !topDown){ //REACTIVE BEHAVIOUR
                if (ob.getAngle(start)<-.1){//angle formed with robot at last safe pose
                    d= Direction::RIGHT; //go left
                }
                else if (ob.getAngle(start)>0.1){ //angle formed with robot at last safe pose, around .1 rad tolerance
                    d= Direction::LEFT; //
                }   
            }
		}
}
    return d;
}

void Task::Ray::assign(const Robot& robot, const Disturbance & disturbance){
	if (!disturbance.isValid()){
		return;
	}
	b2RayCastInput result;
	result.p1=getClosest(robot, disturbance);
	result.p2= disturbance.bf.pose.p;
	result.maxFraction=1.5;
	*input=result;
}

b2Vec2 Task::Ray::getClosest(const Robot& robot, const Disturbance & disturbance){
	return b2Vec2();

}



void Task::Ray::update(const b2Transform& t){
    if (NULL==input){
        return;
    }
    b2Transform tmp(input->p1, b2Rot(0));
    math::applyAffineTrans(t, tmp);
    input->p1=tmp.p;
}


void Task::setEndCriteria(Angle angle, Distance distance){
	switch(disturbance.getAffIndex()){
		case PURSUE:{
			endCriteria.angle=Angle(0);
			endCriteria.distance = Distance(0+DISTANCE_ERROR_TOLERANCE);
		}
		break;
		// case AVOID:{
		// 	if (direction==DEFAULT){
				
		// 	}
		// 	else{
		// 		//endCriteria.distance = distance;
		// 		endCriteria.angle = angle;
		// 	}
		// }
		break;
		default:
		endCriteria.distance = distance;
		endCriteria.angle = angle;
		break;
	}
	if (disturbance.isValid()){
		endCriteria.valid_d=true;
	}
}

EndedResult Task::checkEnded(b2Transform robotTransform, Direction dir,bool relax, b2Body * robot, std::pair<bool,b2Transform> use_start){ //self-ended , std::pair<bool,b2Transform> use_start
	if (dir==UNDEFINED){
		dir=direction;
	}
	b2Transform this_start= start;
	if (!use_start.first){
		this_start=use_start.second;
	}
	EndedResult r;
	Angle a;
	Distance d;
	//printf("check ended\n");
	b2Vec2 distance=this_start.p-robotTransform.p;
	if (round(distance.Length()*100)/100>=BOX2DRANGE){ //if length reached or turn
		r.ended =true;
	}
	if (disturbance.isValid()){
		b2Vec2 v = disturbance.getPosition() - robotTransform.p; //distance between disturbance and robot
		d= Distance(v.Length());
		if (action.getOmega()!=0){
			a =Angle(robotTransform.q.GetAngle());					
			float angleL = start.q.GetAngle()+SAFE_ANGLE;
			float angleR = start.q.GetAngle()-SAFE_ANGLE;
			float robotAngle=robotTransform.q.GetAngle();
			int mult= start.q.GetAngle()/(3*M_PI_4);
			if (mult>0){
				if (dir==LEFT& robotAngle<0){
					robotAngle+=2*M_PI;
				}
			}
			else if (mult<0){
				if (dir==RIGHT & robotAngle>0){
					robotAngle-=2*M_PI;
				}
			}
			bool finishedLeft=robotAngle>=angleL;//-(action.getOmega()*HZ)/2;
			bool finishedRight=robotAngle<=angleR;//+(action.getOmega()*HZ)/2;
			if (finishedLeft|| finishedRight){
				if (disturbance.getAffIndex()==AVOID){
					disturbance.invalidate();
				}
				r.ended = 1;
			}
		}
		else if (getAffIndex()== int(InnateAffordances::NONE)){
			a =Angle(robotTransform.q.GetAngle());		
			r.ended = true;
		}
		else if (getAffIndex()==int(InnateAffordances::PURSUE)){
			a = Angle(disturbance.getAngle(robotTransform));
			//local level if D
			if (robot!=NULL){
				b2Vec2 pos_local=disturbance.getPosition();
				pos_local=robot->GetLocalPoint(pos_local);
				r.ended=fabs(pos_local.x)<=endCriteria.distance.get()/2;
			}
			else if (relax){
				Distance _d(RELAXED_DIST_ERROR_TOLERANCE);
				r.ended = d<=_d; 
			}
			else{
				r.ended = d<=endCriteria.distance; 
			}
		}
	}
	else if (dir==LEFT || dir ==RIGHT){
		float angleL = this_start.q.GetAngle()+endCriteria.angle.get();
		float angleR = this_start.q.GetAngle()-endCriteria.angle.get();
		r.ended = (robotTransform.q.GetAngle()>=angleL || robotTransform.q.GetAngle()<=angleR);	
	}
	else if (dir==DEFAULT && getAffIndex()==AVOID){
		r.ended=true;
	}
	r.estimatedCost = endCriteria.getStandardError(a,d);
	return r;

}

EndedResult Task::checkEnded(State n,  Direction dir, bool relax, std::pair<bool,b2Transform> use_start){ //check error of node compared to the present Task
	EndedResult r;
	Angle a;
	Distance d;
	r = checkEnded(n.endPose, dir, relax,NULL, use_start);
	r.estimatedCost+= endCriteria.getStandardError(a,d, n);
	return r;
}
// b2AABB Task::makeRobotSensor(b2Body* robotBody){
// 	b2AABB result;
// 	if (!(disturbance.getAffIndex()==AVOID)){
// 		return result;
// 	}
// 	b2PolygonShape * poly_robo=(b2PolygonShape*)robotBody->GetFixtureList()->GetShape();
// 	//b2PolygonShape * poly_d=(b2PolygonShape*)disturbance.bf.fixture.GetShape();
// 	std::vector <b2Vec2> all_points=arrayToVec(poly_robo->m_vertices, poly_robo->m_count), d_vertices=disturbance.vertices();
// 	for (b2Vec2 p: d_vertices){
// 		p =robotBody->GetLocalPoint(p);
// 		all_points.push_back(p);
// 	}
// 	float minx=(std::min_element(all_points.begin(),all_points.end(), CompareX())).base()->x;
// 	float miny=(std::min_element(all_points.begin(), all_points.end(), CompareY())).base()->y;
// 	float maxx=(std::max_element(all_points.begin(), all_points.end(), CompareX())).base()->x;
// 	float maxy=(std::max_element(all_points.begin(), all_points.end(), CompareY())).base()->y;
// 	float halfLength=(fabs(maxy-miny))/2; //
//     float halfWidth=(fabs(maxx-minx))/2;
// 	b2Vec2 centroid(maxx-halfWidth, maxy-halfLength);
// 	b2Vec2 offset=centroid - robotBody->GetLocalPoint(robotBody->GetPosition());
// 	b2PolygonShape shape;
// 	shape.SetAsBox(halfWidth, halfLength, offset, 0);
// 	b2FixtureDef fixtureDef;
// 	fixtureDef.isSensor=true;
// 	fixtureDef.shape=&shape;
// 	robotBody->CreateFixture(&fixtureDef);
// 	shape.ComputeAABB(&result, robotBody->GetTransform(), 0);
// 	return result;
	
// }


EndCriteria Task::getEndCriteria(const Disturbance &d){
	EndCriteria endCriteria;
	switch(disturbance.getAffIndex()){
	case PURSUE:{
		endCriteria.angle=Angle(0);
		endCriteria.distance = Distance(0+DISTANCE_ERROR_TOLERANCE);
	}
	break;
	default:
	endCriteria.distance = BOX2DRANGE;
	break;
}
}


