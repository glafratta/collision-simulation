#include "task.h"



simResult Task::willCollide(b2World & _world, int iteration, bool debugOn, float remaining, float simulationStep){ //CLOSED LOOP CONTROL, og return simreult
		simResult result=simResult(simResult::resultType::successful);
		// if (discrete & action.getOmega()==0){
		// 	result = simResult(simResult::resultType::safeForNow); //indicator that it is discretized
		// }
		Robot robot(&_world);
		Listener listener;
		_world.SetContactListener(&listener);	
		FILE * robotPath;
		if (debugOn){
			sprintf(planFile, "/tmp/robot%04i.txt", iteration);
			robotPath = fopen(planFile, "a");
		}
		float theta = start.q.GetAngle();
		b2Vec2 instVelocity = {0,0};
		b2Vec2 pointInObject;
		robot.body->SetTransform(start.p, theta);
		int step=0;
		for (step; step < (HZ*remaining); step++) {//3 second
			instVelocity.x = action.getLinearSpeed()*cos(theta);
			instVelocity.y = action.getLinearSpeed()*sin(theta);
			robot.body->SetLinearVelocity(instVelocity);
			robot.body->SetAngularVelocity(action.getOmega());
			robot.body->SetTransform(robot.body->GetPosition(), theta);
			if (debugOn){
				fprintf(robotPath, "%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y); //save predictions/
			}
			if (checkEnded(robot.body->GetTransform()).ended || (start.p-robot.body->GetTransform().p).Length()>=simulationStep){
				break;
			}
			_world.Step(1.0f/HZ, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
			theta += action.getRecOmega()/HZ; //= omega *t
			if (listener.collisions.size()>0){ //
				int index = int(listener.collisions.size()/2);
				Disturbance collision = Disturbance(1, listener.collisions[index]);
				b2Vec2 distance = collision.getPosition()-robot.body->GetTransform().p;
				result = simResult(simResult::resultType::crashed, collision);
				robot.body->SetTransform(start.p, start.q.GetAngle()); //if the simulation crashes reset position for 
				collision.setOrientation(robot.body->GetTransform().q.GetAngle());
				break;
			}
			// else if (getAffIndex()== int(InnateAffordances::AVOID)){
			// 	result.step =step;
			// }
		}
		result.collision.setAngle(robot.body->GetTransform());	
		b2Vec2 distance; //= robot.body->GetPosition();
		distance.x = robot.body->GetPosition().x - start.p.x;
		distance.y = robot.body->GetPosition().y - start.p.y;
		result.distanceCovered = distance.Length() ;
		result.endPose = robot.body->GetTransform();
		result.step=step;
		//int roboCount=0;
		for (b2Body * b = _world.GetBodyList(); b!=NULL; b = b->GetNext()){
			_world.DestroyBody(b);
			//b=NULL;
		}
		if (debugOn){
			fclose(robotPath);
		}
		int bodies = _world.GetBodyCount();
		return result;
	
}


//void Task::trackDisturbance(Disturbance & d, float timeElapsed, b2VEc2 robVelocity, b2Vec2 robPos){ //isInternal refers to whether the tracking is with respect to the global coordinate frame (i.e. in willCollide) if =1, if isIntenal =0 it means that the Disturbance is tracked with the robot in the default position (0.0)
void Task::trackDisturbance(Disturbance & d, float timeElapsed, b2Transform robVelocity, b2Transform pose){ //isInternal refers to whether the tracking is with respect to the global coordinate frame (i.e. in willCollide) if =1, if isIntenal =0 it means that the Disturbance is tracked with the robot in the default position (0.0)
	b2Transform shift(b2Vec2(-robVelocity.p.x*timeElapsed, -robVelocity.p.y*timeElapsed), b2Rot(-robVelocity.q.GetAngle()*timeElapsed)); //calculates shift in the time step
	b2Vec2 newPos(d.getPosition().x+shift.p.x,d.getPosition().y + shift.p.y);
	d.setPosition(newPos);
	// float angle = d.getAngle(robVelocity);
	float angle = d.getAngle(pose);
	if (d.isPartOfObject()){
		d.setOrientation(d.getOrientation() + shift.q.GetAngle());
	}
	d.setAngle(angle); //with respect to robot's velocity
}

void Task::trackDisturbance(Disturbance & d, Action a){
	// //switch(dir){
	float angleTurned =MOTOR_CALLBACK*a.getOmega()/FRICTION;
	printf("initial angle =%f, angle turned = %f\n", d.pose.q.GetAngle(), angleTurned);
	d.pose.q.Set(d.pose.q.GetAngle()-angleTurned);	
	// printf("NEW angle =%f\n", d.pose.q.GetAngle());
	// //float deltaLength = d.pose.p.Length()-s*MOTOR_CALLBACK*a.getLinearSpeed();
	// printf("omega = %f, linear speed %f, pose = %f", a.getOmega(), a.getLinearSpeed(), d.pose.q.GetAngle());
	// //float angle = atan2(d.pose.p.y, d.pose.p.x);
	// d.pose.p.x-= -sin(angleTurned)* s*MOTOR_CALLBACK*a.getLinearSpeed();
	// d.pose.p.y -= cos(angleTurned)* s*MOTOR_CALLBACK*a.getLinearSpeed();
	// //}
	float distanceTraversed = MOTOR_CALLBACK*a.getLinearSpeed();
	float initialL = d.pose.p.Length();
	d.pose.p.x=cos(d.pose.q.GetAngle())*initialL-cos(angleTurned)*distanceTraversed;
	d.pose.p.y = sin(d.pose.q.GetAngle())*initialL-sin(angleTurned)*distanceTraversed;
}

void Task::controller(float timeElapsed){
//float recordedAngle = action.getOmega()/0.2;
	float tolerance = 0.01; //tolerance in radians/pi = just under 2 degrees degrees
	bool ended = checkEnded().ended;
	if (direction !=Direction::DEFAULT || motorStep<1 || motorStep%5!=0){ //only check every second
		return;
	}
	float timeStepError =action.getRecOmega()/timeElapsed; 
	//accumulatedError += timeStepError; 
	if (timeStepError>tolerance){
		float normAccErr = timeStepError/SAFE_ANGLE;
		printf("error = %f\n", normAccErr);
		action.L -= normAccErr*pGain;  
		action.R += normAccErr *pGain; 
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

Direction Task::H(Disturbance ob, Direction d, bool topDown){
	if (ob.isValid()){
        if (ob.getAffIndex()==int(InnateAffordances::AVOID)){ //REACTIVE BEHAVIOUR
            if (d == Direction::DEFAULT){ //REACTIVE BEHAVIOUR
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
    //printf("angle to ob = %f\n", ob.getAngle());
}
    return d;
}

void Task::setEndCriteria(){
	Distance d;
	Angle a;
	b2Vec2 v;
	switch(disturbance.getAffIndex()){
		case AVOID: {
	//		if (direction !=BACK){
				endCriteria.distance.setValid(1);
	//		}
	//		else{
	//			v = disturbance.getPosition() - start.p;
	//			endCriteria.distance = Distance(v.Length()+SAFE_DISTANCE);
	//		}
			if (disturbance.isPartOfObject()){
				endCriteria.angle.setValid(1);
				}
			else{
				endCriteria.angle=Angle(SAFE_ANGLE);
			}
		}
		break;
		case PURSUE:{
//			endCriteria.angle = Angle(0);
			endCriteria.distance = Distance(0+DISTANCE_ERROR_TOLERANCE);
		}
		break;
		default:
		break;
	}
	
}

EndedResult Task::checkEnded(b2Transform robotTransform){ //self-ended
	EndedResult r;
	Angle a;
	Distance d;
	if (disturbance.isValid()){
		b2Vec2 v = disturbance.getPosition() - robotTransform.p; //distance between disturbance and robot
		d= Distance(v.Length());
		// if (getAffIndex()== int(InnateAffordances::AVOID)){
		// 	if (!disturbance.isPartOfObject()){
		// 		a= Angle(fabs(disturbance.getAngle(robotTransform)));
		// 		r.ended= a>= endCriteria.angle || d>=endCriteria.distance;
		// 	}
		// 	else{
		// 		//a = Angle(abs(atan(robotTransform.q.s/robotTransform.q.c)-disturbance.getOrientation())); //operations on angles between -Pi/2 and +pi/2, difference between orientation of d and robot
		// 		a = Angle(fabs((robotTransform.q.GetAngle())-disturbance.getOrientation())); //operations on angles between -Pi/2 and +pi/2, difference between orientation of d and robot
		// 		//float a = abs(a.get()); // the robot and disturbance are parallel
		// 		//float angleTolerance = 1*M_PI/180;
		// 		Angle comparAngle =Angle(fabs(endCriteria.angle.get()));
		// 		comparAngle.setValid(endCriteria.angle.isValid());
		// 		r.ended = a<=comparAngle || d>=endCriteria.distance;
		// 	}

		// }
		if (action.getOmega()!=0){
			float angleL = start.q.GetAngle()+M_PI_2;
			float angleR = start.q.GetAngle()-M_PI_2;
			r.ended = robotTransform.q.GetAngle()>=angleL || robotTransform.q.GetAngle()<=angleR;
			//r.ended=fabs(fabs(atan(robotTransform.q.s/robotTransform.q.c))-fabs(atan(start.q.s/start.q.c)))>=M_PI_2;
		}
		else if (getAffIndex()== int(InnateAffordances::NONE)){
			r.ended = true;
		}
		else if (getAffIndex()==int(InnateAffordances::PURSUE)){
			a = Angle(disturbance.getAngle(robotTransform));
			//b2Vec2 v = disturbance.getPosition() - robotTransform.p;
			//""d= Distance(v.Length());
			// Angle lowAngle = Angle(endCriteria.angle.get()-ANGLE_ERROR_TOLERANCE);
			// lowAngle.setValid(endCriteria.angle.isValid());
			// Angle hiAngle =Angle(endCriteria.angle.get() +ANGLE_ERROR_TOLERANCE);
			// hiAngle.setValid(endCriteria.angle.isValid());
			//bool angleEnd = (fabs(robotTransform.q.GetAngle())-fabs(start.q.GetAngle()))>=M_PI_2;
			r.ended = d<=endCriteria.distance; 
		}
	}
	// else if ((fabs(robotTransform.q.GetAngle())-fabs(start.q.GetAngle()))>=M_PI_2 & getAffIndex()==int(InnateAffordances::NONE)){
	// 	r.ended =true;
	// }
	else{
		float angleL = start.q.GetAngle()+M_PI_2;
		float angleR = start.q.GetAngle()-M_PI_2;
		r.ended = robotTransform.q.GetAngle()>=angleL || robotTransform.q.GetAngle()<=angleR;
	}
	//if (round(robotTransform.p.Length()*100)/100>=BOX2DRANGE || fabs(fabs(robotTransform.q.GetAngle())-fabs(start.q.GetAngle()))>=M_PI_2){ //if length reached or turn
	if (round(robotTransform.p.Length()*100)/100>=BOX2DRANGE){ //if length reached or turn
		r.ended =true;
	}
	r.errorFloat = endCriteria.getStandardError(a,d);
	return r;

}

EndedResult Task::checkEnded(Node n){ //check error of node compared to the present Task
	EndedResult r;
	Angle a;
	Distance d;
	// if (disturbance.isValid()){
	// 	b2Vec2 v = disturbance.getPosition() - n.endPose.p; //distance between disturbance and robot
	// 	if (getAffIndex()== int(InnateAffordances::AVOID)){
	// 		if (!disturbance.isPartOfObject()){
	// 			a= Angle(disturbance.getAngle(n.endPose));
	// 			d= Distance(v.Length());
	// 			r.ended= abs(a.get())>= endCriteria.angle.get() && d.get()>=endCriteria.distance.get();
	// 		}
	// 		else{
	// 			a = Angle(atan(n.endPose.q.s/n.endPose.q.c)-disturbance.getOrientation()); //operations on angles between -Pi/2 and +pi/2, difference between orientation of d and robot
	// 			//float a = abs(a.get()); // the robot and disturbance are parallel
	// 			float angleTolerance = 1*M_PI/180;
	// 			r.ended = abs(a.get()) <= abs(endCriteria.angle.get()) + angleTolerance & d.get()>=endCriteria.distance.get();
	// 		}

	// 	}
	// 	else if (getAffIndex()== int(InnateAffordances::NONE)){
	// 		r.ended = true;
	// 	}
	// 	else if (getAffIndex()==int(InnateAffordances::PURSUE)){
	// 		a = Angle(disturbance.getAngle(n.endPose));
	// 		//b2Vec2 v = disturbance.getPosition() - robotTransform.p;
	// 		d= Distance(v.Length());
	// 		r.ended = v.Length()<=endCriteria.distance.get() & (endCriteria.angle.get()-ANGLE_ERROR_TOLERANCE) <=a.get() & (endCriteria.angle.get() +ANGLE_ERROR_TOLERANCE)>=a.get();
	// 	}
	// }
	r = checkEnded(n.endPose);
	r.errorFloat+= endCriteria.getStandardError(a,d, n);
	return r;


	
}

// std::pair<bool, b2Vec2> Task::findNeighbourPoint(b2World &w, b2Vec2 v, float radius){
// 	std::pair <bool, b2Vec2> result(false, b2Vec2());
// 	for (b2Body * b= w.GetBodyList(); b !=NULL; b=b->GetNext()){
// 		Point p(*b->GetPosition());
// 		if (p.isInRadius(v, radius)){
// 			return result=std::pair<bool, b2Vec2>(true, b->GetPosition());
// 		}
// 	}
// 	return result;
// }

float Task::findOrientation(b2Vec2 v1, b2Vec2 v2){
	float slope = (v2.y- v1.y)/(v2.x - v1.x);
	return atan(slope);
}

