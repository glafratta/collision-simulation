#include "task.h"



simResult Task::willCollide(b2World & _world, int iteration, bool debugOn=0, float remaining=8.0){ //CLOSED LOOP CONTROL, og return simreult
		simResult result;
		if (!discrete){
			result = simResult(simResult::resultType::successful);
		}
		else{
			result = simResult(simResult::resultType::safeForNow); //indicator that it is discretized
		}
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
			instVelocity.x = action.getRecSpeed()*cos(theta);
			instVelocity.y = action.getRecSpeed()*sin(theta);
			robot.body->SetLinearVelocity(instVelocity);
			robot.body->SetAngularVelocity(action.getRecOmega());
			robot.body->SetTransform(robot.body->GetPosition(), theta);
			if (debugOn){
				fprintf(robotPath, "%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y); //save predictions/
			}
			_world.Step(1.0f/HZ, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
			theta += action.getRecOmega()/HZ; //= omega *t
			if (checkEnded(robot.body->GetTransform()).ended){
				if (direction==BACK){
					result = simResult(simResult::resultType::successful, disturbance);
				}
				break;
			}
			if (listener.collisions.size()>0){ //
				int index = int(listener.collisions.size()/2);
				Disturbance collision = Disturbance(1, listener.collisions[index]);
				// std::pair<bool, b2Vec2> neighbour = findNeighbourPoint(_world,listener.collisions[index]);
				// if (neighbour.first){
				// 	float orientation =findOrientation(listener.collisions[index], neighbour.second);
				// 	collision.setOrientation(orientation);
				// }
				b2Vec2 distance = collision.getPosition()-robot.body->GetTransform().p;
				if (direction==DEFAULT && step/HZ >REACTION_TIME){ //stop 2 seconds before colliding so to allow the robot to explore
					b2Vec2 posReadjusted;
					posReadjusted.x = start.p.x+ instVelocity.x*(step/HZ-REACTION_TIME);						
					posReadjusted.y = start.p.y+ instVelocity.y*(step/HZ-REACTION_TIME);
					// posReadjusted.x = robot.body->GetTransform().p.x -cos(action.getOmega())*(SAFE_DISTANCE-distance.Length());						
					// posReadjusted.y = robot.body->GetTransform().p.y -sin(action.getOmega())*(SAFE_DISTANCE-distance.Length());						
					robot.body->SetTransform(posReadjusted, start.q.GetAngle()); //if the simulation crashes reset position for 
					result = simResult(simResult::resultType::safeForNow, collision);
				}
				else{
					result = simResult(simResult::resultType::crashed, collision);
					//DEBUG PRINT STATEMTNS
// 					if ((direction==BACK) & (start.p == b2Vec2(0,0)) & (start.q.GetAngle()==0)){
// //						printf("SSSSHIIIIITTTTTTTTTTTTTTT SOMETHING BEHIND EMEEEEEEEEEEEEEE at %f, %f, step = %i\n", listener.collisions[index].x, listener.collisions[index].y, step);
// 					}
					//END DEBUG
					robot.body->SetTransform(start.p, start.q.GetAngle()); //if the simulation crashes reset position for 
					//result.collision.safeForNow =0;
					}
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
		int roboCount=0;
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
	b2Transform shift(b2Vec2(-robVelocity.p.x*timeElapsed, -robVelocity.p.y*timeElapsed), b2Rot(-robVelocity.q.GetAngle())); //calculates shift in the time step
	b2Vec2 newPos(d.getPosition().x+shift.p.x,d.getPosition().y + shift.p.y);
	d.setPosition(newPos);
	// float angle = d.getAngle(robVelocity);
	float angle = d.getAngle(pose);
	if (d.isPartOfObject()){
		d.setOrientation(d.getOrientation() + shift.q.GetAngle());
	}
	d.setAngle(angle); //with respect to robot's velocity
}

Task::controlResult Task::controller(){
//float recordedAngle = action.getOmega()/0.2;
float tolerance = 0.01; //tolerance in radians/pi = just under 2 degrees degrees
bool ended = checkEnded().ended;
if (disturbance.isValid() & disturbance.getAffIndex() == int(InnateAffordances::AVOID)){}
else {
	float timeStepError =action.getOmega()/0.2; 
	accumulatedError += timeStepError; 
	if (timeStepError<tolerance){
		action.L = 0.5;
		action.R = 0.5;
	}
	else{
		float normAccErr = timeStepError/M_PI_2;
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
	return CONTINUE;
}

Direction Task::H(Disturbance ob, Direction d){
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
			if (d == Direction::DEFAULT){ //REACTIVE BEHAVIOUR
                if (ob.getAngle(start)<-.1){//angle formed with robot at last safe pose
                    d= Direction::RIGHT; //go left
                }
                else if (ob.getAngle(start)>0.1){ //angle formed with robot at last safe pose, around .1 rad tolerance
                    d= Direction::LEFT; //
                }   
                else{
                    d = DEFAULT;

                }
            }
		}
    //printf("angle to ob = %f\n", ob.getAngle());
}
    return d;
}

// void Task::setEndCriteria(){ //standard end criteria, can be modified by changing angle/distnace
// 	switch (direction){
// 		case DEFAULT: {
// 			if (disturbance.getAffIndex()==int(InnateAffordances::PURSUE)){
// 			endCriteria.distance = Distance(0.05); //end if D is 5 cm away
// 		}
// 		break;
// 		}
// 		case LEFT: {
// 			if (disturbance.getAffIndex()==int(InnateAffordances::AVOID)){
// 			if (!disturbance.isPartOfObject()){
// 				endCriteria.angle = Angle(SAFE_ANGLE);
// 			}
// 			else{
// 				endCriteria.angle = Angle(0);
// 			}
// 			}
// 			else if (disturbance.getAffIndex()==int(InnateAffordances::PURSUE)){
// 				endCriteria.angle = Angle(0);
// 			}
// 			break;
// 		}
// 		case RIGHT:{
// 			if (disturbance.getAffIndex()==int(InnateAffordances::AVOID)){
// 				if (!disturbance.isPartOfObject()){
// 					endCriteria.angle = Angle(SAFE_ANGLE);
// 				}
// 				else{
// 					endCriteria.angle = Angle(0);
// 				}
// 			}
// 			else if (disturbance.getAffIndex()==int(InnateAffordances::PURSUE)){
// 				endCriteria.angle = Angle(0);
// 			}
// 			break;
// 		} 
// 		case BACK: {
// 			if (disturbance.getAffIndex()==int(InnateAffordances::AVOID)){
// 			b2Vec2 v = disturbance.getPosition() - start.p;
// 			Distance d(v.Length());
// 			endCriteria.distance = Distance(v.Length()+SAFE_DISTANCE);
// 		}
// 		break;
// 		}
// 		case STOP:{
// 		if (disturbance.getAffIndex()==int(InnateAffordances::AVOID)){
// 			endCriteria.angle = Angle(SAFE_ANGLE);
// 			b2Vec2 v = disturbance.getPosition() - start.p;
// 			endCriteria.distance = Distance(v.Length()+SAFE_DISTANCE);
// 		}
// 		break;
// 		}

// 		default:break;
// 	}
// }

void Task::setEndCriteria(){
	Distance d;
	Angle a;
	b2Vec2 v;
	switch(disturbance.getAffIndex()){
		case AVOID: {
			if (direction !=BACK){
				endCriteria.distance.setValid(1);
			}
			else{
				v = disturbance.getPosition() - start.p;
				endCriteria.distance = Distance(v.Length()+SAFE_DISTANCE);
			}
			if (disturbance.isPartOfObject()){
				endCriteria.angle.setValid(1);
				}
			else{
				endCriteria.angle=Angle(SAFE_ANGLE);
			}
		}
		break;
		case PURSUE:{
			endCriteria.angle = Angle(0);
			endCriteria.distance = Distance(0);
		}
		break;
		default:
		break;
	}
	
}

EndedResult Task::checkEnded(b2Transform robotTransform){
	EndedResult r;
	Angle a;
	Distance d;
	if (disturbance.isValid()){
		b2Vec2 v = disturbance.getPosition() - robotTransform.p; //distance between disturbance and robot
		d= Distance(v.Length());
		if (getAffIndex()== int(InnateAffordances::AVOID)){
			if (!disturbance.isPartOfObject()){
				a= Angle(abs(disturbance.getAngle(robotTransform)));
				r.ended= a>= endCriteria.angle || d>=endCriteria.distance;
			}
			else{
				a = Angle(abs(atan(robotTransform.q.s/robotTransform.q.c)-disturbance.getOrientation())); //operations on angles between -Pi/2 and +pi/2, difference between orientation of d and robot
				//float a = abs(a.get()); // the robot and disturbance are parallel
				float angleTolerance = 1*M_PI/180;
				Angle comparAngle =Angle(abs(endCriteria.angle.get()) + angleTolerance);
				comparAngle.setValid(endCriteria.angle.isValid());
				r.ended = a<=comparAngle || d>=endCriteria.distance;
			}

		}
		else if (getAffIndex()== int(InnateAffordances::NONE)){
			r.ended = true;
		}
		else if (getAffIndex()==int(InnateAffordances::PURSUE)){
			a = Angle(disturbance.getAngle(robotTransform));
			//b2Vec2 v = disturbance.getPosition() - robotTransform.p;
			//""d= Distance(v.Length());
			Angle lowAngle = Angle(endCriteria.angle.get()-ANGLE_ERROR_TOLERANCE);
			lowAngle.setValid(endCriteria.angle.isValid());
			Angle hiAngle =Angle(endCriteria.angle.get() +ANGLE_ERROR_TOLERANCE);
			hiAngle.setValid(endCriteria.angle.isValid());
			r.ended = d<=endCriteria.distance & lowAngle <=a & hiAngle>=a;
		}
	}
	r.errorFloat = endCriteria.getStandardError(a,d);
	return r;

}

EndedResult Task::checkEnded(Node n){ //check error of node compared to the present Task
	EndedResult r;
	Angle a;
	Distance d;
	if (disturbance.isValid()){
		b2Vec2 v = disturbance.getPosition() - n.endPose.p; //distance between disturbance and robot
		if (getAffIndex()== int(InnateAffordances::AVOID)){
			if (!disturbance.isPartOfObject()){
				a= Angle(disturbance.getAngle(n.endPose));
				d= Distance(v.Length());
				r.ended= abs(a.get())>= endCriteria.angle.get() && d.get()>=endCriteria.distance.get();
			}
			else{
				a = Angle(atan(n.endPose.q.s/n.endPose.q.c)-disturbance.getOrientation()); //operations on angles between -Pi/2 and +pi/2, difference between orientation of d and robot
				//float a = abs(a.get()); // the robot and disturbance are parallel
				float angleTolerance = 1*M_PI/180;
				r.ended = abs(a.get()) <= abs(endCriteria.angle.get()) + angleTolerance & d.get()>=endCriteria.distance.get();
			}

		}
		else if (getAffIndex()== int(InnateAffordances::NONE)){
			r.ended = true;
		}
		else if (getAffIndex()==int(InnateAffordances::PURSUE)){
			a = Angle(disturbance.getAngle(n.endPose));
			//b2Vec2 v = disturbance.getPosition() - robotTransform.p;
			d= Distance(v.Length());
			r.ended = v.Length()<=endCriteria.distance.get() & (endCriteria.angle.get()-ANGLE_ERROR_TOLERANCE) <=a.get() & (endCriteria.angle.get() +ANGLE_ERROR_TOLERANCE)>=a.get();
		}
	}
	r.errorFloat = endCriteria.getStandardError(a,d, n);
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

