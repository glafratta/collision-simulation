#include "task.h"



Task::simResult Task::willCollide(b2World & _world, int iteration, bool debugOn=0, b2Vec2 start = b2Vec2(), float _theta=0.0, float remaining=8.0){ //CLOSED LOOP CONTROL, og return simreult
		simResult result = simResult(simResult::resultType::successful);
		Robot robot(&_world);
		Listener listener;
		_world.SetContactListener(&listener);	
		FILE * robotPath;
		if (debugOn){
			sprintf(planFile, "/tmp/robot%04i.txt", iteration);
			robotPath = fopen(planFile, "a");
		}
		float theta = _theta;
		b2Vec2 instVelocity = {0,0};
		robot.body->SetTransform(start, theta);
		int step=0;
		for (step; step < (HZ*remaining); step++) {//3 second
			instVelocity.x = RecordedVelocity.Length()*cos(theta); //integrate?
			instVelocity.y = RecordedVelocity.Length()*sin(theta);
			robot.body->SetLinearVelocity(instVelocity);
			robot.body->SetAngularVelocity(action.getOmega());
			robot.body->SetTransform(robot.body->GetPosition(), theta);
			if (debugOn){
				fprintf(robotPath, "%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y); //save predictions
			}
			_world.Step(1.0f/HZ, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
			theta += action.getOmega()/HZ; //= omega *t
//			if (disturbance.isValid() || disturbance.getAffIndex() == int(InnateAffordances::AVOID)){
//				float absAngleToObstacle = abs(disturbance.getAngle(robot.body));
				// if (absAngleToObstacle>=endAvoid){
				// 	bool ended = checkEnded(robot.body->GetTransform()); //this will substitute the if loop below
				// 	break;
				// }

//			}
			if (checkEnded(robot.body->GetTransform())){
					break;
			}
			if (listener.collisions.size()>0){ //
				int index = int(listener.collisions.size()/2);
				if (getAffIndex()==int(InnateAffordances::NONE) && step/HZ >REACTION_TIME){ //stop 2 seconds before colliding so to allow the robot to explore
					b2Vec2 posReadjusted;
					posReadjusted.x = start.x+ instVelocity.x*(step/HZ-REACTION_TIME);						
					posReadjusted.y = start.y+ instVelocity.y*(step/HZ-REACTION_TIME);						
					robot.body->SetTransform(posReadjusted, _theta); //if the simulation crashes reset position for 
					result = simResult(simResult::resultType::safeForNow, Disturbance(1, listener.collisions[index]));
				}
				else{
					result = simResult(simResult::resultType::crashed, Disturbance(1, listener.collisions[index]));
					robot.body->SetTransform(start, _theta); //if the simulation crashes reset position for 
					result.collision.safeForNow =0;

					}
				break;
			}
		}
		result.collision.setAngle(robot.body->GetTransform());	
		b2Vec2 distance; //= robot.body->GetPosition();
		distance.x = robot.body->GetPosition().x - start.x;
		distance.y = robot.body->GetPosition().y - start.y;
		result.distanceCovered = distance.Length() ;
		result.endPose = robot.body->GetTransform();
		int roboCount=0;
		for (b2Body * b = _world.GetBodyList(); b!=NULL; b = b->GetNext()){
			_world.DestroyBody(b);
		}
		if (debugOn){
			fclose(robotPath);
		}

		return result;
	
}


//void Task::trackDisturbance(Disturbance & d, float timeElapsed, b2VEc2 robVelocity, b2Vec2 robPos){ //isInternal refers to whether the tracking is with respect to the global coordinate frame (i.e. in willCollide) if =1, if isIntenal =0 it means that the Disturbance is tracked with the robot in the default position (0.0)
void Task::trackDisturbance(Disturbance & d, float timeElapsed, b2Vec2 robVelocity, b2Transform pose){ //isInternal refers to whether the tracking is with respect to the global coordinate frame (i.e. in willCollide) if =1, if isIntenal =0 it means that the Disturbance is tracked with the robot in the default position (0.0)
	b2Vec2 shift = {-robVelocity.x*timeElapsed, -robVelocity.y*timeElapsed}; //calculates shift in the time step
	b2Vec2 newPos(d.getPosition().x+shift.x,d.getPosition().y + shift.y);
	d.setPosition(newPos);
	// float angle = d.getAngle(robVelocity);
	float angle = d.getAngle(pose);
	d.setAngle(angle); //with respect to robot's velocity
}

Task::controlResult Task::controller(){
float recordedAngle = atan(RecordedVelocity.y/RecordedVelocity.x);
float tolerance = 0.01; //tolerance in radians/pi = just under 2 degrees degrees
bool ended = checkEnded();
if (disturbance.isValid() & disturbance.getAffIndex() == int(InnateAffordances::AVOID)){
	// float obstacleAngle = atan(disturbance.getPosition().y/disturbance.getPosition().x);
	// float angleDifference = obstacleAngle - recordedAngle;
	// if (abs(angleDifference) >= endAvoid){
	// 	disturbance.invalidate();
	// 	return DONE;
	// }
}
else {
	float timeStepError =action.getOmega()*0.2 - recordedAngle; 
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
                if (ob.getAngle()<0){//angle formed with robot at last safe pose
                    d= Direction::LEFT; //go left
                }
                else if (ob.getAngle()>0){ //angle formed with robot at last safe pose
                    d= Direction::RIGHT; //
                }   
                else{
                    int c = rand() % 2;
                    d = static_cast<Direction>(c);

                }
            }
        }
    //printf("angle to ob = %f\n", ob.getAngle());
}
    return d;
}

void Task::setEndCriteria(){ //standard end criteria, can be modified by changing angle/distnace
	switch (direction){
		case DEFAULT: break;
		case LEFT: 
		if (disturbance.getAffIndex()==int(InnateAffordances::AVOID)){
			endCriteria.angle = Angle(M_PI_2);
		}
		break;
		case RIGHT: 
		if (disturbance.getAffIndex()==int(InnateAffordances::AVOID)){
			endCriteria.angle = Angle(M_PI_2);
		}
		break;
		case BACK: 
		if (disturbance.getAffIndex()==int(InnateAffordances::AVOID)){
			endCriteria.distance = Distance(.05);
		}
		break;
		case STOP:
		if (disturbance.getAffIndex()==int(InnateAffordances::AVOID)){
			endCriteria.angle = Angle(M_PI_2);
			endCriteria.distance = Distance(.05);
		}
		break;
		default:break;
	}
}

bool Task::checkEnded(b2Transform robotTransform){
	bool r = false;
	if (disturbance.isValid()){
		if (getAffIndex()== int(InnateAffordances::AVOID)){
			Angle a(abs(disturbance.getAngle(robotTransform)));
			b2Vec2 v = disturbance.getPosition() - robotTransform.p;
			Distance d(v.Length());
			r= a>= endCriteria.angle && d>=endCriteria.distance;
		}
		else if (getAffIndex()== int(InnateAffordances::NONE)){
			r = true;
		}
	}
	return r;

}
