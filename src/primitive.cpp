#include "primitive.h"
#include "robot.h"
//#include "opencv2/opencv.hpp"



Primitive::simResult Primitive::willCollide(b2World & _world, int iteration, bool debugOn=0, b2Vec2 start = b2Vec2(), float _theta=0.0){ //CLOSED LOOP CONTROL, og return simreult
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
		//printf("starting from x =%f, y=%f, theta = %fpi \n", start.x, start.y, theta/M_PI);
		b2Vec2 instVelocity = {0,0};
		robot.body->SetTransform(start, theta);
		//printf("entering for loop\n");
		int step=0;
		//printf("action type = %i, speed = %f, omega = %f\n", action.getDirection(),RecordedVelocity.Length(), action.getOmega());
		for (step; step < (hz*simDuration); step++) {//3 second
			instVelocity.x = RecordedVelocity.Length()*cos(theta); //integrate?
			instVelocity.y = RecordedVelocity.Length()*sin(theta);
			robot.body->SetLinearVelocity(instVelocity);
			robot.body->SetAngularVelocity(action.getOmega());
			robot.body->SetTransform(robot.body->GetPosition(), theta);
			if (debugOn){
				fprintf(robotPath, "%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y); //save predictions
			}
			_world.Step(1.0f/hz, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
			theta += action.getOmega()/hz; //= omega *t
			//printf("x");
			if (obstacle.isValid()){
				//float absAngleToObstacle = abs(obstacle.getAngle(robot.body->GetAngle()));
				float absAngleToObstacle = abs(obstacle.getAngle(robot.body));

				if (absAngleToObstacle>=M_PI_2){
					break;
				}
			}
			if (listener.collisions.size()>0){ //
				int index = int(listener.collisions.size()/2);
				// b2Vec2 closestCollision = listener.collisions[0];
				// for (b2Vec2 c: listener.collisions){
				// 	b2Vec2 d;
				// 	d.x = robot.body->GetPosition().x - c.getPosition().x;
				// 	d.y = robot.body->GetPosition().y - c.getPosition().y;
				// 	if (d.Length()< closestCollision.Length()){
					//closestCollision = c;
				// }
				//result = simResult(simResult::resultType::crashed,  Object(ObjectType::obstacle, listener.collisions[index]));
				if (type == Primitive::Type::BASELINE && step/hz >REACTION_TIME){ //stop 2 seconds before colliding so to allow the robot to explore
					//if (step/hz >REACTION_TIME){
						b2Vec2 posReadjusted;
						//OG
						//posReadjusted.x = robot.body->GetPosition().x- instVelocity.x*(REACTION_TIME);
						//posReadjusted.y = robot.body->GetPosition().y -instVelocity.y*(REACTION_TIME);
						//NEW: 
						posReadjusted.x = start.x+ instVelocity.x*(step/hz-REACTION_TIME);						
						posReadjusted.y = start.y+ instVelocity.y*(step/hz-REACTION_TIME);						
						robot.body->SetTransform(posReadjusted, _theta); //if the simulation crashes reset position for 
						result = simResult(simResult::resultType::safeForNow, Object(ObjectType::obstacle, listener.collisions[index]));
					//}
				}
				else{
						result = simResult(simResult::resultType::crashed, Object(ObjectType::obstacle, listener.collisions[index]));
						robot.body->SetTransform(start, _theta); //if the simulation crashes reset position for 
						result.collision.safeForNow =0;

					}
				//printf("collision at %f %f\n", result.collision.getPosition().x, result.collision.getPosition().y);
				//fprintf(robotDebug,"%f\t%f\n", result.collision.getPosition().x, result.collision.getPosition().y);
				break;
			}
		}	
		//result.distanceCovered = robot.body->GetPosition().Length() - start.Length();
		// if (result.resultCode==simResult::resultType::crashed){
		// 	robot.body->SetTransform(start, _theta); //if the simulation crashes reset position for 
		// }
		b2Vec2 distance; //= robot.body->GetPosition();
		distance.x = robot.body->GetPosition().x - start.x;
		distance.y = robot.body->GetPosition().y - start.y;
		result.distanceCovered = distance.Length() ;
		result.endPose = robot.body->GetTransform();
		//endPose = robot.body->GetTransform();
		//_world.DestroyBody(robot.body);		
		int roboCount=0;
		for (b2Body * b = _world.GetBodyList(); b!=NULL; b = b->GetNext()){
			if (b->GetUserData()!=NULL){
				roboCount++;
			}
			_world.DestroyBody(b);

		}
		if (debugOn){
			fclose(robotPath);
		}
		//printf("end pose x =%f, y=%f, theta = %f pi\n", result.endPose.p.x, result.endPose.p.y, result.endPose.q.GetAngle()/M_PI);
		//result.stepDuration=step;
		return result;
		//simulationResult = result;
	
}


void Primitive::trackObject(Object & object, float timeElapsed, b2Vec2 robVelocity, b2Vec2 robPos){ //isInternal refers to whether the tracking is with respect to the global coordinate frame (i.e. in willCollide) if =1, if isIntenal =0 it means that the object is tracked with the robot in the default position (0.0)
	//returns x, y, angle to the robot
	//printf("robVel %f, %f, timeelapsed %f\n", robVelocity.x, robVelocity.y, timeElapsed);
	b2Vec2 shift = {-robVelocity.x*timeElapsed, -robVelocity.y*timeElapsed}; //calculates shift in the time step
	//printf("shift: %f, %f\n", shift.x, shift.y);
	b2Vec2 newPos(object.getPosition().x+shift.x,object.getPosition().y + shift.y);
	//printf("new pos after tracking: %f, %f\n", newPos.x, newPos.y);
	object.setPosition(newPos);
//	printf("new position %f\t%f\n", object.getPosition().x, object.getPosition().y);
	float angle = object.getAngle(robVelocity);
	object.setAngle(angle); //with respect to robot's velocity
}

Primitive::controlResult Primitive::controller(){
float recordedAngle = atan(RecordedVelocity.y/RecordedVelocity.x);
    float tolerance = 0.2; //tolerance in radians (angle): 5.8 degrees circa
    if (obstacle.isValid()){
        //printf("obstacle valid\n");
        float obstacleAngle = atan(obstacle.getPosition().y/obstacle.getPosition().x);
        float angleDifference = obstacleAngle - recordedAngle;
        if (abs(angleDifference) >= M_PI_2){
			obstacle.invalidate();
            return DONE;
        }
    }
    else {
		float timeStepError =action.getOmega()*0.2 - recordedAngle; 
        accumulatedError += timeStepError; //og was new variable angleerror
		printf("acc error = %f, desired angle = %f\n", accumulatedError, action.getOmega()*0.2);
		float normAccErr = accumulatedError/M_PI;
        if (accumulatedError>=tolerance){
			printf("error at time step = %f, accumulated error = %f\n", timeStepError, accumulatedError);
            //printf("accumulated error: %f pi; correcting straight path\n\n", accumulatedError);
            action.LeftWheelSpeed -= normAccErr*pGain;  //og angle was -angle
            action.RightWheelSpeed += normAccErr *pGain; //og was + angle
            if (action.LeftWheelSpeed>1.0){
            action.LeftWheelSpeed=1.0;
            }
            if (action.RightWheelSpeed>1.0){
                action.RightWheelSpeed=1;
            }
            if (action.LeftWheelSpeed<(-1.0)){
                action.LeftWheelSpeed=-1;
            }
            if (action.RightWheelSpeed<(-1.0)){
                action.RightWheelSpeed=-1;
            }

        }
    }
    return CONTINUE;
}
