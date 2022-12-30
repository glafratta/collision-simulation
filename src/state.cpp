#include "state.h"
#include "robot.h"
//#include "opencv2/opencv.hpp"



State::simResult State::willCollide(b2World & _world, int _iteration, b2Vec2 start = b2Vec2(), float _theta=0.0){ //CLOSED LOOP CONTROL, og return simreult
		simResult result = simResult(simResult::resultType::successful);
		Robot robot(&_world);
		Listener listener;
		_world.SetContactListener(&listener);	
		//printf("in state, world has %i bodies\n", _world.GetBodyCount());
		sprintf(planFile, "/tmp/robot%04i.txt", _iteration);
		FILE * robotPath = fopen(planFile, "a");
		//char debug[250];
		//sprintf(debug, "/tmp/collision%04i.txt", _iteration);
		//FILE * robotDebug = fopen(debug, "w");
		//float theta=0;
		float theta = _theta;
		printf("starting from x =%f, y=%f, theta = %fpi \n", start.x, start.y, theta/M_PI);
		b2Vec2 instVelocity = {0,0};
		robot.body->SetTransform(start, theta);
		//printf("entering for loop\n");
		int step=0;
		printf("speed = %f\n", RecordedVelocity.Length());
		for (step; step <= (hz*simDuration); step++) {//3 second
			instVelocity.x = RecordedVelocity.Length()*cos(theta); //integrate?
			instVelocity.y = RecordedVelocity.Length()*sin(theta);
			robot.body->SetLinearVelocity(instVelocity);
			robot.body->SetAngularVelocity(action.getOmega());
			robot.body->SetTransform(robot.body->GetPosition(), theta);
			fprintf(robotPath, "%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y); //save predictions
			_world.Step(1.0f/hz, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
			theta += action.getOmega()/hz; //= omega *t
			if (obstacle.isValid()){
				//printf("robot angle = %f pi\n", robot.body->GetAngle()/M_PI);
				if (abs(obstacle.getAngle(robot.body->GetAngle()))>=M_PI_2){
				//printf("obstacle successfully avoided after %i steps\n", step);
				break;
				}
			}
			if (listener.collisions.size()>0){ //
				int index = int(listener.collisions.size()/2);
				result = simResult(simResult::resultType::crashed, _iteration, Object(ObjectType::obstacle, listener.collisions[index]));
				robot.body->SetTransform(start, _theta); //if the simulation crashes reset position for 
				printf("collision at %f %f\n", result.collision.getPosition().x, result.collision.getPosition().y);
				//fprintf(robotDebug,"%f\t%f\n", result.collision.getPosition().x, result.collision.getPosition().y);
				break;
			}

		}	
		//printf("exited for loop after %i steps\n", step);
		// else{
		// 	result = simResult(simResult::resultType::successful);
		// 	//printf("no collisions\n");
		// }
		//printf("robot pose : (%f, %f, %f pi)\n", robot.body->GetPosition().x, robot.body->GetPosition().y, robot.body->GetAngle()/M_PI);
		//fclose(robotDebug);
		_world.DestroyBody(robot.body);		
		fclose(robotPath);
		endPose = robot.body->GetTransform();
		printf("end pose x =%f, y=%f, theta = %f pi\n", endPose.p.x, endPose.p.y, endPose.q.GetAngle()/M_PI);
		result.stepDuration=step;
		return result;
		//simulationResult = result;
	
}


void State::trackObject(Object & object, float timeElapsed, b2Vec2 robVelocity, b2Vec2 robPos){ //isInternal refers to whether the tracking is with respect to the global coordinate frame (i.e. in willCollide) if =1, if isIntenal =0 it means that the object is tracked with the robot in the default position (0.0)
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

State::controlResult State::controller(){
float recordedAngle = atan(RecordedVelocity.y/RecordedVelocity.x);
    float tolerance = 0.2; //tolerance in radians (angle): 5.8 degrees circa
    if (obstacle.isValid()){
        printf("obstacle valid\n");
        float obstacleAngle = atan(obstacle.getPosition().y/obstacle.getPosition().x);
        float angleDifference = obstacleAngle - recordedAngle;
        if (abs(angleDifference) >= M_PI_2){
			obstacle.invalidate();
            return DONE;
        }
    }
    else {
        accumulatedError += action.getOmega()*0.2 - recordedAngle; //og was new variable angleerror
		float normAccErr = accumulatedError/M_PI;
        if (accumulatedError>=tolerance){
            printf("accumulated error: %f pi; correcting straight path\n\n", accumulatedError);
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
