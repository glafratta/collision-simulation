#include "state.h"
#include "robot.h"
//#include "opencv2/opencv.hpp"



State::simResult State::willCollide(b2World & _world, int _iteration){ //CLOSED LOOP CONTROL
		simResult result;
		Robot robot(&_world);
		Listener listener;
		_world.SetContactListener(&listener);	
		//printf("in state, world has %i bodies\n", _world.GetBodyCount());
		sprintf(planFile, "/tmp/robot%04i.txt", _iteration);
		planNo++;
		FILE * robotPath = fopen(planFile, "w+");
		float theta=0;
		b2Vec2 instVelocity;
		for (int step = 0; step <= (hz*simDuration); step++) {//3 second
			theta += action.getOmega()/hz; //= omega *t
			instVelocity.x = action.getLinearSpeed()*cos(theta);
			instVelocity.y = action.getLinearSpeed()*sin(theta);
			robot.setVelocity(instVelocity);
			_world.Step(1.0f/hz, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
			fprintf(robotPath, "%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y); //save predictions
			if (listener.collisions.size()>0){ //
				int index = int(listener.collisions.size()/2);
				result = simResult(simResult::crashed, _iteration, Object(ObjectType::obstacle, listener.collisions[index]));
			}
			else{
			result = simResult(simResult::successful);
			}


		}	
		fclose(robotPath);
		return result;
	
}


void State::trackObject(Object & object, float timeElapsed, b2Vec2 robVelocity, b2Vec2 robPos){ //isInternal refers to whether the tracking is with respect to the global coordinate frame (i.e. in willCollide) if =1, if isIntenal =0 it means that the object is tracked with the robot in the default position (0.0)
	//returns x, y, angle to the robot
	//printf("robVel %f, %f, timeelapsed %f\n", robVelocity.x, robVelocity.y, timeElapsed);
	b2Vec2 shift = {-robVelocity.x*timeElapsed, -robVelocity.y*timeElapsed}; //calculates shift in the time step
	//printf("shift: %f, %f\n", shift.x, shift.y);
	b2Vec2 newPos(object.getPosition().x+shift.x,object.getPosition().y + shift.y);
	printf("new pos after tracking: %f, %f\n", newPos.x, newPos.y);
	object.setPosition(newPos);
//	printf("new position %f\t%f\n", object.getPosition().x, object.getPosition().y);
	float angle = object.getAngle(robVelocity);
	object.setAngle(angle); //with respect to robot's velocity
}

// void State::controller(){
// 	if (obstacle.isValid()){
// 		return; //if there is an obstacle do not try to follow a trajectory
// 	}
// 	else {
// 		b2Vec2 desiredPosition, nextPosition, recordedPosition; //target for genearting a corrective trajectory
// 		double angleError=0;
// 		double distanceError=0;
// 		if (iteration > 0){
// 			float x,y, t;
// 			t=0; //discrete time
// 			char name[50];
// 			sprintf(name, "", fileNameBuffer); //
// 			//printf("%s\n", name);
// 			std::ifstream file(name);

// 			while (file>>x>>y){ 
// 				t= t+ 1.0f/60.0f;
// 				if(totalTime<t && t<=(totalTime+1/60.f)){ 
// 					desiredPosition = b2Vec2(x,y);
// 					printf("desired position out of file: %f, %f\t time recorded as: %f, timeElapsed: %f\n", x, y,t, timeElapsed);
// 					break;
// 				}
// 				// else {
// 				//     leftWheelSpeed = 0;
// 				//     rightWheelSpeed = 0;
// 				// }

// 			}

// 			file.close();
// 			//desiredVelocity =b2Vec2(desiredPosition.x/timeElapsed, desiredPosition.y/timeElapsed);
// 			recordedPosition = {state->getRecordedVelocity().x, state->getRecordedVelocity().y};
// 			printf("recordedpos = %f, %f\n", recordedPosition.x, recordedPosition.y);
// 			//float desiredAngle = atan2(recordedPosition.y)
// 		// angleError = atan2(desiredPosition.x, desiredPosition.y)-atan2(recordedPosition.x, recordedPosition.y); //flag
// 			angleError = atan2(desiredPosition.y, desiredPosition.x)-atan2(recordedPosition.y, recordedPosition.x); //flag    
// 			//normalise error
// 			double maxError = 2* M_PI;
// 			angleError /= maxError;


// 			printf("desired angle: %f pi\t recorded angle: %f pi\n", atan2(desiredPosition.y, desiredPosition.x)/M_PI, atan2(recordedPosition.y, recordedPosition.x)/M_PI);
// 			//printf("desired position = %f, %f\trecorded position: %f, %f\n", desiredPosition.x, desiredPosition.y, recordedPosition.x, recordedPosition.y);
// 			printf("angleError =%f\n", angleError);
// 			distanceError = desiredPosition.Length() - recordedPosition.Length();
// 			printf("distanceError = %f\n", distanceError);
// 			}

// 		leftWheelSpeed = state->getTrajectory().getLWheelSpeed() + angleError*gain+ distanceError*gain;  //og angle was +angle
// 		rightWheelSpeed = state->getTrajectory().getRWheelSpeed()- angleError *gain + distanceError*gain; //og was - angle

// 		if (leftWheelSpeed>1.0){
// 			leftWheelSpeed=1.0;
// 		}
// 		if (rightWheelSpeed>1.0){
// 			rightWheelSpeed=1;
// 		}
// 		if (leftWheelSpeed<(-1.0)){
// 			leftWheelSpeed=-1;
// 		}
// 		if (rightWheelSpeed<(-1.0)){
// 			rightWheelSpeed=1;

// 		}

// 	}
//}