#include "state.h"
#include "robot.h"
//#include "opencv2/opencv.hpp"



State::simResult State::willCollide(b2World & _world, int _iteration){ //CLOSED LOOP CONTROL
		//crashed =false;	
		simResult result;
		Robot robot(&_world);
		Listener listener;
		_world.SetContactListener(&listener);	
		char name_r[50];
		sprintf(name_r, "/tmp/robot%04i.txt", _iteration);
		planNo++;
		FILE * robotPath = fopen(name_r, "w+");
		float theta=0;
        //float omegaT=0; //instantaneous angular velocity at time T
		//Object ob = obstacle; //copy the obstacle object for tracking
		//TO DO: 	copy trajectory and update it automatically based on obstacle distance
		//Trajectory internalTrajectory;
		b2Vec2 instVelocity;
		for (int step = 0; step <= (hz*simDuration); step++) {//3 second
			theta += trajectory.getOmega()/hz; //= omega *t
			//printf("omega = %f\t theta = %f\n", trajectory.getOmega(), theta);
			// if ((step*10/int(hz)) %2 ==0){
			// 	b2Vec2 prevVelocity = internalTrajectory.getVelocity();
			// 	internalTrajectory = Trajectory(obstacle, simDuration, maxSpeed, prevVelocity, robot.body->GetPosition()); //this simulates the behaviour of the robot as it gets further from the obstacle
			// 	dOmega ==trajectory.getOmega()/hz;
			// 	}
			//timesPlanned++;
			//theta = trajectory.getOmega()*hz;
			instVelocity.x = trajectory.getLinearSpeed()*cos(theta);
			instVelocity.y = trajectory.getLinearSpeed()*sin(theta);
			//printf("inst vel %f\t%f\n", instVelocity.x, instVelocity.y);
			// robot.setVelocity(internalTrajectory.getVelocity()); //instantaneous linear veloctiy
			robot.setVelocity(instVelocity);
			_world.Step(1.0f/hz, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
			fprintf(robotPath, "%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y); //save predictions
			if (obstacle.isValid()){ //if we are in the process of avoiding an obstacle evaluate if you can safely steer away
				// int count=1;
				// for (b2Body * b= _world.GetBodyList(); b!=NULL; b= b->GetNext()){
				// 	if (b!= robot.body){
				// 		char name[250];
				// 		sprintf(name, "/tmp/obstacle%04i", _iteration);
				// 		FILE * file = fopen(name, "w+");
				// 		fprintf(file, "%f\t%f\n", b->GetPosition().x, b->GetPosition().y);
				// 		fclose(file);
				// 	}

				// }


					// if (abs(obstacle.getAngle(robot.body->GetLinearVelocity(), robot.body->GetPosition()))>=M_PI_2 && listener.collisions.size()==0){
					// 	printf("willCollide: at step %i angle from obstacle = %f\n", step, obstacle.getAngle(robot.body->GetLinearVelocity(), robot.body->GetPosition()));
					// 	result = simResult(simResult::successful);
					// 	fclose(robotPath);
					// 	//trajectory.setSafe(1);
					// 	return result;
					// }

									//PLOT MOVEMENT OF OBSTACLE
				
				}			
			if (listener.collisions.size()>0){ //
				//find center of collision site
				//TO DO: remember as a big body
				// float midX = listener.collisions[0].getPosition().x - listener.collisions[-1].getPosition().x;
				// float midY = listener.collisions[0].getPosition().y - listener.collisions[-1].getPosition().y;
				// b2Vec2 avgCollisionPos={midX, midY}; //the x coordinate is perpendicular to the angle of the robot (c0s), y is parallel (sin)

				//GET VERTICES OF ROBOT

				// char name[250];
				// sprintf(name, "/tmp/finalRobot%04i_%i.dat", _iteration, planNo);
				// FILE * file = fopen(name, "w");
				// std::vector <cv::Point2f> vec = {cv::Point2f(-0.04, 0.085), cv::Point2f(0.04, 0.085), cv::Point2f(-0.04, -.085), cv::Point2f(0.04, -0.085)};
				// std::vector <cv::Point2f> endRobot(vec.size());
				// float turningAngle = robot.body->GetAngle();
				// cv::Mat transformMatrix = (cv::Mat_<double>(2,3)<<cos(turningAngle), -sin(turningAngle), robot.body->GetPosition().x, sin(turningAngle), cos(turningAngle), robot.body->GetPosition().y);
				// cv::transform(vec, endRobot, transformMatrix);
				// for (cv::Point2f p:endRobot){
				// 	fprintf(file, "%f\t%f\n", p.x, p.y);
				// }
				
				// fclose(file);
				int index = int(listener.collisions.size()/2);
				//printf("collisioN step %i position =%f, %f\n", step, listener.collisions[index].x, listener.collisions[index].y);
				result = simResult(simResult::crashed, _iteration, Object(ObjectType::obstacle, listener.collisions[index]));
				fclose(robotPath);

				return result;
			}


		}	
		//setPlan(_action); 		//this needs refinement as the thingy for the alphabot is hard-coded. coudl be, if one callback is 10d R or 13 to the L, get the angle from each speed in the plan and divide it
		//printf("path is safe\n");
		fclose(robotPath);
		result = simResult(simResult::successful);
		return result;
	
}


void State::trackObject(Object & object, float timeElapsed, b2Vec2 robVelocity, b2Vec2 robPos = b2Vec2(0.0,0.0)){ //isInternal refers to whether the tracking is with respect to the global coordinate frame (i.e. in willCollide) if =1, if isIntenal =0 it means that the object is tracked with the robot in the default position (0.0)
	//returns x, y, angle to the robot
	//printf("robVel %f, %f, timeelapsed %f\n", robVelocity.x, robVelocity.y, timeElapsed);
	b2Vec2 shift = {-robVelocity.x*timeElapsed, -robVelocity.y*timeElapsed}; //calculates shift in the time step
	//printf("shift: %f, %f\n", shift.x, shift.y);
	b2Vec2 newPos(object.getPosition().x+shift.x,object.getPosition().y + shift.y);
	//printf("new pos: %f, %f\n", newPos.x, newPos.y);

	object.setPosition(newPos);
//	printf("new position %f\t%f\n", object.getPosition().x, object.getPosition().y);
	float angle = object.getAngle(robVelocity, robPos);
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