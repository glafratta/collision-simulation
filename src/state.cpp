#include "state.h"
#include "robot.h"
#include "opencv2/opencv.hpp"



State::simResult State::willCollide(b2World & _world, int _iteration){ //CLOSED LOOP CONTROL
		//crashed =false;	
		simResult result;
		Robot robot(&_world);
		Listener listener(_iteration);
		_world.SetContactListener(&listener);	
		char name_r[50];
		sprintf(name_r, "/tmp/robot%04d_%i.txt", _iteration, timesPlanned);
		//planNo++;
		FILE * robotPath = fopen(name_r, "w+");
        float dOmega;
        //float omegaT=0; //instantaneous angular velocity at time T
		//Object ob = obstacle; //copy the obstacle object for tracking
		//TO DO: 	copy trajectory and update it automatically based on obstacle distance
		Trajectory internalTrajectory;
		for (int step = 0; step <= (hz*simDuration); step++) {//3 second
			if ((step*10/int(hz)) %2 ==0){
				b2Vec2 prevVelocity = internalTrajectory.getVelocity();
				internalTrajectory = Trajectory(obstacle, simDuration, maxSpeed, prevVelocity, robot.body->GetPosition()); //this simulates the behaviour of the robot as it gets further from the obstacle
				dOmega ==trajectory.getOmega()/hz;
				//printf("recomuputing trajectory at multuple of .2s\n");
				}
			timesPlanned++;
            // float dx= internalTrajectory.getLinearSpeed()*(-sin(internalTrajectory)); //recorded velocity because we assume that the linear velocity is constant in a state
            // float dy = internalTrajectory.getLinearSpeed()*cos(omegaT);
			//printf("velocity: %f, %f", dx, dy);
			// b2Vec2 instVel(dx, dy);
			//b2Vec2 prevPosition = robot.body->GetPosition();
			robot.setVelocity(internalTrajectory.getVelocity()); //instantaneous linear veloctiy
			_world.Step(1.0f/hz, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
			fprintf(robotPath, "%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y); //save predictions
			//printf("%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y);
			//recompute trajectory at the .2 ms mark (when it would be recomputed by newScan)

            //omegaT+=dOmega;
			//b2Vec2 shiftIncrement(0.0f,0.0f);
			if (obstacle.isValid()){ //if we are in the process of avoiding an obstacle evaluate if you can safely steer away
				// //find distance vector from robot COM to obstacle COM
				// //b2Vec2 prevDistance(robot.body->GetPosition().x - ob.getPosition().x, robot.body->GetPosition().y - ob.getPosition().y);
				// //perpendicular : m of one vector is equal to -m of the other
				// //slope of line passing through obstacle and robot
				// float m = (robot.body->GetPosition().y -obstacle.getPosition().y )/(robot.body->GetPosition().x -obstacle.getPosition().x) ; 
				// //slope of line passing through previous robot position and current position
				// float m2 = (robot.body->GetPosition().y-prevPosition.y)/(robot.body->GetPosition().x -prevPosition.x);
				// //if distance vector forms 90 degree with velocity vector break
				// if (m+m2 >=0 ){ //if they form an angle of 90 deg or more
				//printf("body count in willcollide: %i", _world.GetBodyCount());
				int count=1;
				for (b2Body * b= _world.GetBodyList(); b!=NULL; b= b->GetNext()){
					//printf("entered for loop\n");
					if (b!= robot.body){
						//printf("hi im here\n");
						char name[250];
						sprintf(name, "/tmp/obstacle%04i", _iteration);
						FILE * file = fopen(name, "w+");
						fprintf(file, "%f\t%f\n", b->GetPosition().x, b->GetPosition().y);
						fclose(file);
					}

				}


					if (abs(obstacle.getAngle())>=M_PI_2 && listener.collisions.size()==0){
						printf("willCollide: angle from obstacle = %f\n", obstacle.getAngle());
						result = simResult(simResult::successful);
						fclose(robotPath);
						trajectory.setSafe(1);
						return result;
					}

									//PLOT MOVEMENT OF OBSTACLE
				
				}			
			if (listener.collisions.size()>0){ //
				//find center of collision site
				//TO DO: remember as a big body
				float midX = listener.collisions[0].getPosition().x - listener.collisions[-1].getPosition().x;
				float midY = listener.collisions[0].getPosition().y - listener.collisions[-1].getPosition().y;
				b2Vec2 avgCollisionPos={midX, midY}; //the x coordinate is perpendicular to the angle of the robot (c0s), y is parallel (sin)
				printf("collided at step %i\n", step);
				printf("robot angle: %f\n", robot.body->GetAngle());

				//GET VERTICES OF ROBOT

				char name[250];
				sprintf(name, "/tmp/finalRobot%04i_%i.dat", _iteration, timesPlanned);
				FILE * file = fopen(name, "w");
				std::vector <cv::Point2f> vec = {cv::Point2f(-0.04, 0.085), cv::Point2f(0.04, 0.085), cv::Point2f(-0.04, -.085), cv::Point2f(0.04, -0.085)};
				std::vector <cv::Point2f> endRobot(vec.size());
				float turningAngle = robot.body->GetAngle();
				cv::Mat transformMatrix = (cv::Mat_<double>(2,3)<<cos(turningAngle), -sin(turningAngle), robot.body->GetPosition().x, sin(turningAngle), cos(turningAngle), robot.body->GetPosition().y);
				cv::transform(vec, endRobot, transformMatrix);
				for (cv::Point2f p:endRobot){
					fprintf(file, "%f\t%f\n", p.x, p.y);
				}
				
				fclose(file);
				result = simResult(simResult::crashed, _iteration, Object(ObjectType::obstacle, avgCollisionPos));
				fclose(robotPath);

				return result;
			}


		}	
		//setPlan(_action); 		//this needs refinement as the thingy for the alphabot is hard-coded. coudl be, if one callback is 10d R or 13 to the L, get the angle from each speed in the plan and divide it
		printf("path is safe\n");
		fclose(robotPath);
		result = simResult(simResult::successful);
		return result;
	
}


void State::trackObject(Object & object, float stepSize, b2Vec2 robotPos, b2Vec2 shift){ //isInternal refers to whether the tracking is with respect to the global coordinate frame (i.e. in willCollide) if =1, if isIntenal =0 it means that the object is tracked with the robot in the default position (0.0)
	//returns x, y, angle to the robot
	float x = object.getPosition().x+shift.x; //stepsize in seconds
	float y = object.getPosition().y + shift.y;
	object.setPosition(x,y);
	object.setAngle(-atan2(x, y)); //the angle is the distance vector from the robot's center of mass to the obstacle's center of mass
}

