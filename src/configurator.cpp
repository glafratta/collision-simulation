#pragma once
#include "configurator.h"
#include <chrono>
#include <iostream>

void Configurator::NewScan(){ 
		iteration++;
		//printf("begin newScan plan size %i\n", plan.size());
		bool isObstacleStillThere=0;
		//see time delay between scans to calculate speed more accurately

        auto now =std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli>diff= now - previousTimeScan; //in seconds
		timeElapsed=float(diff.count())/1000; //express in seconds
		printf("time elapsed between newscans = %f ms\n", timeElapsed);
		if ( timeElapsed< samplingRate){
			timeElapsed= samplingRate;
		}
		totalTime += timeElapsed; //for debugging
		//printf("time elapsed = %f ms\n", timeElapsed);
		previousTimeScan=now; //update the time of sampling
		//WRITE TO FILE (debug) + WORLD BUILDING
		b2World world = b2World({0.0f,0.0f});
		char name[256];
		// sprintf(name, "/tmp/bodies%04d.txt", iteration);
		// FILE *file= fopen(name, "w+");
		// printf("%s\n", name);
		
		//put velocity calculations before world creation so that we can check if obstacle is still there
		
		Configurator::getVelocityResult affineTransResult= GetRealVelocity();
		b2Vec2 estimatedVelocity= affineTransResult.displacement;
		absPosition.x += estimatedVelocity.x;
		absPosition.y += estimatedVelocity.y;
		dumpPath = fopen("/tmp/dumpPath.txt", "a+");
		fprintf(dumpPath, "%f\t%f\n", absPosition.x, absPosition.y);
		fclose(dumpPath);
		//printf("estimatedVelocity = %f, %f\n", estimatedVelocity.x, estimatedVelocity.y);
		if (affineTransResult.valid){
			estimatedVelocity = affineTransResult.displacement;
		}
		else{
			estimatedVelocity = {0.0f, 0.0f};
			//return 2; //if returns 2 it means its time to panic
		}
		//if obstacles are out of sight, clear them from memory
		if (plan.size()>0 && plan[0].obstacle.isValid()){
			//State state = plan[0];
			

			//UPDATE OBSTACLE POSITION
				//returns x, y, angle to the robot
		// 	b2Vec2 shift = {-estimatedVelocity.x*timeElapsed, -estimatedVelocity.y*timeElapsed}; //calculates shift in the time step
		// 	b2Vec2 newPos(plan[0].obstacle.getPosition().x+shift.x, plan[0].obstacle.getPosition().y + shift.y);
		// 	plan[0].obstacle.getPosition().Set(newPos.x, newPos.y);
		// //	printf("new position %f\t%f\n", object.getPosition().x, object.getPosition().y);
		// 	float angle = plan[0].obstacle.getAngle(estimatedVelocity, b2Vec2(0.0f, 0.0f));
		// 	plan[0].obstacle.setAngle(angle); //with respect to robot's velocity






			plan[0].trackObject(plan[0].obstacle, timeElapsed, estimatedVelocity, {0.0f, 0.0f}); //robot default position is 0,0
//			printf("new obstacle position %f, %f and angle: %f", plan[0].obstacle.getPosition().x, plan[0].obstacle.getPosition().y, plan[0].obstacle.getAngle());
			//printf("object angle after tracking: %f", state.obstacle.getAngle(estimatedVelocity, b2Vec2(0.0f, 0.0f))); //default robot pos is origin
			//plan[0].obstacle.setAngle(plan[0].obstacle.getAngle(estimatedVelocity, b2Vec2(0.0f, 0.0f)));
			if (plan[0].obstacle.getAngle() >= M_PI_2){ 		//find angle between robot body and collision 		//if angle larger than 90 deg delete this state in the plan and recheck
				plan.erase(plan.begin());
				printf("erased plan\n");
			}
			else{
//				state.recomputeTrajectory(); //update the trajectory based on the obstacle relative position after movign
				State::Object temp = plan[0].obstacle;
				plan[0]= State(temp);
				// printf("is obstacle valid: %i\n", plan[0].obstacle.isValid());

				// printf("plan should be same size: %i\n", plan.size());
			}

		}






		//printf("before making body\n");
		for (int i=0; i<current.size(); i++){ //makes obstacles and checks for duplicates
			b2Vec2 pointCurrent(roundf((current[i].x)*100.f)/100.f, roundf((current[i].y)*100.f)/100.f);
			b2Vec2 pointPrevious(roundf((current[i-1].x)*100.f)/100.f, roundf((current[i-1].y)*100.f)/100.f);

			if ((pointCurrent.x != pointPrevious.x || pointCurrent.y!=pointPrevious.y)&& pointCurrent.y>=0){ // robot only sees obstacles ahead of it
				//printf("making body!\n");
				b2Body * body;
				b2BodyDef bodyDef;
				b2FixtureDef fixtureDef;
				bodyDef.type = b2_dynamicBody;
				b2PolygonShape fixture; //giving the point the shape of a box
				fixtureDef.shape = &fixture;
				fixture.SetAsBox(.001f, .001f); 
				if (pointCurrent == b2Vec2(0.0f, 0.0f)){
					printf("obstacle at 0,0, not normal\n");
					printf("body n. %i\n", i);
				}
				//can get transform from body
			//printf("plan size %i\n", plan.size());
				if (plan.size()>0 && plan[0].obstacle.isValid()){
					//printf("plan is there and obstacle valid\n");
					// printf("point %f, %f\n", pointCurrent.x, pointCurrent.y);
					// printf("pointCurrent.x > plan[0].obstacle.getPosition().x-0.05 : %i\n ", pointCurrent.x > plan[0].obstacle.getPosition().x-0.05);
					// printf("pointCurrent.x < plan[0].obstacle.getPosition().x+0.05 : %i\n ", pointCurrent.x < plan[0].obstacle.getPosition().x+0.05);
					// printf("pointCurrent.y > plan[0].obstacle.getPosition().y-0.05 : %i\n ", pointCurrent.y > plan[0].obstacle.getPosition().y-0.05);
					// printf("pointCurrent.y < plan[0].obstacle.getPosition().y+0.05 : %i\n ", pointCurrent.y < plan[0].obstacle.getPosition().y+0.05);
					//printf("all together: %i", (((plan[0].obstacle.getPosition().x)-0.05<pointCurrent.x<(plan[0].obstacle.getPosition().x)+0.05) and ((plan[0].obstacle.getPosition().y)-0.05<pointCurrent.y<(plan[0].obstacle.getPosition().y)+0.05)));

					if (pointCurrent.x > plan[0].obstacle.getPosition().x-0.05 && pointCurrent.x < plan[0].obstacle.getPosition().x+0.05 && pointCurrent.y > plan[0].obstacle.getPosition().y-0.05 && pointCurrent.y < plan[0].obstacle.getPosition().y+0.05){
						isObstacleStillThere =1;
						//printf("changed still there\n");
					}
				}
					bodyDef.position.Set(pointCurrent.x, pointCurrent.y); 
					body = world.CreateBody(&bodyDef);
					body->CreateFixture(&fixtureDef);
					//fprintf(file, "%f\t%f\n", body->GetPosition().x, body->GetPosition().y);

			}
		}
		printf("body count = %i\n", world.GetBodyCount());
		//fclose(file);
		printf("obstacle is still there: %i\n", isObstacleStillThere);
		//printf("bodies created\n");
		if (!isObstacleStillThere && plan.size()>0){
			plan.erase(plan.begin());
		}

		current.clear();

		//once the representation is updated see if you're going to crash

		//DONE	//willCollide should follow a trajectory determined by the position of the obstacle


		//if no obstacles to keep track of, check if with the current trajectory it will bump into something else
		//if the plan vector is empty the state is the default state
		printf("plan size %i\n", plan.size());
		if (plan.size() ==0){
			printf("state is desired state\n");
		desiredState.setRecordedVelocity(estimatedVelocity); //logs in previous recorded velocity in case affine transformation fails
		//TO DO		//generate error from estimatedVelocity
		State::simResult result =desiredState.willCollide(world, iteration);
			if (result.resultCode == State::simResult::crashed){
				State newState(result.collision);
				//generate new trajectory for this state: AUTOMATIC in state constructor
				plan.push_back(newState); //if state has one or more obstacles it is "avoid" state
				//TO DO: //start computing a plan to avoid obstacle (trajectory) -just obstacle
				printf("collided, new state has trajectory omega= %f,  linear speed: %f\n", newState.trajectory.getOmega(), newState.trajectory.getLinearSpeed() );
			}
		}
		else if (plan.size()>0){ //the program enters this loop both if a disturbance has just been detected and if it was previously detected
			plan[0].setRecordedVelocity(estimatedVelocity);
			 //check if the trajectory being followed will bump the robot into something
			State::simResult result =plan[0].willCollide(world, iteration);
			if (result.resultCode == State::simResult::crashed){
				State newState(result.collision);
				//generate new trajectory for this state: AUTOMATIC in state constructor
				plan.push_back(newState); //if state has one or more obstacles it is "avoid" state
				//TO DO: //start computing a plan to avoid obstacle (trajectory) -just obstacle
				printf("plan[0] collided, new state has trajectory omega= %f, linear speed: %f\n", newState.trajectory.getOmega(), newState.trajectory.getLinearSpeed() );
			}

		}

		
		//compare detected velocity with predicted velocity and correct motor output to minimize error


	}

Configurator::getVelocityResult Configurator::GetRealVelocity(){		
		getVelocityResult result;
		//std::vector <cv::Point2f> currentTmp =current;
		std::vector <cv::Point2f> previousTmp;

		if (iteration >0){
			char prevPath[256];
			sprintf(prevPath, "%s/transmap%04d.dat", folder, iteration);
        	//printf("getting previous\n");
			std::ifstream prev(prevPath);
			float x,y;
			while (prev>>x>>y){
				previousTmp.push_back(cv::Point2f(x,y));
			}
			prev.close();
		}

		int diff = current.size()-previousTmp.size(); //if +ve,current is bigger, if -ve, previous is bigger
		//printf("current: %i, previous: %i\n", current.size(), previousTmp.size());


        //adjust for discrepancies in vector size		//int diff = currSize-prevSize;
		if(diff>0){ //(current.size()>previous.size()){
			if (previousTmp.empty()){
				for (cv::Point p:current){
					previousTmp.push_back(cv::Point());
				} }
			else{
				for (int i=0; i<abs(diff); i++){
					previousTmp.push_back(previousTmp[0]); //before it was [-1]
				if (previousTmp[-1].x == 0 && previousTmp[-1].y ==0){
					printf("previous fucked up\n");
				}

			}
			}
		}
	
		else if (diff<0){//(current.size()<previous.size()){
			if (current.empty()){
				for (cv::Point p:previousTmp){
					current.push_back(cv::Point());
					printf("no data");
					return result;
				} }
			else{
				for (int i=0; i<abs(diff); i++){
			current.push_back(current[0]);
				if (current[-1].x == 0 && current[-1].y ==0){
					printf("current fucked up\n");
				}

				}
		}
		//printf("p>curr\n");
		}

	//use partial affine transformation to estimate displacement
		cv::Mat transformMatrix = cv::estimateAffinePartial2D(previousTmp, current, cv::noArray(), cv::LMEDS);
		// //printf("currentTmp\n");
		// for (cv::Point2f p: currentTmp){
		// 	//printf("%f, %f\n", p.x, p.y);
		// }
		// //printf("previousTmp\n");
		// for (cv::Point2f p: previousTmp){
		// 	//printf("%f, %f\n", p.x, p.y);
		// }


			//printf("matrix is empty? %i\n", transformMatrix.empty());
			if (!transformMatrix.empty()){
				b2Vec2 tmp;
				tmp.x= -(transformMatrix.at<double>(0,2))/samplingRate;
				tmp.y = -(transformMatrix.at<double>(1,2))/samplingRate;
				// if (abs(tmp.Length())>maxAbsSpeed){
				// 	tmp=defaultSpeed;
				// }
				//printf("matrix not empty\n");
				//printf("get velocity from affine trans\n");
				return getVelocityResult(tmp);
			}
			else if (plan.empty()){ //if the plan is empty look at the default wheel speed
				b2Vec2 estimatedVel;
				estimatedVel =desiredState.trajectory.getVelocity();
				result = getVelocityResult(estimatedVel);
				//printf("getting velocity from desired state\n");
				return result;
			}
			else if (plan.size() >0 && transformMatrix.empty()){ //if an obstacle is being avoided use the set speed
			b2Vec2 estimatedVel = plan[0].trajectory.getVelocity();
				result = getVelocityResult(estimatedVel);
				//printf("getting velocity from plan state\n");
				return result;
			}
			else{
				printf("could not find velocity\n");
				return result;
			}
		}
	//}

// 	void Configurator::controller(){
// 	//FIND ERROR
// 	b2Vec2 desiredPosition, nextPosition, recordedPosition; //target for genearting a corrective trajectory
// 	printf("iteration =%i\n", iteration);
// 	State * state;
// 		if (!plan.empty()){
// 			state = &(plan[0]);
// 		}
// 		else if (plan.empty()){
// 			state = & desiredState;
// 		}



// 	float angleError=0;
// 	float distanceError=0;
// 	if (iteration >=1){
// 		float x,y, t;
// 		t=0; //discrete time
// 		char name[50];
// 		sprintf(name, "/tmp/robot%04i.txt", iteration -1); //
// 		//printf("%s\n", name);
// 		std::ifstream file(name);


// 		while (file>>x>>y){
// 			t= t+ 1.0f/60.0f;
// 			if(timeElapsed<t && t<=(timeElapsed+1/60.f)){ 
// 				desiredPosition = b2Vec2(x,y);
// 				printf("desired position out of file: %f, %f\t time recorded as: %f, timeElapsed: %f\n", x, y,t, timeElapsed);
// 				break;
// 			}
// 			else if (timeElapsed*2<=t<=(timeElapsed*2+1/(state->hz))){ //next position
// 				nextPosition = b2Vec2(x,y);
// 			}
// 		}
// 		file.close();
// 		desiredVelocity =b2Vec2(desiredPosition.x/timeElapsed, desiredPosition.y/timeElapsed);
// 		recordedPosition = {state->getRecordedVelocity().x*timeElapsed, state->getRecordedVelocity().y*timeElapsed};
// 		//float desiredAngle = atan2(recordedPosition.y)
// 		angleError = atan2(desiredPosition.x, desiredPosition.y)-atan2(recordedPosition.x, recordedPosition.y); //flag
// 		printf("desired position = %f, %f\trecorded position: %f, %f\n", desiredPosition.x, desiredPosition.y, recordedPosition.x, recordedPosition.y);
// 		printf("angleError =%f\n", angleError);
// 		distanceError = desiredPosition.Length() - recordedPosition.Length();
// 		printf("distanceError = %f\n", distanceError);
// 		}

// 	leftWheelSpeed = state->getTrajectory().getLWheelSpeed() + angleError*gain+ distanceError*gain;
// 	rightWheelSpeed = state->getTrajectory().getRWheelSpeed()- angleError *gain + distanceError*gain; 

// 	//control function = v0 + error *gain

// 	//Generate corrective trajectory?
// 	// Object target(ObjectType::target, targetPosition);
// 	// trajectory= Object(target, simDuration, maxSpeed);


// }