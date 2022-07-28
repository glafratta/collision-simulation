#pragma once
#include "configurator.h"
#include <chrono>
#include <iostream>

void Configurator::NewScan(){ 
		iteration++;
		//see time delay between scans to calculate speed more accurately

        auto now =std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli>diff= now - previousTimeScan; //in seconds
		float timeElapsed=float(diff.count())/1000; //express in seconds
		if ( timeElapsed< samplingRate){
			timeElapsed= samplingRate;
		}
		printf("time elapsed = %f ms\n", timeElapsed);
		//std::cout<<"sampling rate = "<<timeElapsed<<std::endl;
		previousTimeScan=now; //update the time of sampling
		//WRITE TO FILE (debug) + WORLD BUILDING
		b2World world = b2World({0.0f,0.0f});
		char name[256];
		sprintf(name, "/tmp/bodies%04d.txt", iteration);
		FILE *file= fopen(name, "w+");
		printf("%s\n", name);
		
		//printf("before making body\n");
		for (int i=0; i<current.size(); i++){ //makes obstacles and checks for duplicates
			b2Vec2 pointCurrent(roundf((current[i].x)*100.f)/100.f, roundf((current[i].y)*100.f)/100.f);
			//printf("pointCurrent = %f, %f\n", pointCurrent.x, pointCurrent.y);
			b2Vec2 pointPrevious(roundf((current[i-1].x)*100.f)/100.f, roundf((current[i-1].y)*100.f)/100.f);
			//printf("pointPrevious = %f, %f\n", pointPrevious.x, pointPrevious.y);
			//printf("is pointcurrent same as pointPrevious %i\n", pointCurrent.x != pointPrevious.x && pointCurrent.y!=pointPrevious.y);
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
					printf("weird!\n");
					printf("pointCurrent = %f, %f\n", pointCurrent.x, pointCurrent.y);
					printf("body n. %i\n", i);
				}
					bodyDef.position.Set(pointCurrent.x, pointCurrent.y); 
					body = world.CreateBody(&bodyDef);
					body->CreateFixture(&fixtureDef);
					fprintf(file, "%f\t%f\n", body->GetPosition().x, body->GetPosition().y);

			}
		}
		printf("body count = %i\n", world.GetBodyCount());
		fclose(file);
		//printf("bodies created\n");
		Configurator::getVelocityResult affineTransResult= GetRealVelocity();
		b2Vec2 estimatedVelocity= affineTransResult.displacement;
		printf("estimatedVelocity = %f, %f\n", estimatedVelocity.x, estimatedVelocity.y);
		if (affineTransResult.valid){
			estimatedVelocity = affineTransResult.displacement;
		}
		else{
			estimatedVelocity = {0.0f, 0.0f};
			//return 2; //if returns 2 it means its time to panic
		}
		//if obstacles are out of sight, clear them from memory
		if (plan.size()>0){
			b2Vec2 objectShift = {-estimatedVelocity.x*timeElapsed, -estimatedVelocity.y*timeElapsed}; //calculates shift in the time step
			State state = plan[0];
			
			state.trackObject(state.obstacle, timeElapsed, {0,0}, objectShift); //robot default position is 0,0
			printf("object angle after tracking: %f", state.obstacle.getAngle());
			if (state.obstacle.getAngle() >= M_PI_2){ 		//find angle between robot body and collision 		//if angle larger than 90 deg delete this state in the plan and recheck
				plan.erase(plan.begin());
			}
			else{
				//state.obstacle.setPosition({objectPose.x, objectPose.y}); //else update the location of the object represented and keep them in memory
				state.recomputeTrajectory(); //update the trajectory based on the obstacle relative position after movign

				//TO DO 	OPTIONAL//check if the obstacle is still there in the box2d thing, if not clear it (probably using a standard error for it); now using a range of +/- 5 cm
				//for (body in bodies00x.dat){if (body.coordinates > +- setrange; delete obstacle)}  but probably can be done more efficiently. This is probably better with 
				//point clustering and extraction of lines. Maybe worth looking into lines or smth in box2d
			}

			}
		

		//once the representation is updated see if you're going to crash

		//DONE	//willCollide should follow a trajectory determined by the position of the obstacle


		//if no obstacles to keep track of, check if with the current trajectory it will bump into something else
		//if the plan vector is empty the state is the default state

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
		int diff = current.size()-previous.size(); //if +ve,current is bigger, if -ve, previous is bigger
		std::vector <cv::Point2f> currentTmp =current;
		std::vector <cv::Point2f> previousTmp=previous;
        //adjust for discrepancies in vector size		//int diff = currSize-prevSize;
		if(diff>0){ //(current.size()>previous.size()){
			if (previousTmp.empty()){
				for (cv::Point p:currentTmp){
					previousTmp.push_back(cv::Point());
				} }
			else{
				for (int i=0; i<abs(diff); i++){
					previousTmp.push_back(previousTmp[-1]);
			}
			}
		}
	
		else if (diff<0){//(current.size()<previous.size()){
			if (currentTmp.empty()){
				for (cv::Point p:previousTmp){
					currentTmp.push_back(cv::Point());
					printf("no data");
					return result;
				} }
			else{
				for (int i=0; i<abs(diff); i++){
			currentTmp.push_back(currentTmp[-1]);

				}
		}
		//printf("p>curr\n");
		}

	//use partial affine transformation to estimate displacement
		cv::Mat transformMatrix = cv::estimateAffinePartial2D(previousTmp, currentTmp, cv::noArray(), cv::LMEDS);
		//printf("currentTmp\n");
		for (cv::Point2f p: currentTmp){
			//printf("%f, %f\n", p.x, p.y);
		}
		//printf("previousTmp\n");
		for (cv::Point2f p: previousTmp){
			//printf("%f, %f\n", p.x, p.y);
		}



			if (!transformMatrix.empty()){
				b2Vec2 tmp;
				tmp.x= -(transformMatrix.at<double>(0,2))/samplingRate;
				tmp.y = -(transformMatrix.at<double>(1,2))/samplingRate;
				// if (abs(tmp.Length())>maxAbsSpeed){
				// 	tmp=defaultSpeed;
				// }
				//printf("matrix not empty\n");
				return getVelocityResult(tmp);
			}
			else if (plan.empty()){ //if the plan is empty look at the default wheel speed
				b2Vec2 estimatedVel;
				estimatedVel =desiredState.trajectory.getVelocity();
				result = getVelocityResult(estimatedVel);
				//printf("getting velocity from desired state\n");
				return result;
			}
			else if (plan.size() >0){ //if an obstacle is being avoided use the set speed
			b2Vec2 estimatedVel = plan[0].trajectory.getVelocity();
				result = getVelocityResult(estimatedVel);
				printf("getting velocity from plan state\n");
				return result;
			}
			else{
				printf("could not find velocity\n");
				return result;
			}
		}
	//}