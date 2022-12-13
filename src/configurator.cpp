#pragma once
#include "configurator.h"
#include <chrono>
#include <iostream>

void Configurator::NewScan(std::vector <Point> & data){ 
	//PREPARE VECTORS TO RECEIVE DATA
	iteration++; //iteration set in getVelocity
	std::vector <Point> previous;
	for (Point p:current){
		previous.push_back(p);
	}
	current.clear();
	for (Point p:data){
		current.push_back(p);
	}
	printf("data = %i\n", current.size());
	//printf("current length: %i, previous length%i\n", current.size(), previous.size());

	//BENCHMARK + FIND TRUE SAMPLING RATE
	auto now =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>diff= now - previousTimeScan; //in seconds
	timeElapsed=float(diff.count())/1000; //express in seconds
	//printf("time elapsed between newscans = %4f ms\n", timeElapsed);
	totalTime += timeElapsed; //for debugging
	previousTimeScan=now; //update the time of sampling

	//timeElapsed = samplingRate;
	//DISCARD BAD SCANS
	if ( timeElapsed< .15){
		//timeElapsed= samplingRate;
		//return;
	}
	else if (timeElapsed >.25){
		printf("took too long!");
		//return;
	}

	//CREATE BOX2D ENVIRONMENT
	b2World world = b2World({0.0f,0.0f});
	char name[256];

	
	//CALCULATE VELOCITY 
	
	Configurator::getVelocityResult affineTransResult= GetRealVelocity(current, previous);
	b2Vec2 estimatedVelocity;

	if (affineTransResult.valid){
		estimatedVelocity = affineTransResult.vector;
	}
	else{
		estimatedVelocity = state.getAction().getLinearVelocity();
		//printf("invalid affine trans result\n");
	}
	//printf("estimatedVelocity = %f, %f\n", estimatedVelocity.x, estimatedVelocity.y);

	//MAKE NOTE OF WHAT STATE WE'RE IN BEFORE RECHECKING FOR COLLISIONS
	bool wasAvoiding = 0;
	bool isSameState = 1;
	//UPDATE ABSOLUTE POSITION (SLAM-ISH for checking accuracy of velocity measurements)

	absPosition.x += estimatedVelocity.x*timeElapsed;
	absPosition.y += estimatedVelocity.y*timeElapsed;
	//dumpPath = fopen("/tmp/dumpPath.txt", "a+");
	//fprintf(dumpPath, "%f\t%f\n", absPosition.x, absPosition.y);
	//fclose(dumpPath);

	//IF WE  ALREADY ARE IN AN OBSTACLE-AVOIDING STATE, ROUGHLY ESTIMATE WHERE THE OBSTACLE IS NOW
	if (state.obstacle.isValid()){
		wasAvoiding =1; //remember that the robot was avoiding an obstacle
		//printf("old untracked obstacle position: %f\t%f\n", plan[0].obstacle.getPosition().x, plan[0].obstacle.getPosition().y);
		state.trackObject(state.obstacle, timeElapsed, estimatedVelocity, {0.0f, 0.0f}); //robot default position is 0,0
		if (state.obstacle.getAngle(estimatedVelocity) >= M_PI_2){ 		//if obstacle (pos) and robot (vel) are perpendicular
			state.obstacle.invalidate();
			state = desiredState;
			//plan.states.erase(plan.states.begin());						//no need to be in obstacle avoiding state
		//	printf("erased plan\n");
		}
		else{
			State::Object temp = state.obstacle;			//otherwise update current state with new obstacle position
			//plan.states[0]= State(temp);
			state = State(temp);
		}

	}

	//MAKE BOX2D BODIES 

	bool isObstacleStillThere=0;
	for (Point &p:current){
		if (p != *(&p-1)&& p.x >=0 && p.r < state.box2dRange){
			b2Body * body;
			b2BodyDef bodyDef;
			b2FixtureDef fixtureDef;
			bodyDef.type = b2_dynamicBody;
			b2PolygonShape fixture; //giving the point the shape of a box
			fixtureDef.shape = &fixture;
			fixture.SetAsBox(.001f, .001f); 

		//CHECK IF BODIES ARE OBSERVED IN THE GENERAL AREA WHERE THE OBSTACLE SHOULD BE 
			if (state.obstacle.isValid()){
				if (p.isInSquare(state.obstacle.getPosition())){
					isObstacleStillThere =1;
				}
			}
			if (p.x !=0 && p.y!=0){
				bodyDef.position.Set(p.x, p.y); 
				body = world.CreateBody(&bodyDef);
				body->CreateFixture(&fixtureDef);
			}

		}
	}
	//printf("bodies = %i\n", world.GetBodyCount());
	//printf("is obstacle still there = %i\n", isObstacleStillThere);
	// if (!isObstacleStillThere && plan.states.size()>0){ 
	if (!isObstacleStillThere){ 
		//plan.states.erase(plan.states.begin());
		state = desiredState;
	}

	//CREATE ANOTHER PARALLEL PLAN TO COMPARE IT AGAINST THE ONE BEING EXECUTED: currently working on greedy algorithm so local minimum wins
	Plan plan;
	Plan plan2;
	b2Vec2 start(0.0f, 0.0f);
	float theta=0;



	//CHECK IF WITH THE CURRENT STATE THE ROBOT WILL CRASH
	isSameState = wasAvoiding == state.obstacle.isValid();
	State::simResult result;
	state.setRecordedVelocity(estimatedVelocity);
	int advanceIndex=0;
	//if plan contains less than 3 valid obstacles or is within the 1.5m range, plan next action
	
	//while (plan.getStepDuration()< getMaxStepDuration() || plan.getObstacleCount()< getMaxObstacleWM()){ //if there is no obstacles the max duration should be fulfilled with one state
		//printf("plan duration (Steps) = %i, max plan duration = %i", plan.getStepDuration(), getMaxStepDuration());
		//check if the trajectory being followed will bump the robot into something
		result =state.willCollide(world, iteration, start, theta);
		start = state.endPose.p;
		theta = state.endPose.q.GetAngle();
		if (result.resultCode == State::simResult::crashed){
			//printf("crashed\n");
			//IF THERE IS NO PLAN OR THE OBJECT WE CRASHED INTO IS NOT ALREADY BEING AVOIDED ADD NEW STATE TO THE PLAN
			Point p(result.collision.getPosition());
			if (!state.obstacle.isValid() || !(p.isInSquare(state.obstacle.getPosition()))){
				state = State(result.collision);
				plan.states.push_back(State(result.collision)); //ADD TO THE QUEUE OF OBSTACLES TO AVOID
				// printf("pushed back plan of duration = %i steps\n", state.stepDuration);
				// printf("plan size after pushback= %i\n", plan.states.size());
				//printf("created new trajectory omega= %f, linear speed: %f\n", plan[0].getAction().getOmega(), plan[0].getAction().getLinearSpeed() );

			}			
		}
		else {//printf("not crashed\n");
		}
	// 	advanceIndex++;
	// 	printf("plan duration in steps = %f\n", plan.getStepDuration());

	// }
	//printf("plan size = %i\n", plan.states.size());
	//IF THE STATE DIDN'T CHANGE, CORRECT PATH 
	//printf("was avoiding? %i\tis same state? %i\n", wasAvoiding, isSameState);
	applyController(isSameState, state);
	//printf("plan size (end of newscan) = %i, iteration %i\n", plan.size(), iteration);


}




void Configurator::applyController(bool isSameState, State & state){
	if (isSameState){
		//printf("state is the same\n");
		if (state.controller()==State::controlResult::DONE){
			state = desiredState;
			//plan.states.erase(plan.states.begin());	
			//printf("obstacle successfully avoided\n");					//no need to be in obstacle avoiding state
		}
	}


}


	


	

Configurator::getVelocityResult Configurator::GetRealVelocity(std::vector <Point> &_current, std::vector <Point> &_previous){	 //does not modify current vector, creates copy	
		getVelocityResult result;

		int diff = _current.size()-_previous.size(); //if +ve,current is bigger, if -ve, previous is bigger
        //adjust for discrepancies in vector size		//int diff = currSize-prevSize;
		std::vector <cv::Point2f> currentTmp, previousTmp;
		//MAKE OPENCV VECTORS
		for (Point p: _current){
			currentTmp.push_back(cv::Point2f(p.x, p.y));
		}
		for (Point p: _previous){
			previousTmp.push_back(cv::Point2f(p.x, p.y));
		}

		if(diff>0){ //(current.size()>previous.size()){
			if (previousTmp.empty()){
				previousTmp = currentTmp;
				}
			else{
				for (int i=0; i<abs(diff); i++){
					previousTmp.push_back(previousTmp[0]); //before it was [-1]
				if (previousTmp[-1].x == 0 && previousTmp[-1].y ==0){
					printf("can't get previous data\n");
				}

			}
			}
		}
	
		else if (diff<0){//(current.size()<previous.size()){
			if (currentTmp.empty()){
				for (cv::Point2f p:previousTmp){
					//current.push_back(cv::Point2f());
					printf("no data\n");
					return result;
				} 
				//current = previousTmp;
				}
			else{
				for (int i=0; i<abs(diff); i++){
			currentTmp.push_back(currentTmp[0]);
				if (currentTmp[-1].x == 0 && currentTmp[-1].y ==0){
				}

				}
		}
		//printf("p>curr\n");
		}

	//use partial affine transformation to estimate displacement
	cv::Mat transformMatrix = cv::estimateAffinePartial2D(previousTmp, currentTmp, cv::noArray(), cv::LMEDS);
	float theta;
		//printf("matrix is empty? %i\n", transformMatrix.empty());
		if (!transformMatrix.empty()){
			b2Vec2 tmp;
			tmp.x= -(transformMatrix.at<double>(0,2))/timeElapsed;
			tmp.y = -(transformMatrix.at<double>(1,2))/timeElapsed;
			//printf("velocity result %f %f\n", tmp.x, tmp.y);
			float tmpAngle = atan(tmp.y/tmp.x); //atan2 gives results between pi and -pi, atan gives pi/2 to -pi/2
			if (tmp.y ==0 && tmp.x ==0){
				tmpAngle =0;
			}
			float tmpPi = tmpAngle/M_PI;
			//dumpDeltaV = fopen("/tmp/deltaV.txt", "a");
			//fprintf(dumpDeltaV, "og velocity x= %f, y=%f, length = %f\n", tmp.x, tmp.y, tmp.Length()); //technically
			//fprintf(dumpDeltaV,"tmpAngle = %f pi\n", tmpPi);
			if (tmp.Length()>state.getMaxSpeed()){
				affineTransError += tmp.Length()-state.getMaxSpeed();
				tmp.x = state.getAction().getLinearSpeed() *cos(tmpAngle);
				tmp.y = state.getAction().getLinearSpeed() *sin(tmpAngle);
				//printf("getting velocity from proprioception\n");
			//fprintf(dumpDeltaV, "changed velocity to x= %f, y=%f, length = %f\n", tmp.x, tmp.y, tmp.Length()); //technically

			}
			//fclose(dumpDeltaV);
			return getVelocityResult(tmp);
		}
		else if (transformMatrix.empty()){ //if the plan is empty look at the default wheel speed
			b2Vec2 estimatedVel;
			theta = state.getAction().getOmega()* timeElapsed;
			estimatedVel ={state.getAction().getLinearSpeed()*cos(theta),state.getAction().getLinearSpeed()*sin(theta)};
			result = getVelocityResult(estimatedVel);
			//printf("getting velocity from current state\n");
			return result;
		}
		else{
			printf("could not find velocity\n");
			return result;
		}
	}
	//}



