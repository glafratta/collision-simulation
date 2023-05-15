#pragma once
#include "configurator.h"
#include <chrono>

void Configurator::NewScan(std::vector <Point> & data){ 
	//PREPARE VECTORS TO RECEIVE DATA
	iteration++; //iteration set in getVelocity
	std::vector <Point> previous;
	previous = current;
	current.clear();
	current = data;

	// //ROUND TO 2 decimal PLACES FOR BOX2D
	// for (Point &d: data){
	// 	float x = round(d.x*100)/100;
	// 	float y= round(d.y*100)/100;
	// 	Point p(x,y)
	// 	if (p!=*(&p-1) && p !=*(&p-2)){
	// 		currentBox2D.push_back(p);
	// 	}

	// }


	//BENCHMARK + FIND TRUE SAMPLING RATE
	auto now =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>diff= now - previousTimeScan; //in seconds
	timeElapsed=float(diff.count())/1000; //express in seconds
	totalTime += timeElapsed; //for debugging
	previousTimeScan=now; //update the time of sampling

	// if (debugOn){
	// 	//printf("time elapsed: %f\n", timeElapsed);
	// 	if (iteration>1){
	// 		FILE * f = fopen(statFile, "a+");
	// 		fprintf(f,"%.05f\n", timeElapsed);
	// 		fclose(f);
	// 	}
	// }

	//DISCARD BAD SCANS
	if ( timeElapsed< .19){		
		if (debugOn){
			timeElapsed = .2;
		}
	}
	else if (timeElapsed >.21){
		printf("took too long! %f\n", timeElapsed);
		if (debugOn){
			timeElapsed = .2;
		}
		
	}

	//CREATE BOX2D ENVIRONMENT
	b2World world = b2World({0.0f,0.0f});
	char name[256];
	//int bodies=0;
	
	//CALCULATE VELOCITY 
	
	Configurator::getVelocityResult affRes= GetRealVelocity(current, previous);
	b2Transform deltaP =affRes.vector;

	// if (affineTransResult.valid){
	// 	estimatedVelocity = affineTransResult.vector;
	// }
	// else{
	// 	estimatedVelocity = currentTask.getAction().getLinearVelocity();
	// 	//printf("invalid affine trans result\n");
	//}

	//MAKE NOTE OF WHAT STATE WE'RE IN BEFORE RECHECKING FOR COLLISIONS
	bool wasAvoiding = 0;
	bool isSameTask = 1;
	//UPDATE ABSOLUTE POSITION (SLAM-ISH for checking accuracy of velocity measurements)

	// absPosition.x += estimatedVelocity.x*timeElapsed;
	// absPosition.y += estimatedVelocity.y*timeElapsed;

	//IF WE  ALREADY ARE IN AN OBSTACLE-AVOIDING STATE, ROUGHLY ESTIMATE WHERE THE OBSTACLE IS NOW
	if (currentTask.obstacle.isValid()){
		wasAvoiding =1; //remembesfr that the robot was avoiding an obstacle
		//printf("old untracked obstacle position: %f\t%f\n", plan[0].obstacle.getPosition().x, plan[0].obstacle.getPosition().y);
		currentTask.trackObject(currentTask.obstacle, timeElapsed, deltaP.p, {0.0f, 0.0f}); //robot default position is 0,0
		if (currentTask.obstacle.getAngle(deltaP.p) >= currentTask.endAvoid){ 		//if obstacle (pos) and robot (vel) are perpendicular
			currentTask.obstacle.invalidate();
			currentTask = desiredTask;
		}
		// else{
		// 	// Task::Object temp = currentTask.obstacle;			//otherwise update current state with new obstacle position
		// 	// Task::Direction dir = currentTask.getAction().getDirection();
		// 	// //currentTask = Task(temp, dir);
		// 	// currentTask.obstacle.
		// }

	}

	//MAKE BOX2D BODIES 

	bool isObstacleStillThere=0;
	isObstacleStillThere=constructWorldRepresentation(world, currentTask.getAction().getDirection(), b2Transform(b2Vec2(0.0, 0.0), b2Rot(0)), &currentTask);

	// for (Point &p:current){
	// 	if (p != *(&p-1) && p != *(&p-2)&& p.x >=0 && p.r < BOX2DRANGE){
	// 		b2Body * body;
	// 		b2BodyDef bodyDef;
	// 		b2FixtureDef fixtureDef;
	// 		bodyDef.type = b2_dynamicBody;
	// 		b2PolygonShape fixture; //giving the point the shape of a box
	// 		fixtureDef.shape = &fixture;
	// 		fixture.SetAsBox(.001f, .001f); 

	// 	//CHECK IF BODIES ARE OBSERVED IN THE GENERAL AREA WHERE THE OBSTACLE SHOULD BE 
	// 		if (currentTask.obstacle.isValid()){
	// 			if (p.isInRadius(currentTask.obstacle.getPosition())){
	// 				isObstacleStillThere =1;
	// 			}
	// 		}
	// 		if (p.x !=0 && p.y!=0){
	// 			bodyDef.position.Set(p.x, p.y); 
	// 			body = world.CreateBody(&bodyDef);
	// 			body->CreateFixture(&fixtureDef);
	// 			bodies++;
	// 		}

	//  	}
	//  }
	if (!isObstacleStillThere){ 
		currentTask = desiredTask;
	}
	//int bodyCount = world.GetBodyCount();

	//CREATE ANOTHER PARALLEL PLAN TO COMPARE IT AGAINST THE ONE BEING EXECUTED: currently working on greedy algorithm so local minimum wins
	b2Vec2 start(0.0f, 0.0f);
	float theta=0;



	//CHECK IF WITH THE CURRENT currentTask THE ROBOT WILL CRASH
	isSameTask = wasAvoiding == currentTask.obstacle.isValid();
	Task::simResult result;
	currentTask.setRecordedVelocity(deltaP.p);

	//creating decision tree object
	CollisionGraph g;
	vertexDescriptor v0 = boost::add_vertex(g);
	std::vector <vertexDescriptor> leaves;
	edgeDescriptor e;
	Task::Direction dir;

	auto startTime =std::chrono::high_resolution_clock::now();

	/////////////REACTIVE AVOIDANCE: substitute the currentTask
	switch (planning){
		case 0:
			printf("reacting\n");
			reactiveAvoidance(world, result, currentTask, start, theta);
			break;
		case 1:
			currentTask.change = build_tree(v0, g, currentTask, world, leaves); //for now should produce the same behaviour because the tree is not being pruned. original build_tree returned bool, now currentTask.change is changed directly
			e = findBestBranch(g, leaves);
			if (currentTask.change){
				//see search algorithms for bidirectional graphs (is this like incorrect bonkerballs are mathematicians going to roast me)
				//FIND BEST OPTION FOR CHANGING
				if (g.m_vertices.size()>1){
					dir =g[e].direction;
					printf("bestdirection = %i\n", dir);
					currentTask = Task(g[v0].obstacle, dir); //new currentTask has the obstacle of the previous and the direction of the edge remaining 
				}
				else{ //FALLBACK, ensure it's still operating even if tree building fails
				 	currentTask = Task(g[v0].obstacle, Task::Direction::NONE); //was stop
				 	printf("using fallback\n");
				}
			}
			break;
		default: 
		break;

	}
	printf("tree size = %i\n", g.m_vertices.size());
	if (debugOn){
		auto endTime =std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli>d= startTime- endTime; //in seconds
		float duration=float(d.count())/1000; //express in seconds
		FILE * f = fopen(statFile, "a+");
		fprintf(f,"%i\t%i\t%f\n", bodies, g.m_vertices.size(), duration);
		fclose(f);
	}
	bodies =0;
	currentBox2D.clear();

	//CHOOSE BEXT NEXT currentTask BASED ON LOOKING AHEAD OF THE PRESENT OBSTACLE

	printf("new currentTask wheel speeds: L= %f, R=%f\n", currentTask.getAction().LeftWheelSpeed, currentTask.getAction().RightWheelSpeed);

	//IF THE currentTask DIDN'T CHANGE, CORRECT PATH 
	applyController(isSameTask, currentTask);


}


void Configurator::applyController(bool isSameTask, Task & task){
	if (isSameTask){
		if (task.controller()==Task::controlResult::DONE){
			task = desiredTask;
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
				printf("no data\n");
				for (cv::Point2f p:previousTmp){
					return result;
				} 
				}
			else{
				for (int i=0; i<abs(diff); i++){
			currentTmp.push_back(currentTmp[0]);

				}
		}
		}
//	printf("current Tmp = %i previoustmp %i\n", currentTmp.size(), previousTmp.size());
	//use partial affine transformation to estimate displacement
	cv::Mat transformMatrix =cv::estimateAffinePartial2D(previousTmp, currentTmp, cv::noArray(), cv::LMEDS);
	float theta;
		if (!transformMatrix.empty()){
			result.valid =1;
			result.affineResult.p.x= -(transformMatrix.at<double>(0,2))/timeElapsed;
			result.affineResult.p.y = -(transformMatrix.at<double>(1,2))/timeElapsed;
			result.affineResult.q.Set(acos(transformMatrix.at<double>(0,0))/timeElapsed);
			result.vector = result.affineResult;
			float posAngle = atan(result.affineResult.p.y/result.affineResult.p.x); //atan2 gives results between pi and -pi, atan gives pi/2 to -pi/2
			if (result.affineResult.p.y ==0 && result.affineResult.p.x ==0){
				posAngle =0;
			}
			if (result.affineResult.p.Length()>currentTask.getMaxSpeed()){
				result.vector.p.x = currentTask.getAction().getLinearSpeed() *cos(posAngle);
				result.vector.p.y = currentTask.getAction().getLinearSpeed() *sin(posAngle);
			}
			
		}
		else if (transformMatrix.empty()){ //if the plan is empty look at the default wheel speed
			b2Transform deltaP;
			theta = currentTask.getAction().getOmega()* timeElapsed;
			deltaP.p ={currentTask.getAction().getLinearSpeed()*cos(theta),currentTask.getAction().getLinearSpeed()*sin(theta)};
			deltaP.q.Set(currentTask.getAction().getOmega());
			result = getVelocityResult(deltaP);
		}
		else{
			printf("could not find velocity\n");
		}
		return result;
	}



// Configurator::getVelocityResult Configurator::GetVelocityFromReference(std::vector <Point> &_current, std::vector <Point> &_previous){	 //does not modify current vector, creates copy	
// 		getVelocityResult result;

//         //adjust for discrepancies in vector size		//int diff = currSize-prevSize;
// 		std::vector <cv::Point2f> currentTmp, previousTmp;
// 		//MAKE OPENCV VECTORS
// 		for (Point p: _current){
// 			if(p.y<=0.1 && p.y >=-0.1 && p.x >=0){
// 				currentTmp.push_back(cv::Point2f(p.x, p.y));
// 			}
// 		}
// 		for (Point p: _previous){
// //			previousTmp.push_back(cv::Point2f(p.x, p.y));
// 			if(p.y<=0.1 && p.y >=-0.1 && p.x >=0){
// 				previousTmp.push_back(cv::Point2f(p.x, p.y));
// 			}
// 		}
// 		int diff = currentTmp.size()-previousTmp.size(); //if +ve,current is bigger, if -ve, previous is bigger


// 		if(diff>0){ //(current.size()>previous.size()){
// 			if (previousTmp.empty()){
// 				previousTmp = currentTmp;
// 				}
// 			else{
// 				for (int i=0; i<abs(diff); i++){
// 					previousTmp.push_back(previousTmp[0]); //before it was [-1]
// 				if (previousTmp[-1].x == 0 && previousTmp[-1].y ==0){
// 					printf("can't get previous data\n");
// 				}

// 			}
// 			}
// 		}
	
// 		else if (diff<0){//(current.size()<previous.size()){
// 			if (currentTmp.empty()){
// 				printf("no data\n");
// 				for (cv::Point2f p:previousTmp){
// 					return result;
// 				} 
// 				}
// 			else{
// 				for (int i=0; i<abs(diff); i++){
// 			currentTmp.push_back(currentTmp[0]);
// 				if (currentTmp[-1].x == 0 && currentTmp[-1].y ==0){
// 				}

// 				}
// 		}
// 		}

// 	//use partial affine transformation to estimate displacement
// 	cv::Mat transformMatrix = cv::estimateAffinePartial2D(previousTmp, currentTmp, cv::noArray(), cv::LMEDS);
// 	float theta;
// 		if (!transformMatrix.empty()){
// 			result.affineResult;
// 			result.affineResult.x= -(transformMatrix.at<double>(0,2))/timeElapsed;
// 			result.affineResult.y = -(transformMatrix.at<double>(1,2))/timeElapsed;
// 			float tmpAngle = atan(result.affineResult.y/result.affineResult.x); //atan2 gives results between pi and -pi, atan gives pi/2 to -pi/2
// 			if (result.affineResult.y ==0 && result.affineResult.x ==0){
// 				tmpAngle =0;
// 			}
// 			if (result.affineResult.Length()>currentTask.getMaxSpeed()){
// 				affineTransError += result.affineResult.Length()-currentTask.getMaxSpeed();
// 				result.vector.x = currentTask.getAction().getLinearSpeed() *cos(tmpAngle);
// 				result.vector.y = currentTask.getAction().getLinearSpeed() *sin(tmpAngle);
// 			}
// 			//return getVelocityResult(tmp);
// 		}
// 		else if (transformMatrix.empty()){ //if the plan is empty look at the default wheel speed
// 			b2Vec2 estimatedVel;
// 			theta = currentTask.getAction().getOmega()* timeElapsed;
// 			estimatedVel ={currentTask.getAction().getLinearSpeed()*cos(theta),currentTask.getAction().getLinearSpeed()*sin(theta)};
// 			result = getVelocityResult(estimatedVel);
// 			//return result;
// 		}
// 		else{
// 			printf("could not find velocity\n");
// 			//return result;
// 		}
// 		return result;
// 	}





void Configurator::reactiveAvoidance(b2World & world, Task::simResult &r, Task &s, b2Vec2 & start, float & angle){ //returns true if disturbance needs to be eliminated	
	r =s.willCollide(world, iteration, debugOn, start, angle, s.getSimDuration());
	if (r.resultCode == Task::simResult::crashed){
		printf("crashed\n");
		//IF THERE IS NO PLAN OR THE OBJECT WE CRASHED INTO IS NOT ALREADY BEING AVOIDED ADD NEW Task TO THE PLAN
		Point p(r.collision.getPosition());
		if ((!s.obstacle.isValid()|| !(p.isInRadius(s.obstacle.getPosition())))){ 
			s = Task(r.collision, Task::Direction::NONE);
		}			
	}
}


vertexDescriptor Configurator::eliminateDisturbance(vertexDescriptor v, CollisionGraph&g, Task  s, b2World & w, std::vector <vertexDescriptor> &_leaves){
	//PREPARE TO LOOK AT BACK EDGES
	edgeDescriptor inEdge;
	vertexDescriptor srcVertex=v; //default
	bool notRoot = boost::in_degree(v, g)>0;
	bool isLeaf=0;
	vertexDescriptor v1 = v; //by default if no vertices need to be added the function returns the startingVertex

		//FIND IF THE PRESENT STATE WILL COLLIDE
	Task::simResult result; 
	float remaining=s.getSimDuration();
	//IDENTIFY SOURCE NODE, IF ANY
	if (notRoot){
		inEdge = boost::in_edges(v, g).first.dereference();
		srcVertex = boost::source(inEdge, g);
		//find remaining distance to calculate
		if(g[inEdge].direction == Task::NONE){
			remaining= (BOX2DRANGE-g[srcVertex].distanceSoFar)*2/s.getMaxSpeed();
		} 
		if (remaining<0){
			remaining=0;
		}
		result =s.willCollide(w, iteration, debugOn, g[srcVertex].endPose.p, g[srcVertex].endPose.q.GetAngle(), remaining); //start from where the last Task ended (if previous Task crashes)
		g[inEdge].distanceCovered= result.distanceCovered; //assign data to edge
		g[v].predecessors = g[srcVertex].predecessors +1;
	}
	else{
		result =s.willCollide(w, iteration, debugOn, {0.0, 0.0}, 0, remaining); //default start from 0
	}

	//FILL IN CURRENT NODE WITH ANY COLLISION AND END POSE
	if (result.distanceCovered <=.01){
		g[v].nodesInSameSpot = g[srcVertex].nodesInSameSpot+1; //keep track of how many times the robot is spinning on the spot
	}
	else{
		g[v].nodesInSameSpot =0; //reset if robot is moving
	}
	g[v].obstacle = result.collision;
	if (result.collision.isValid()){
		g[v].totObstacles++;
	}
	g[v].endPose = result.endPose;
	g[v].distanceSoFar = g[srcVertex].distanceSoFar + (round(result.distanceCovered*100))/100; //rounding to 2 decimals to eliminate floating point errors
	g[v].outcome = result.resultCode;
	//IS THIS NODE LEAF? to be a leaf 1) either the maximum distance has been covered or 2) avoiding an obstacle causes the robot to crash
	bool fl = g[v].distanceSoFar >= BOX2DRANGE; //full length
	bool fullMemory = g[v].totObstacles >=4;
	bool moreCostlyThanLeaf =0; 

	//ABANDON EARLY IF CURRENT PATH IS MORE COSTLY THAN THE LAST LEAF: if this vertex is the result of more branching while traversing a smaller distance than other leaves, it is more costly
	for (auto l: _leaves){
		if (g[v].distanceSoFar <= g[l].distanceSoFar && (g[v].outcome == g[l].outcome && g[v].predecessors>g[l].predecessors)){
			moreCostlyThanLeaf =1;
		}

	}
	
	//ADD OPTIONS FOR CURRENT ACTIONS BASED ON THE OUTCOME OF THE Task/TASK/MOTORPLAN ETC i haven't decided a name yet
	if(!fl&& !moreCostlyThanLeaf && !fullMemory){//} && ((v==srcVertex) || (g[srcVertex].endPose !=g[v].endPose))){
		if (result.resultCode != Task::simResult::successful){ //accounts for simulation also being safe for now
			if (s.getType()==Task::Type::BASELINE){
				Task::Direction dir = Task::Direction::NONE;
				if (boost::in_degree(srcVertex, g)>0){ //was >
					dir = g[boost::in_edges(srcVertex, g).first.dereference()].direction;
				}
				if (result.resultCode == Task::simResult::crashed && dir != Task::NONE && g[v].nodesInSameSpot<maxNodesOnSpot){
						g[v].options.push_back(dir);
						//g[v].options.push_back(Task::Direction::BACK); //NEWLY ADDED

					}
				else if (result.resultCode == Task::simResult::safeForNow || boost::in_degree(srcVertex, g)==0){
					Task::Action reflex;
					reflex.__init__(result.collision, Task::Direction::NONE);
					dir= reflex.getDirection();
					g[v].options.push_back(dir);// the first branch is the actions generating from a reflex to the collision
					g[v].options.push_back(getOppositeDirection(dir));
				}
				}
			// else if (s.getAction().getDirection()==Task::Direction::BACK){//NEWLY ADDED
				
//			}

			}
		else { //will only enter if successful
			if (s.getType()==Task::Type::AVOID){
				g[v].options.push_back(Task::Direction::NONE);
			}
	}	
			//}
	}


	isLeaf = (g[v].options.size() ==0);

	//IF THE Task COLLIDES CREATE A PLAN, DEPTH-FIRST
			//DEFINE POSSIBLE NEXT TaskS DEPENDING ON OUTCOME, only if it's not a leaf
	bool straightFollowStraight=0;
	if (!isLeaf){
		addVertex(v, v1, g); //ADD AN EMPTY VERTEX. only info entered for the next vertex is the direction 
		return v1; //added now	
	}
	//IF NO VERTICES CAN BE ADDED TO THE CURRENT BRANCH, CHECK THE CLOSEST BRANCH
	else {
		_leaves.push_back(v);
                while (g[v].options.size()==0){ //keep going back until it finds an incomplete node
                    if(boost::in_degree(v, g)>0){
	                    inEdge = boost::in_edges(v, g).first.dereference();
						v = inEdge.m_source;
						if (g[v].options.size()>0){ //if if the vertex exiting the while loop is incomplete add a new node
							addVertex(v,v1,g);
							return v1;
						}
                    }
                    else{
                        break;
                    }
                }

		}
		return v1;
	}


bool Configurator::build_tree(vertexDescriptor v, CollisionGraph& g, Task s, b2World & w, std::vector <vertexDescriptor> &_leaves){
	char n[250];
	sprintf(n, "/tmp/bodies%04i.txt", iteration);
	if (debugOn){
			FILE *f = fopen(n, "w"); //erase contents from previous run
			fclose(f);
	}
	if (debugOn){
		FILE *f = fopen(n, "a+");
		for (b2Body * b = w.GetBodyList(); b!=NULL; b= b->GetNext()){
			fprintf(f, "%f\t%f\n", b->GetPosition().x, b->GetPosition().y);
		}
		fclose(f);
	}
	vertexDescriptor v1 = eliminateDisturbance(v, g,s,w, _leaves); 
		//destroying world causes segfault even if it's no longer required so skipping for now
    while (v1!= v){
		//printf("reset world\n");
		b2World newWorld({0.0f, 0.0f});
		edgeDescriptor v1InEdge = boost::in_edges(v1, g).first.dereference();

		vertexDescriptor v1Src = v1InEdge.m_source;
		Task::Direction d = g[v1InEdge].direction;
		s = Task(g[v1Src].obstacle, d);
		constructWorldRepresentation(newWorld, d, g[v1Src].endPose); //was g[v].endPose
		int bodyCount = newWorld.GetBodyCount();
		//DEBUG
		if (debugOn){
			FILE *f = fopen(n, "a+");
			for (b2Body * b = newWorld.GetBodyList(); b!=NULL; b= b->GetNext()){
				fprintf(f, "%f\t%f\n", b->GetPosition().x, b->GetPosition().y);
			}
			fclose(f);
		}
		//END DEBUG
		v= v1;
		v1 = eliminateDisturbance(v,g,s, newWorld, _leaves);
	}
	return !g[0].obstacle.safeForNow;

}