#pragma once
#include "configurator.h"
#include <chrono>

void ConfiguratorInterface::setReady(bool b){
	ready = b;
}


bool ConfiguratorInterface::isReady(){
	return ready;
}

void Configurator::Spawner(CoordinateContainer data, CoordinateContainer data2fp){ 
	//printf("started spawner\n");
	//PREPARE VECTORS TO RECEIVE DATA
	//printf("current size = %i, previous size = 0, currentbox2d size = %i\n", current.size(), currentBox2D.size());
	if (data.empty()){
		printf("data empty!\n");
		return;
	}
	//printf("starting vector update\n");
	CoordinateContainer previous =current;
	//previous = CoordinateContainer(current);
	//printf("previous=current1n");
	current.clear();
	//printf("current clear\n");
	current = CoordinateContainer(data);
	//printf("current=data\n");
	currentBox2D.clear();
	//printf("box2d clear\n");
	currentBox2D = CoordinateContainer(data2fp);
	//printf("updated coordinate vectors\n");
	iteration++; //iteration set in getVelocity
	worldBuilder.iteration++;

	sprintf(bodyFile, "/tmp/bodies%04i.txt", iteration);
	sprintf(worldBuilder.bodyFile, "%s",bodyFile);
	FILE *f;
	if (debugOn){
		f = fopen(bodyFile, "w");
		fclose(f);
		printf("planfile = robot%04i.txt\n", iteration);
	}
	//printf("current = %i, vurrentbox2d = %i", current.size(), currentBox2D.size());
	//BENCHMARK + FIND TRUE SAMPLING RATE
	auto now =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>diff= now - previousTimeScan; //in seconds
	timeElapsed=float(diff.count())/1000; //express in seconds
	totalTime += timeElapsed; //for debugging
	previousTimeScan=now; //update the time of sampling
	//printf("calculated time elapsed = %f\n", timeElapsed);

	if (timerOff){
		timeElapsed = .2;
	}


	//CREATE BOX2D ENVIRONMENT
	b2Vec2 gravity = {0.0, 0.0};
	b2World world= b2World(gravity);
	char name[256];

	//CALCULATE VELOCITY 
	//printf("current = %i\t previous = %i\n", current.size(), previous.size());
	//DeltaPose deltaPose= GetRealVelocity(current, previous); //closed loop, sensor feedback for velocity
	DeltaPose deltaPose = assignDeltaPose(currentTask.getAction(), timeElapsed); //open loop
	if (currentTask.direction ==DEFAULT){
		//currentTask.action.setOmega(deltaPose.q.GetAngle()); //NO SETTING ANGLE
		currentTask.action.setRecSpeed(SignedVectorLength(deltaPose.p));
		currentTask.action.setRecOmega(deltaPose.q.GetAngle());
	}
//	printf("calculated velocity\n");

	//MAKE NOTE OF WHAT STATE WE'RE IN BEFORE RECHECKING FOR COLLISIONS
	bool wasAvoiding = currentTask.disturbance.isValid();
	bool isSameTask = 1;
	//UPDATE ABSOLUTE POSITION (SLAM-ISH for checking accuracy of velocity measurements)

	//IF WE  ALREADY ARE IN AN OBSTACLE-AVOIDING STATE, ROUGHLY ESTIMATE WHERE THE OBSTACLE IS NOW
	//currentTask.trackDisturbance(currentTask.disturbance, timeElapsed, deltaPose); //robot default position is 0,0
	//controlGoal.trackDisturbance(controlGoal.disturbance,timeElapsed, deltaPose);
	//printf("currentTask direction =%i\n", int(currentTask.direction));
	//bool isObstacleStillThere=constructWorldRepresentation(world, currentTask.direction, b2Transform(b2Vec2(0.0, 0.0), b2Rot(0)), &currentTask); 
	bool isObstacleStillThrere = worldBuilder.buildWorld(world, currentBox2D, currentTask.start, currentTask.direction, &currentTask);
	//printf("bodies = %i\n", bodies);
	//printf("obstill there! = %i\n", isObstacleStillThere);
	//EndedResult tempEnded = currentTask.checkEnded();
	if (controlGoal.change){
		currentTask=Task(Disturbance(), STOP);
		return;
	}
	//EndedResult controlEnded = controlGoal.checkEnded();
	// if (controlEnded.ended){
	// 	currentTask= Task(Disturbance(), STOP);
	// 	return;
	// }
	//printf("obstill there = %i\n", isObstacleStillThere);
	// if(tempEnded.ended|| !isObstacleStillThere){
	// 	if (!plan.empty()){
	// 		currentTask = Task(plan[0].first, plan[0].second);
	// 		Sequence s = {TaskSummary(plan[0].first, plan[0].second)};
	// 		printf("switched to ");
	// 		printPlan(s);
	// 		plan.erase(plan.begin());
	// 	}
	// 	else{
	// 		currentTask = Task(controlGoal.disturbance, DEFAULT); //fall back to control goal
	// 		printf("no plan\n");
	// 	}
	//	}

	//CHECK IF WITH THE CURRENT currentTask THE ROBOT WILL CRASH
	isSameTask = wasAvoiding == currentTask.disturbance.isValid();
	simResult result;
	collisionGraph.clear();
	//creating decision tree Disturbance
	vertexDescriptor v0 = boost::add_vertex(collisionGraph);
	std::vector <vertexDescriptor> leaves;
	Direction dir;

	auto startTime =std::chrono::high_resolution_clock::now();
	//printf("set velocity and created empty graph\n");
	//printf("planning =%i\n", planning);
	/////////////REACTIVE AVOIDANCE: substitute the currentTask
	vertexDescriptor bestLeaf = v0;
	//if (!planning){
		//printf("evaluating current task\n");
		// reactiveAvoidance(world, result, currentTask);
		// collisionGraph[v0].fill(result);
		result = evaluateNode(v0,collisionGraph, currentTask, world);
		
		//printf("outcome of v0 = %i, linear speed = %f, omega = %f\n", int(collisionGraph[v0].outcome), currentTask.getAction().getRecSpeed(), currentTask.getAction().getRecOmega());
	//}	
	//printf("planning = %i, collisionGraph[v0],outcome =%i, planbuild.dynamic = %i, plan.empty= %i", planning, int(collisionGraph[v0].outcome), planBuild!=STATIC, plan.empty() );
	if (planning & ( planBuild!=STATIC || plan.empty())){ //og. collisionGraph[v0].outcome !=simResult::successful || 
		switch (graphConstruction){
			case BACKTRACKING:{
				//printf("backtracking build\n");
				backtrackingBuildTree(v0, collisionGraph, currentTask, world, leaves); //for now should produce the same behaviour because the tree is not being pruned. original build_tree returned bool, now currentTask.change is changed directly
				bestLeaf = findBestLeaf(collisionGraph, leaves, v0);
				break;
			}	
			case A_STAR:{
				classicalAStar(v0, collisionGraph, currentTask, world, bestLeaf);
				break;
			}
			case A_STAR_DEMAND:{
				onDemandAStar(v0, collisionGraph, currentTask, world, bestLeaf);
				break;
			}
			// case SIMPLE_TREE:{
			// 	printf("simple\n");
			// 	classicalAStar(v0, collisionGraph, currentTask, world, bestLeaf);
			// 	break;
			// }
			default:
				break;
		}
		plan = getCleanSequence(collisionGraph, bestLeaf);
		printf("plan:");
		printPlan(plan);
	}
	//printf("best leaf ends at %f %f\n",g[bestLeaf].endPose.p.x, g[bestLeaf].endPose.p.y);


	// if (collisionGraph[v0].outcome == simResult::crashed){ //only change task if outcome is crashed
	// 	if (!plan.empty()){
	// 		Sequence next= {plan[0]};
	// 		printf("change to:");
	// 		printPlan(next);
	// 		currentTask = Task(plan[0].first, plan[0].second);
	// 		plan.erase(plan.begin());
	// 	}

	// }
	currentTask.change = collisionGraph[v0].outcome==simResult::crashed;
	//if (currentTask.change){

	//}
	printf("outcome code = %i, change task cause it fails = %i\n", int(collisionGraph[v0].outcome), currentTask.change);
	//printf("action: recLInSpeed = %f, recOmega= %f, direction = %i\n", currentTask.action.getRecSpeed(), currentTask.action.getRecOmega(), int(currentTask.direction));
	//changeTask(currentTask.change, plan, collisionGraph[v0]);
	//printf("tree size = %i, bodies = %i, plan size = %i\n", collisionGraph.m_vertices.size(), bodies, plan.size());
	float duration=0;
	if (benchmark){
	 	auto endTime =std::chrono::high_resolution_clock::now();
	 	std::chrono::duration<float, std::milli>d= startTime- endTime; //in seconds
	 	float duration=abs(float(d.count())/1000); //express in seconds
		FILE * f = fopen(statFile, "a+");
	//	printf("open stat\n");
		fprintf(f,"%i\t%i\t%f\n", bodies, collisionGraph.m_vertices.size(), duration);
		fclose(f);

	}
	bodies =0;
	//CHOOSE BEXT NEXT Task BASED ON LOOKING AHEAD OF THE PRESENT OBSTACLE

	//IF THE TASK DIDN'T CHANGE, CORRECT PATH 
	if (isSameTask){
		currentTask.controller();
		//printf("applied controller: wheel speeds: L=%f, R=%f\n", currentTask.getAction().L, currentTask.getAction().R);
	}

	//graph should be saved and can check, if plan actually executed successfully, the probability to transition to that state increases. Read on belief update

}


// void Configurator::applyController(bool isSameTask, Task & task){
// 	if (isSameTask){
// 		if (task.controller()==Task::controlResult::DONE){
// 			task = controlGoal;
// 		}
// 	}
// }

std::pair <bool, Direction> Configurator::getOppositeDirection(Direction d){
	std::pair <bool, Direction> result(false, DEFAULT);
	if (numberOfM == THREE_M){
		switch (d){
		case Direction::LEFT: result.first = true; result.second = RIGHT;break;
		case Direction::RIGHT: result.first = true; result.second = LEFT;break;
		default:
		break;
	}
	}
	else if (numberOfM == FOUR_M){
		switch (d){
		case Direction::LEFT: result.first = true; result.second = RIGHT;break;
		case Direction::RIGHT: result.first = true; result.second = LEFT;break;
		case Direction::DEFAULT: result.first = true; result.second = BACK;break;
		case Direction::BACK: result.first = true; result.second = DEFAULT;break;
		default:
		break;		
	}
	}
	return result;
}

DeltaPose Configurator::GetRealVelocity(CoordinateContainer &_current, CoordinateContainer &_previous){	 //does not modify current vector, creates copy	
		DeltaPose result;
		float theta;
	 	theta = currentTask.getAction().getOmega()* timeElapsed;
		result.p ={currentTask.getAction().getLinearSpeed()*cos(theta),currentTask.getAction().getLinearSpeed()*sin(theta)};
		result.q.Set(currentTask.getAction().getOmega());

		int diff = _current.size()-_previous.size(); //if +ve,current is bigger, if -ve, previous is bigger
        //adjust for discrepancies in vector size		//int diff = currSize-prevSize;
		std::vector <cv::Point2f> currentTmp, previousTmp;
		//MAKE OPENCV VECTORS
		for (Point p:_current){
			currentTmp.push_back(cv::Point2f(p.x, p.y));
		}
		for (Point p: _previous){
			previousTmp.push_back(cv::Point2f(p.x, p.y));
		}

		if(diff>0){ 
			if (previousTmp.empty()){
				//previousTmp = currentTmp;
				return result;
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
		else if (diff<0){
			if (currentTmp.empty()){
				printf("no data\n");
				//for (cv::Point2f p:previousTmp){
					return result;
				//} 
				}
			else{
				for (int i=0; i<abs(diff); i++){
					currentTmp.push_back(currentTmp[0]);
				}
			}
		}		
	//use partial affine transformation to estimate displacement
	cv::Mat transformMatrix =cv::estimateAffinePartial2D(previousTmp, currentTmp, cv::noArray(), cv::LMEDS);
	if (!transformMatrix.empty()){
		result.p.x= -(transformMatrix.at<double>(0,2))/timeElapsed;
		result.p.y = -(transformMatrix.at<double>(1,2))/timeElapsed;
		result.q.Set(acos(transformMatrix.at<double>(0,0))/timeElapsed);
		float posAngle = atan(result.p.y/result.p.x); //atan2 gives results between pi and -pi, atan gives pi/2 to -pi/2
		if (result.p.y ==0 && result.p.x ==0){
			posAngle =0;
		}
		if (result.p.Length()>MAX_SPEED){
			result.p.x = currentTask.getAction().getLinearSpeed() *cos(posAngle);
			result.p.y = currentTask.getAction().getLinearSpeed() *sin(posAngle);
		}
		
	}
	// else if (transformMatrix.empty()){ //if the plan is empty look at the default wheel speed
	// 	theta = currentTask.getAction().getOmega()* timeElapsed;
	// 	result.p ={currentTask.getAction().getLinearSpeed()*cos(theta),currentTask.getAction().getLinearSpeed()*sin(theta)};
	// 	result.q.Set(currentTask.getAction().getOmega());
	// }
	// else{
	// 	printf("could not find velocity\n");
	// }	
	return result;
	}



void Configurator::reactiveAvoidance(b2World & world, simResult &r, Task &s){ //returns true if disturbance needs to be eliminated	
	r =s.willCollide(world, iteration, debugOn, SIM_DURATION);
	if (r.resultCode == simResult::crashed){
		printf("crashed\n");
		//IF THERE IS NO PLAN OR THE Disturbance WE CRASHED INTO IS NOT ALREADY BEING AVOIDED ADD NEW Task TO THE PLAN
		Point p(r.collision.getPosition());
		if ((!s.disturbance.isValid()|| !(p.isInRadius(s.disturbance.getPosition())))){ 
			s = Task(r.collision, Direction::DEFAULT);
		}			
	}
}


simResult Configurator::evaluateNode(vertexDescriptor v, CollisionGraph&g, Task  s, b2World & w){
	//PREPARE TO LOOK AT BACK EDGES
	edgeDescriptor inEdge;
	vertexDescriptor srcVertex=v; //default
	bool notRoot = boost::in_degree(v, g)>0;
	bool isLeaf=0;
	vertexDescriptor v1 = v; //by default if no vertices need to be added the function returns the startingVertex

		//EVALUATE NODE()
	simResult result; 
	float remaining = simulationStep*2/MAX_SPEED;
	if (s.action.getOmega()!=0){
		remaining =113*2/MAX_SPEED;
	}
	// if (s.discrete){
	// 	remaining = DISCRETE_SIMDURATION;
	// 	range =DISCRETE_RANGE;
	// }
	// else{
	// 	remaining=SIM_DURATION;	
	// 	range = BOX2DRANGE;
	// }
	//IDENTIFY SOURCE NODE, IF ANY
	if (notRoot){
		inEdge = boost::in_edges(v, g).first.dereference();
		srcVertex = boost::source(inEdge, g);
		//find remaining distance to calculate
		if(g[inEdge].direction == Direction::DEFAULT){
		//float remainder = (round(g[srcVertex].endPose.p.Length()*100)%round(simulationStep*100))/100;
			//remaining= (BOX2DRANGE-g[srcVertex].endPose.p.Length())/controlGoal.getAction().getLinearSpeed();
		} 
		if (remaining<0.01){
			remaining=0;
		}
	}
	result =s.willCollide(w, iteration, debugOn, remaining); //default start from 0
	//FILL IN CURRENT NODE WITH ANY COLLISION AND END POSE
	if (result.distanceCovered <=.01){
		g[v].nodesInSameSpot = g[srcVertex].nodesInSameSpot+1; //keep track of how many times the robot is spinning on the spot
	}
	else{
		g[v].nodesInSameSpot =0; //reset if robot is moving
	}
	//SET ORIENTATION OF POINT RELATED TO ITS NEIGHBOURS
//	std::pair <bool, float> orientation = findOrientation(result.collision.getPosition());

	// if (orientation.first){
	// 	// float orientation =s.findOrientation(result.collision.getPosition(), neighbour.second);
	// 	result.collision.setOrientation(orientation.second);
	// }
	result.collision.setOrientation(atan(result.endPose.q.c/result.endPose.q.s)); //90 deg turn
	g[v].fill(result);	
	return result;
	}


void Configurator::backtrackingBuildTree(vertexDescriptor v, CollisionGraph& g, Task s, b2World & w, std::vector <vertexDescriptor> &_leaves){
	//PRINT DEBUG
	//END DEBUG FILE
	vertexDescriptor v1=v;
	Direction dir = s.direction;
	vertexDescriptor v1Src=v;
    do{			
		v= v1;
		//evaluate
		int ct = w.GetBodyCount();
		if (!g[v].filled){
			evaluateNode(v, g,s, w);
		}
		EndedResult er = controlGoal.checkEnded(g[v].endPose);
		// if (!hasStickingPoint(g, v, er)&&  betterThanLeaves(g, v, _leaves, er, dir) ){
		if (betterThanLeaves(g, v, _leaves, er, dir) ){
			applyTransitionMatrix(g,v, s.direction, er.ended, _leaves);
		}
		if (g[v].options.size()==0){
			g[v].error = er.errorFloat;
			_leaves.push_back(v);
			backtrack(g, v);
		}
		if (!addVertex(v, v1, g)){
			return;
		}
		edgeDescriptor v1InEdge = boost::in_edges(v1, g).first.dereference();
		v1Src = v1InEdge.m_source;
		dir = g[v1InEdge].direction;
		s = Task(g[v1Src].disturbance, dir, g[v1Src].endPose);
		worldBuilder.buildWorld(w, currentBox2D, g[v1Src].endPose, dir); //was g[v].endPose
		// if (benchmark){
		// 	printf("bodies in construct= %i\n", w.GetBodyCount());
		// }
	}while (v1!= v);
	//return !g[0].disturbance.safeForNow;
}

// void Configurator::DFIDBuildTree(vertexDescriptor v, CollisionGraph& g, Task s, b2World & w, vertexDescriptor & bestNext){
// 	vertexDescriptor v1 =v;
// 	do{		
// 		v=bestNext;
// 		if (!(g[v].filled)){ //for the first vertex
// 			evaluateNode(v, g, s, w);			
// 		}
// 		EndedResult er = controlGoal.checkEnded(g[v].endPose);
// 		applyTransitionMatrix(g, v, s.direction, er.ended);
// 		for (Direction d: g[v].options){ //add and evaluate all vertices
// 			bool added = addVertex(v, v1, g, g[v].disturbance); //add
// 			s = Task(g[v].disturbance, d, g[v].endPose);
// 			worldBuilder.buildWorld(w, currentBox2D, g[v].endPose, d); //was g[v].endPose
// 			//constructWorldRepresentation(w, d, g[v].endPose); //was g[v].endPose
// 			evaluateNode(v1, g, s, w); //find simulation result
// 			g[v1].error= controlGoal.checkEnded(g[v1].endPose).errorFloat;
// 			if (bestNext==0|| g[bestNext].error >g[v1].error){ //find error
// 				// if (bestNext.vertex!=0){
// 				// 	boost::remove_vertex(bestNext.vertex, g);
// 				// }
// 				bestNext = v1;
// 			}
// 		}
// 	}while(bestNext!=v); //this means that v has progressed
// }

void Configurator::classicalAStar(vertexDescriptor v, CollisionGraph& g, Task s, b2World & w, vertexDescriptor & bestNext){
	vertexDescriptor v1, v0;
	float error;
	bool added;
	Direction direction = s.direction;
	do{	
		std::vector <vertexDescriptor> frontier;	
		v=bestNext;
		if (!(g[v].filled)){ //for the first vertex
			evaluateNode(v, g, s, w);			
		}
		EndedResult er = findError(v, g, direction);
		// if (graphConstruction ==SIMPLE_TREE & g[v].outcome !=simResult::successful){ //calculate error and then reset
		// 	g[v].endPose = s.start;
		// 	g[v].outcome = simResult::crashed;
		// }
		applyTransitionMatrix(g, v, direction, er.ended);
		//printf("options = %i\n", g[v].options.size());
		for (Direction d: g[v].options){ //add and evaluate all vertices
			v0=v;
			v1 =v0;
			do {
			added =addVertex(v0, v1, g, g[v0].disturbance); //add
			edgeDescriptor e = boost::in_edges(v1, g).first.dereference();
			s = Task(g[v0].disturbance, g[e].direction, g[v0].endPose);
			//constructWorldRepresentation(w, g[e].direction, s.start); //was g[v].endPose
			worldBuilder.buildWorld(w, currentBox2D, s.start, g[e].direction); //was g[v].endPose
			evaluateNode(v1, g, s, w); //find simulation result
			applyTransitionMatrix(g, v1, g[e].direction, er.ended);
			v0=v1;
			}while(s.direction !=DEFAULT & added);
			g[v1].error = findError(v1, g, s.direction).errorFloat;
			// if (graphConstruction ==SIMPLE_TREE & g[v1].outcome !=simResult::successful){ //calculate error and then reset
			// 	g[v1].endPose = g[v0].endPose;
			// }
			frontier.push_back(v1);
		}
		bestNext = findBestLeaf(g, frontier, v);
		direction = g[boost::in_edges(bestNext, g).first.dereference()].direction;
	}while(bestNext !=v); //this means that v has progressed
}

void Configurator::onDemandAStar(vertexDescriptor v, CollisionGraph& g, Task s, b2World & w, vertexDescriptor & bestNext){
	discretized=1;
	vertexDescriptor v1=v;
	std::vector <vertexDescriptor> priorityQueue ={v}, evaluationQueue = {v};
	//std::map <vertexDescriptor, std::vector <b2Transform>> steps;
	bool end=0, added =0;
	bool discrete =0;
//	evaluateNode(priorityQueue[0], g, s, w);		
	do {
		v= priorityQueue[0];
		priorityQueue.erase(priorityQueue.begin());
		//DISCOVER AND ADD TWO VERTICES
		for (Direction d: g[v].options){ //add and evaluate all vertices
		vertexDescriptor v0=v;
		v1 =v0;
		do {
		added =addVertex(v0, v1, g, g[v0].disturbance); //add
		edgeDescriptor e = boost::in_edges(v1, g).first.dereference();
		s = Task(g[v0].disturbance, g[e].direction, g[v0].endPose);
		//constructWorldRepresentation(w, g[e].direction, s.start); //was g[v].endPose
		worldBuilder.buildWorld(w, currentBox2D, s.start, g[e].direction); //was g[v].endPose
		evaluateNode(v1, g, s, w); //find simulation result
		applyTransitionMatrix(g, v1, d, controlGoal.checkEnded(g[v1]).ended);
		v0=v1;
		}while(s.direction !=DEFAULT & added); //evaluate the straight nodes
		evaluationQueue.push_back(v1);
		}
		//SPLIT NODE IF NECESSARY
		for (vertexDescriptor ev: evaluationQueue){
			std::vector <vertexDescriptor> split =splitNode(ev, g, s.direction, s.start);
			if (split.size()>1){
				discrete =1;
			}
			//find error and put in queue *********
			for (vertexDescriptor vertex:split){
				EndedResult er = findError(vertex, g, s.direction);
				applyTransitionMatrix(g, vertex,s.direction, er.ended);
				//applyTransitionMatrix(g,vertex, s.direction, er);
				addToPriorityQueue(g, vertex, priorityQueue);
			}			
		}
		evaluationQueue.clear();
	}while(g[v].evaluationFunction()>=g[priorityQueue[0]].evaluationFunction());
	bestNext=v;
}

std::vector <vertexDescriptor> Configurator::splitNode(vertexDescriptor v, CollisionGraph& g, Direction d, b2Transform start){
	std::vector <vertexDescriptor> split = {v};
	if (d ==RIGHT || d==LEFT){
		return split;
	}
	if (g[v].outcome != simResult::safeForNow){
		return split;
	}
	vertexDescriptor v1=v;
	float nNodes = g[v].endPose.p.Length()/DISCRETE_RANGE;
	b2Transform endPose = g[v].endPose;
	int i=0;
	while(nNodes>0){
		g[v].endPose = start;
		g[v].options = {d};
		if(nNodes >1){
			// g[v].endPose.p = start.p+ b2Vec2(DISCRETE_RANGE*endPose.q.c, DISCRETE_RANGE*endPose.q.s);
			// g[v].endPose.q = start.q;
			// start = g[v].endPose;
			start.p =start.p+ b2Vec2(DISCRETE_RANGE*endPose.q.c, DISCRETE_RANGE*endPose.q.s);
			addVertex(v, v1,g); //passing on the disturbance			
			split.push_back(v1);
		}	
		else if (nNodes<1){
			addVertex(v, v1,g); //passing on the disturbance
			g[v1].endPose = endPose;
			split.push_back(v1);
		}
		g[v1].disturbance = g[v].disturbance;
		g[v1].outcome = g[v].outcome;
		v=v1;
		nNodes-=1;
	}
	return split;
}

// void simpleTree(vertexDescriptor v, CollisionGraph& g, Task s, b2World & w, vertexDescriptor& bestNext){ //one step above reactive
// 	vertexDescriptor v1=v;
// 	bool end=0, added =0;
// 	evaluateNode(v, g, s, w);		
// 	do {
		
// 		for (Direction d: g[v].options){ //add and evaluate all vertices
// 		vertexDescriptor v0=v;
// 		v1 =v0;
// 		do {
// 		added =addVertex(v0, v1, g, g[v0].disturbance); //add
// 		edgeDescriptor e = boost::in_edges(v1, g).first.dereference();
// 		s = Task(g[v0].disturbance, g[e].direction, g[v0].endPose);
// 		constructWorldRepresentation(w, g[e].direction, s.start); //was g[v].endPose
// 		evaluateNode(v1, g, s, w); //find simulation result
// 		applyTransitionMatrix(g, v1, d, controlGoal.checkEnded(g[v1]).ended);
// 		v0=v1;
// 		}while(s.direction !=DEFAULT & added); //evaluate the straight nodes
// 		evaluationQueue.push_back(v1);
// 		}
// 		//SPLIT NODE IF NECESSARY
// 		for (vertexDescriptor ev: evaluationQueue){
// 			std::vector <vertexDescriptor> split =splitNode(ev, g, s.direction, s.start);
// 			if (split.size()>1){
// 				discrete =1;
// 			}
// 			//find error and put in queue *********
// 			for (vertexDescriptor vertex:split){
// 				EndedResult er = findError(vertex, g, s.direction);
// 				applyTransitionMatrix(g, vertex,s.direction, er.ended);
// 				//applyTransitionMatrix(g,vertex, s.direction, er);
// 				addToPriorityQueue(g, vertex, priorityQueue);
// 			}			
// 		}
// 		evaluationQueue.clear();
// 	}while(g[v].evaluationFunction()>=g[priorityQueue[0]].evaluationFunction());
// 	bestNext=v;

// }



void Configurator::removeIdleNodes(CollisionGraph&g, vertexDescriptor leaf, vertexDescriptor root){
	if (leaf <root){
		throw std::invalid_argument("wrong order of vertices for iteration\n");
	}
	vertexDescriptor lastValidNode = leaf;
	while (leaf!=root){
		edgeDescriptor e = boost::in_edges(leaf, g).first.dereference(); //get edge
		vertexDescriptor src = boost::source(e,g);
		if (g[leaf].endPose == g[src].endPose){ //if the leaf does not progress the robot			
			if (lastValidNode != leaf){//create edge with the last working node
				boost::remove_edge(leaf, lastValidNode, g); //remove edge
				boost::add_edge(src, lastValidNode, g);//connect the last working node to the source
			}
			else{
				lastValidNode = src;
			}			
			boost::remove_edge(src, leaf, g); //remove edge
			boost::remove_vertex(leaf, g); //remove vertex
		}
		else{
			lastValidNode = leaf; // the leaf is a valid working node
		}
		leaf = src; //go back
	}
	
}


Sequence Configurator::getCleanSequence(CollisionGraph&g, vertexDescriptor leaf, vertexDescriptor root){
	Sequence p;
	if (leaf <root){
		throw std::invalid_argument("wrong order of vertices for iteration\n");
	}
	while (leaf !=root){
		if (boost::in_degree(leaf, g)<1){
			break;
		}
		else{ 
		edgeDescriptor e = boost::in_edges(leaf, g).first.dereference(); //get edge
		vertexDescriptor src = boost::source(e,g);
		if (g[leaf].endPose != g[src].endPose){ //if the node was successful
			TaskSummary ts(g[src].disturbance, g[e].direction);
			p.insert(p.begin(), ts);	
		}
		leaf = src; //go back
		}
	}
	return p;
	
}

Sequence Configurator::getUnprocessedSequence(CollisionGraph&g, vertexDescriptor leaf, vertexDescriptor root){
	Sequence p;
	if (leaf <root){
		throw std::invalid_argument("wrong order of vertices for iteration\n");
	}
	while (leaf !=root){
		if (boost::in_degree(leaf, g)<1){
			break;
		}
		else{ 
		edgeDescriptor e = boost::in_edges(leaf, g).first.dereference(); //get edge
		vertexDescriptor src = boost::source(e,g);
		TaskSummary ts(g[src].disturbance, g[e].direction);
		p.insert(p.begin(), ts);	
		leaf = src; //go back
		}
	}
	return p;
	
}


vertexDescriptor Configurator::findBestLeaf(CollisionGraph &g, std::vector <vertexDescriptor> _leaves, vertexDescriptor v, EndCriteria * refEnd){
	//FIND BEST LEAF
	if (_leaves.empty()){
		return v;
	}
	vertexDescriptor best = _leaves[0];
	if (refEnd==NULL){
		refEnd = &controlGoal.endCriteria;
	}
	for (vertexDescriptor leaf: _leaves){
		//if (refEnd->hasEnd()){
			if (abs(g[leaf].evaluationFunction())<abs(g[best].evaluationFunction())){
				best=leaf;
				g[best].error= g[leaf].error;
				g[best].cost = g[leaf].cost;
			}
		// }
		// else if (g[leaf].endPose.p.Length() > g[best].endPose.p.Length()){
		// 	best = leaf;
		// }
		// else if (g[leaf].endPose.p.Length() == g[best].endPose.p.Length()){
		// 	if (g[leaf].totDs< g[best].totDs){ //the fact that this leaf has fewer predecessors implies fewer collisions
		// 		best = leaf;
		// 	}
		// }
	}
	return best;
}

EndedResult Configurator::findError(Task s, Node &n){
	EndedResult er = controlGoal.checkEnded(n);
	n.error = er.errorFloat;
	n.cost += s.checkEnded(n).errorFloat;
	return er;
}

EndedResult Configurator::findError(vertexDescriptor v,CollisionGraph& g, Direction d){ 
	EndedResult er = controlGoal.checkEnded(g[v]);
	g[v].error = er.errorFloat;
	b2Transform start= b2Transform(b2Vec2(0, 0), b2Rot(0));
	edgeDescriptor e;
	if(boost::in_degree(v,g)>0){
		e = boost::in_edges(v, g). first.dereference();
		start =g[e.m_source].endPose;
		d = g[e].direction;
	}
	Task s(g[v].disturbance, d, start);
	g[v].cost += s.checkEnded(g[v].endPose).errorFloat;
	return er;
}


Sequence Configurator::getPlan(CollisionGraph &g, vertexDescriptor best){
	//std::vector <edgeDescriptor> bestEdges;
	//int size = g[best].predecessors;
	Sequence p;
	edgeDescriptor e;
	while (boost::in_degree(best, g)){
		best = e.m_source;
		TaskSummary ts(g[best].disturbance, g[e].direction);
		p.insert(p.begin(), ts);
		//p[size-1]=ts; //fill the plan from the end backwards
		//size--;
	}
	return p;
}

void Configurator::printPlan(Sequence p){
	for (TaskSummary ts: p){
		switch (ts.second){
			case DEFAULT: printf("DEFAULT");break;
			case LEFT: printf("LEFT"); break;
			case RIGHT: printf("RIGHT"); break;
			case BACK: printf("BACK"); break;
			case STOP: printf("STOP");break;
			default:break;
		}
		printf(", ");
	}
	printf("\n");
}


void Configurator::start(){ 
	if (ci == NULL){
		throw std::invalid_argument("no data interface found");
		return;
	}
	running =1;
	if (t!=NULL){ //already running
		return;
	}
	t= new std::thread(Configurator::run, this);

}

void Configurator::stop(){
	running =0;
	if (t!=NULL){
		t->join();
		delete t;
		t=NULL;
	}
}

void Configurator::registerInterface(ConfiguratorInterface * _ci){
	ci = _ci;
	ci->ts = TaskSummary(controlGoal.disturbance, controlGoal.direction);
}

void Configurator::run(Configurator * c){
	printf("run\n");
	while (c->running){
		if (c->ci == NULL){
			printf("null pointer to interface\n");
			c->running=0;
			return;
		}
		if (c == NULL){
			printf("null pointer to configurator\n");
			c->running=0;
			return;
		}
		if (c->ci->isReady()){
			printf(".");
			//if (c->ci->data2fp != c->currentBox2D & !(c->ci->data.empty())){
				printf("\nc->ci->data2fp size = %i, currentBox2D size = %i\n", c->ci->data2fp.size(), c->currentBox2D.size());
				c->ci->ready=0;
				c->Spawner(c->ci->data, c->ci->data2fp);
				c->ci->ts = TaskSummary(c->currentTask.disturbance, c->currentTask.direction);
		}
	}
}


void Configurator::transitionMatrix(CollisionGraph&g, vertexDescriptor vd, Direction d){
	Task temp(controlGoal.disturbance, DEFAULT, g[vd].endPose);
	switch (numberOfM){
		case (THREE_M):{
				if (g[vd].outcome != simResult::successful){ //accounts for simulation also being safe for now
					if (d ==DEFAULT){
						if (g[vd].nodesInSameSpot<maxNodesOnSpot){
							//in order, try the task which represents the reflex towards the goal
							if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
								g[vd].options.push_back(temp.direction);
								g[vd].options.push_back(getOppositeDirection(temp.direction).second);
							}
							else{
								g[vd].options = {LEFT, RIGHT};
							}			
						}
						}
				}
				else { //will only enter if successful
					if (d== LEFT || d == RIGHT){
						g[vd].options = {DEFAULT};
					}
					else if (d==DEFAULT){
						if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
							g[vd].options.push_back(temp.direction);
							g[vd].options.push_back(getOppositeDirection(temp.direction).second);
							g[vd].options.push_back(DEFAULT);
						}
						else{
							g[vd].options = {DEFAULT, LEFT, RIGHT};
						}

					}

				}
		}	
		break;
		case (FOUR_M):{
			if (g[vd].outcome != simResult::successful){ //accounts for simulation also being safe for now
				if (d ==DEFAULT){
					if (g[vd].nodesInSameSpot<maxNodesOnSpot){
							g[vd].options= {LEFT, RIGHT, BACK};
					}
				}
			}
			else { //will only enter if successful
				if (d== LEFT || d == RIGHT){
					g[vd].options = {DEFAULT};
				}
				else if (d == BACK){
						g[vd].options= {LEFT, RIGHT};
				}
			}
		}
		break;
		default:
		break;
	}
}

bool Configurator::applyTransitionMatrix(CollisionGraph & g, vertexDescriptor vd, Direction d, bool ended, std::vector <vertexDescriptor> leaves){
	bool result =0;
	if (!g[vd].options.empty()){
		return result;
	}
	if (controlGoal.endCriteria.hasEnd()){
		if (ended){
			return result;
		}
	}
	else if(g[vd].totDs>4 || round(g[vd].endPose.p.Length()*100)/100>=BOX2DRANGE){
			return result;
		}
	transitionMatrix(g, vd, d);
	result = !g[vd].options.empty();
	return result;
}



bool Configurator::betterThanLeaves(CollisionGraph &g, vertexDescriptor v, std::vector <vertexDescriptor> _leaves, EndedResult& er, Direction d){
	bool better =1; 
	er = controlGoal.checkEnded(g[v].endPose); //heuristic
	g[v].error = er.errorFloat;
	//expands node if it leads to less error than leaf
	for (vertexDescriptor l: _leaves){
		if (d==DEFAULT){
			if (abs(g[v].evaluationFunction())< abs(g[l].evaluationFunction())){ //if error lower, regardless of distance, keep expanding
				if (!controlGoal.endCriteria.hasEnd()){
					if (g[v].endPose.p.Length() <= g[l].endPose.p.Length() ){//&& (g[v].outcome == g[l.vertex].outcome && g[v].totDs>g[l.vertex].totDs)){
						if (g[v].totDs>=g[l].totDs){
							better=0;
							break;
						}
					}
				}	
			}
			else{
				better =0;
				break; 
			}
		}
	}
	return better;
}

bool Configurator::hasStickingPoint(CollisionGraph& g, vertexDescriptor v, EndedResult & er){
	bool has =0;
	vertexDescriptor src =v;			
	Point dPosition(g[v].disturbance.getPosition());
	//check for repetition along the branch
	while (boost::in_degree(src, g)>0 & g[v].disturbance.isValid() & !g[v].twoStep){
		src =boost::source(boost::in_edges(src, g).first.dereference(), g);
		if(dPosition.isInRadius(g[src].disturbance.getPosition(), 0.03)){ //if the current disturbance is within a 3cm radius from a previous one
			has=1;
			er = controlGoal.checkEnded(g[src].endPose);
			break;
		}
	}
	return has;

}


void Configurator::backtrack(CollisionGraph&g, vertexDescriptor &v){
	while (g[v].options.size()==0){ //keep going back until it finds an incomplete node
		if(boost::in_degree(v, g)>0){
			edgeDescriptor inEdge = boost::in_edges(v, g).first.dereference();
			v = inEdge.m_source;
			if (g[v].options.size()>0){ //if if the vertex exiting the while loop is incomplete add a new node
				//addVertex(v,v1,g);
				return;
			}
		}
		else{
			return;
		}
    }
}

void Configurator::addToPriorityQueue(CollisionGraph& g, vertexDescriptor v, std::vector <vertexDescriptor>& queue){
	for (auto i =queue.begin(); i!=queue.end(); i++){
		if (abs(g[*i].evaluationFunction()) >abs(g[v].evaluationFunction())){
			queue.insert(i, v);
			return;
		}
	}
	queue.push_back(v);
}


std::pair <bool, b2Vec2> Configurator::findNeighbourPoint(b2Vec2 v, float radius){ //more accurate orientation
	std::pair <bool, b2Vec2> result(false, b2Vec2());
	for (Point p: current){
		if (p.isInRadius(v, radius)){
			return result=std::pair<bool, b2Vec2>(true, p.getb2Vec2());
		}
	}
	//auto vIt = current.find(Point(v));
	return result;
}

std::pair <bool, float> Configurator::findOrientation(b2Vec2 v, float radius){
	int count=0;
	float sumY=0, sumX=0;
	float avgY=0, avgX=0;
	std::pair <bool, float>result(false, 0);
	for (Point p:current){
		if (p.isInRadius(v, radius)){
			auto pIt =current.find(p);
			CoordinateContainer::iterator pItNext = pIt++;
			float deltaY =pItNext->y- pIt->y;
			float deltaX = pItNext->x - pIt->x;
			//if (deltaX !=0){
				//float deltaM = deltaY/deltaX;
			//if (abs(deltaM)<=2*abs(avg) && avg!=0){ //prevent outliers
				result.first=true; //is there a neighbouring point?
				//sum+=deltaM;
				count++;
				//avg = sum/count;
				sumY+=deltaY;
				sumX+=deltaX;
				avgY = sumY/count;
				avgX = sumX/count;
			//}
			//}
			
		}
	}
	result.second=atan(avgY/avgX); 
	return result;
}

// void Configurator::makeBody(b2World&w, Point p){
// 	b2Body * body;
// 	b2BodyDef bodyDef;
// 	b2FixtureDef fixtureDef;
// 	bodyDef.type = b2_dynamicBody;
// 	b2PolygonShape fixture; //giving the point the shape of a box
// 	fixtureDef.shape = &fixture;
// 	fixture.SetAsBox(.001f, .001f); 
// 	bodyDef.position.Set(p.x, p.y); 
// 	body = w.CreateBody(&bodyDef);
// 	bodies++;
// 	body->CreateFixture(&fixtureDef);
// }

void Configurator::checkDisturbance(Point p, bool& obStillThere, Task * curr){
	if (NULL!=curr){ //
		if (p.isInRadius(curr->disturbance.getPosition())){
			obStillThere =1;
		}
	}
}

void Configurator::trackTaskExecution(Task & t){
	//if (t.endCriteria.hasEnd()){
		//printf("task in %i has end\n", iteration);
		if (t.step>0){
			t.step--;
			printf("step =%i\n", t.step);
		}
		if(t.step==0){
			t.change=1;
			printf("change task cause it ends = %i\n", t.change);
			//printf("task set to change\n");
		}
	//}
	printf("change =%i, step =%i\n", t.change, t.step);
}

DeltaPose Configurator::assignDeltaPose(Task::Action a, float timeElapsed){
	DeltaPose result;
	float theta = a.getOmega()* timeElapsed;
	result.p ={a.getLinearSpeed()*cos(theta),a.getLinearSpeed()*sin(theta)};
	result.q.Set(a.getOmega());
	return result;
}

int Configurator::motorStep(Task::Action a, EndCriteria ec){
	int result=0, angleResult=0, distanceResult=0;
	if (ec.angle.isValid()){
		angleResult = ec.angle.get()/(MOTOR_CALLBACK * a.getOmega());
	}
	if (ec.distance.isValid()){
		distanceResult = ec.distance.get()/(MOTOR_CALLBACK * a.getLinearSpeed());
	} 
	result =std::max(angleResult, distanceResult);
	printf("task has %i steps\n", result);
	return result;
}

int Configurator::motorStep(Task::Action a){
	int result=0;
        if (a.getOmega()>0){ //LEFT
            //result = SAFE_ANGLE/(MOTOR_CALLBACK * a.getOmega());
		    //result *=FRICTION_DAMPENING;
			result =12;
        }
		else if (a.getOmega()<0){ //RIGHT
			result=15;
		}
		else if (a.getLinearSpeed()>0){
			result = simulationStep/(MOTOR_CALLBACK*a.getLinearSpeed());
		}
        printf("number of steps at creation = %i\n", abs(result));
	    return abs(result);
    }


void Configurator::changeTask(bool b, Sequence & p, Node n, int&ogStep){
	// if (currentTask.step==0){
	// 	b=1;
	// }
	if (!b){
		//printf("do not change\n");
		return;
	}
	if (planning){
		if (plan.empty()){
			//currentTask = controlGoal;
			return;
		}
		currentTask = Task(p[0].first, p[0].second);
		p.erase(p.begin());
		printf("canged to next in plan, new task has %i steps\n", currentTask.step);
	}
	else{
		if (n.disturbance.isValid()){
			currentTask = Task(n.disturbance, DEFAULT); //reactive
		}
		else{
			currentTask = Task(controlGoal.disturbance, DEFAULT); //reactive
		}
		printf("changed to reactive\n");
	}
	currentTask.step = motorStep(currentTask.getAction());
	ogStep = currentTask.step;
	//printf("set step\n");
}

