#pragma once
#include "configurator.h"
#include <chrono>

void ConfiguratorInterface::setReady(bool b){
	ready = b;
}


bool ConfiguratorInterface::isReady(){
	return ready;
}

void Configurator::Spawner(CoordinateContainer & data, CoordinateContainer & data2fp){ 
	printf("started spawner\n");
	//PREPARE VECTORS TO RECEIVE DATA
	iteration++; //iteration set in getVelocity
	CoordinateContainer previous;
	printf("current size = %i, previous size = %i, currentbox2d size = %i\n", current.size(), previous.size(), currentBox2D.size());
	previous =current;
	current.clear();
	current = data;
	currentBox2D.clear();
	currentBox2D = data2fp;
	printf("updated coordinate vectors\n");

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
	b2World world = b2World({0.0f,0.0f});
	char name[256];

	//CALCULATE VELOCITY 
	//printf("current = %i\t previous = %i\n", current.size(), previous.size());
	DeltaPose deltaPose= GetRealVelocity(current, previous);
	if (currentTask.direction ==DEFAULT){
		//currentTask.action.setOmega(deltaPose.q.GetAngle()); //NO SETTING ANGLE
		currentTask.action.setRecSpeed(SignedVectorLength(deltaPose.p));
		currentTask.action.setRecOmega(deltaPose.q.GetAngle());
	}
	//printf("calculated velocity\n");

	//MAKE NOTE OF WHAT STATE WE'RE IN BEFORE RECHECKING FOR COLLISIONS
	bool wasAvoiding = currentTask.disturbance.isValid();
	bool isSameTask = 1;
	//UPDATE ABSOLUTE POSITION (SLAM-ISH for checking accuracy of velocity measurements)

	//IF WE  ALREADY ARE IN AN OBSTACLE-AVOIDING STATE, ROUGHLY ESTIMATE WHERE THE OBSTACLE IS NOW
	currentTask.trackDisturbance(currentTask.disturbance, timeElapsed, deltaPose); //robot default position is 0,0
	controlGoal.trackDisturbance(controlGoal.disturbance,timeElapsed, deltaPose);
	bool isObstacleStillThere=constructWorldRepresentation(world, currentTask.direction, b2Transform(b2Vec2(0.0, 0.0), b2Rot(0)), &currentTask); 
	EndedResult tempEnded = currentTask.checkEnded();
	EndedResult controlEnded = controlGoal.checkEnded();
	if (controlEnded.ended){
		currentTask= Task(Disturbance(), STOP);
		return;
	}
	if(tempEnded.ended|| !isObstacleStillThere){
		if (!plan.empty()){
			currentTask = Task(plan[0].first, plan[0].second);
			Sequence s = {TaskSummary(plan[0].first, plan[0].second)};
			printf("switched to ");
			printPlan(s);
			plan.erase(plan.begin());
		}
		else{
			currentTask = Task(controlGoal.disturbance, DEFAULT); //fall back to control goal
			printf("no plan\n");
		}
	}

	//CHECK IF WITH THE CURRENT currentTask THE ROBOT WILL CRASH
	isSameTask = wasAvoiding == currentTask.disturbance.isValid();
	simResult result;

	//creating decision tree Disturbance
	CollisionGraph g;
	vertexDescriptor v0 = boost::add_vertex(g);
	std::vector <Leaf> leaves;
	Direction dir;

	auto startTime =std::chrono::high_resolution_clock::now();
	//printf("set velocity and created empty graph\n");

	/////////////REACTIVE AVOIDANCE: substitute the currentTask
		if (!planning){
			printf("reacting\n");
			reactiveAvoidance(world, result, currentTask);
		}	
		else{
			switch (graphConstruction){
				case BACKTRACKING:{
					backtrackingBuildTree(v0, g, currentTask, world, leaves); //for now should produce the same behaviour because the tree is not being pruned. original build_tree returned bool, now currentTask.change is changed directly
					vertexDescriptor bestLeaf = findBestLeaf(g, leaves).vertex;
					plan = getCleanSequence(g, bestLeaf);
					printf("best leaf ends at %f %f\n",g[bestLeaf].endPose.p.x, g[bestLeaf].endPose.p.y);
					printf("plan:");
					printPlan(plan);
					printf("built tree\n");
					break;
				}
					
				case DEPTH_FIRST_ITDE:{
					vertexDescriptor bestLeaf=v0;
					DFIDBuildTree(v0, g, currentTask, world, bestLeaf);
					//vertexDescriptor bestLeaf = *(boost::vertices(g).second); //last is the best because the others are eliminate as we go
					plan = getCleanSequence(g, bestLeaf);
					printf("best leaf ends at %f %f\n",g[bestLeaf].endPose.p.x, g[bestLeaf].endPose.p.y);
					printf("plan:");
					printPlan(plan);
					printf("built tree\n");
					break;
				}
				case DEPTH_FIRST_ITDE_2:{
					vertexDescriptor bestLeaf=v0;
					DFIDBuildTree_2(v0, g, currentTask, world, bestLeaf);
					plan = getCleanSequence(g, bestLeaf);
					printf("best leaf ends at %f %f\n",g[bestLeaf].endPose.p.x, g[bestLeaf].endPose.p.y);
					printf("plan:");
					printPlan(plan);
					printf("built tree\n");
					break;
				}
				default:
					break;
			}
			if (g[v0].outcome == simResult::crashed){ //only change task if outcome is crashed
				if (!plan.empty()){
					Sequence next= {plan[0]};
					printf("change to:");
					printPlan(next);
					currentTask = Task(plan[0].first, plan[0].second);
					plan.erase(plan.begin());
				}
			}
		}
	printf("tree size = %i\n", g.m_vertices.size());
	//float duration=0;
	// if (debugOn){
	 	auto endTime =std::chrono::high_resolution_clock::now();
	 	std::chrono::duration<float, std::milli>d= startTime- endTime; //in seconds
	 	float duration=abs(float(d.count())/1000); //express in seconds
	// 	FILE * f = fopen(statFile, "a+");
	// 	fprintf(f,"%i\t%i\t%f\n", bodies, g.m_vertices.size(), duration);
	// 	fclose(f);
	//}
	bodies =0;
	//CHOOSE BEXT NEXT Task BASED ON LOOKING AHEAD OF THE PRESENT OBSTACLE

	//IF THE TASK DIDN'T CHANGE, CORRECT PATH 
	if (isSameTask){
		currentTask.controller();
	}
	printf("applied controller\n");

	//graph should be saved and can check, if plan actually executed successfully, the probability to transition to that state increases. Read on belief update

}


// void Configurator::applyController(bool isSameTask, Task & task){
// 	if (isSameTask){
// 		if (task.controller()==Task::controlResult::DONE){
// 			task = controlGoal;
// 		}
// 	}
// }

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
		for (Point p: _current){
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


vertexDescriptor Configurator::evaluateNode(vertexDescriptor v, CollisionGraph&g, Task  s, b2World & w){
	//PREPARE TO LOOK AT BACK EDGES
	edgeDescriptor inEdge;
	vertexDescriptor srcVertex=v; //default
	bool notRoot = boost::in_degree(v, g)>0;
	bool isLeaf=0;
	vertexDescriptor v1 = v; //by default if no vertices need to be added the function returns the startingVertex

		//EVALUATE NODE()
	simResult result; 
	float remaining=SIM_DURATION;
	//IDENTIFY SOURCE NODE, IF ANY
	if (notRoot){
		inEdge = boost::in_edges(v, g).first.dereference();
		srcVertex = boost::source(inEdge, g);
		//find remaining distance to calculate
		if(g[inEdge].direction == Direction::DEFAULT){
			remaining= (BOX2DRANGE-SignedVectorLength(g[srcVertex].endPose.p))*2/MAX_SPEED;
		} 
		if (remaining<0){
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
	//std::pair<bool, b2Vec2> neighbour = findNeighbourPoint(result.collision.getPosition());
	std::pair <bool, float> orientation = findOrientation(result.collision.getPosition());
	if (orientation.first){
		// float orientation =s.findOrientation(result.collision.getPosition(), neighbour.second);
		result.collision.setOrientation(orientation.second);
	}
	g[v].fill(result);	
	}


void Configurator::backtrackingBuildTree(vertexDescriptor v, CollisionGraph& g, Task s, b2World & w, std::vector <Leaf> &_leaves){
	char n[250];
	//int bodyCount=0;
	sprintf(n, "/tmp/bodies%04i.txt", iteration);
	//PRINT DEBUG
	if (debugOn){		
		FILE *f = fopen(n, "w"); //erase contents from previous run
		fclose(f);
		FILE *f = fopen(n, "a+");
		for (b2Body * b = w.GetBodyList(); b!=NULL; b= b->GetNext()){
			fprintf(f, "%f\t%f\n", b->GetPosition().x, b->GetPosition().y);
		}
		fclose(f);		
		printf("planfile = robot%04i.txt\n", iteration);

	}
	//END DEBUG FILE
	vertexDescriptor v1=v;
    do{		
		v= v1;
		//evaluate
		int ct = w.GetBodyCount();
		evaluateNode(v, g,s, w);
		EndedResult er = controlGoal.checkEnded(g[v].endPose);
		if (betterThanLeaves(g, v, _leaves, er)){
			applyTransitionMatrix(g,v, s.direction, _leaves);
		}
		if (g[v].options.size()==0){
			_leaves.push_back(Leaf(v, er.errorFloat));
			backtrack(g, v);
		}
		if (!addVertex(v, v1, g)){
			return;
		}
		edgeDescriptor v1InEdge = boost::in_edges(v1, g).first.dereference();
		vertexDescriptor v1Src = v1InEdge.m_source;
		Direction dir = g[v1InEdge].direction;
		s = Task(g[v1Src].disturbance, dir, g[v1Src].endPose);
		constructWorldRepresentation(w, dir, g[v1Src].endPose); //was g[v].endPose
		//DEBUG
		// if (debugOn){
		// 	FILE *f = fopen(n, "a+");
		// 	for (b2Body * b = newWorld.GetBodyList(); b!=NULL; b= b->GetNext()){
		// 		fprintf(f, "%f\t%f\n", b->GetPosition().x, b->GetPosition().y);
		// 	}
		// 	fclose(f);
		// }
		//END DEBUG
	}while (v1!= v);
	//return !g[0].disturbance.safeForNow;
}

void Configurator::DFIDBuildTree(vertexDescriptor v, CollisionGraph& g, Task s, b2World & w, vertexDescriptor & best){
	vertexDescriptor v1 =v;
	Leaf bestNext;
	if (debugOn){
		printf("planfile = robot%04i.txt\n", iteration);
	}
	do{		
		v=bestNext.vertex;
		if (!(g[v].filled)){ //for the first vertex
			evaluateNode(v, g, s, w);			
		}
		EndedResult er = controlGoal.checkEnded(g[v].endPose);
		applyTransitionMatrix(g, v, s.direction);
		for (Direction d: g[v].options){ //add and evaluate all vertices
			bool added = addVertex(v, v1, g, g[v].disturbance); //add
			s = Task(g[v].disturbance, d, g[v].endPose);
			constructWorldRepresentation(w, d, g[v].endPose); //was g[v].endPose
			evaluateNode(v1, g, s, w); //find simulation result
			float error = controlGoal.checkEnded(g[v1].endPose).errorFloat;
			if (!bestNext.valid|| bestNext.error >error){ //find error
				// if (bestNext.vertex!=0){
				// 	boost::remove_vertex(bestNext.vertex, g);
				// }
				bestNext = Leaf(v1, error);
				best=v1;
			}
		}
	}while(bestNext.vertex!=v); //this means that v has progressed
}

void Configurator::DFIDBuildTree_2(vertexDescriptor v, CollisionGraph& g, Task s, b2World & w, vertexDescriptor & best){
	vertexDescriptor v1;
	Leaf bestNext;
	float error;
	bool added;
	if (debugOn){
		printf("planfile = robot%04i.txt\n", iteration);
	}
	do{	
		std::vector <Leaf> frontier;	
		v=bestNext.vertex;
		if (!(g[v].filled)){ //for the first vertex
			evaluateNode(v, g, s, w);			
		}
		EndedResult er = controlGoal.checkEnded(g[v].endPose);
		applyTransitionMatrix(g, v, s.direction);
		for (Direction d: g[v].options){ //add and evaluate all vertices
			vertexDescriptor v0=v;
			v1 =v0;
			do {
			added =addVertex(v0, v1, g, g[v0].disturbance); //add
			edgeDescriptor e = boost::in_edges(v1, g).first.dereference();
			s = Task(g[v0].disturbance, g[e].direction, g[v0].endPose);
			constructWorldRepresentation(w, g[e].direction, s.start); //was g[v].endPose
			evaluateNode(v1, g, s, w); //find simulation result
			applyTransitionMatrix(g, v1, d);
			v0=v1;
			}while(s.direction !=DEFAULT & added);
			error = controlGoal.checkEnded(g[v1]).errorFloat;
			frontier.push_back(Leaf(v1,error));
		}
		bestNext = findBestLeaf(g, frontier);
		best = bestNext.vertex;
	}while(g[best].options.size()>0); //this means that v has progressed
}

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


Leaf Configurator::findBestLeaf(CollisionGraph &g, std::vector <Leaf> _leaves, EndCriteria * refEnd){
	//FIND BEST LEAF
	Leaf best = _leaves[0];
	if (refEnd==NULL){
		refEnd = &controlGoal.endCriteria;
	}
	for (Leaf leaf: _leaves){
		if (refEnd->hasEnd()){
			if (leaf.error<best.error){
				best.vertex=leaf.vertex;
				best.error= leaf.error;
			}
		}
		else if (g[leaf.vertex].endPose.p.Length() > g[best.vertex].endPose.p.Length()){
			best = leaf;
		}
		else if (g[leaf.vertex].endPose.p.Length() == g[best.vertex].endPose.p.Length()){
			if (g[leaf.vertex].totDs< g[best.vertex].totDs){ //the fact that this leaf has fewer predecessors implies fewer collisions
				best = leaf;
			}
		}
	}
	return best;
}

Sequence Configurator::getPlan(CollisionGraph &g, vertexDescriptor best){
	//std::vector <edgeDescriptor> bestEdges;
	//int size = g[best].predecessors;
	Sequence p;
	edgeDescriptor e;
	while (boost::in_degree(best, g)){
		e = boost::in_edges(best, g).first.dereference();
		//bestEdges.push_back(e);
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
}

void Configurator::run(Configurator * c){
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
			//printf(".");
			if (c->ci->data2fp != c->currentBox2D && !(c->ci->data.empty())){
				printf("\nc->ci->data2fp size = %i, currentBox2D size = %i\n", c->ci->data2fp.size(), c->currentBox2D.size());
				c->Spawner(c->ci->data, c->ci->data2fp);
			}
			// else if (c->ci->data.empty()){
			// 	c->currentTask = c->controlGoal;
			// }

		}
	}
}


void Configurator::transitionMatrix(CollisionGraph&g, vertexDescriptor vd, Direction d){
	switch (numberOfM){
		case (THREE_M):{
			if (g[vd].outcome != simResult::successful){ //accounts for simulation also being safe for now
			if (d ==DEFAULT){
				if (g[vd].nodesInSameSpot<maxNodesOnSpot){
						g[vd].options= {LEFT, RIGHT};
				}
				}
			}
			else { //will only enter if successful
				if (d== LEFT || d == RIGHT){
					g[vd].options = {DEFAULT};
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

void Configurator::applyTransitionMatrix(CollisionGraph & g, vertexDescriptor vd, Direction d, std::vector <Leaf> leaves){
	EndedResult er = controlGoal.checkEnded(g[vd].endPose);
	if (controlGoal.endCriteria.hasEnd()){
		if (er.ended){
			return;
		}
	}
	else if(g[vd].totDs>4){
			return;
		}
	transitionMatrix(g, vd, d);
}



bool Configurator::betterThanLeaves(CollisionGraph &g, vertexDescriptor v, std::vector <Leaf> _leaves, EndedResult& er){
	bool better =1; 
	er = controlGoal.checkEnded(g[v].endPose);
	//ABANDON EARLY IF CURRENT PATH IS MORE COSTLY THAN THE LAST LEAF: if this vertex is the result of more branching while traversing a smaller distance than other leaves, it is more costly
	//if (graphConstruction = BACKTRACKING){
		for (auto l: _leaves){
			if (g[v].outcome == g[l.vertex].outcome){
				if (er.errorFloat<= l.error){
					if (!controlGoal.endCriteria.hasEnd()){
						if (g[v].endPose.p.Length() <= g[l.vertex].endPose.p.Length() ){//&& (g[v].outcome == g[l.vertex].outcome && g[v].totDs>g[l.vertex].totDs)){
							if (g[v].totDs>=g[l.vertex].totDs){
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

	//}
	return better;
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
	for (Point p: current){
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
