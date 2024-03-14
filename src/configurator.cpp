#include "configurator.h"
#include <chrono>

void ConfiguratorInterface::setReady(bool b){
	ready = b;
}


bool ConfiguratorInterface::isReady(){
	return ready;
}

bool Configurator::Spawner(CoordinateContainer data, CoordinateContainer data2fp){ 
	//PREPARE VECTORS TO RECEIVE DATA
	if (data.empty()){
		printf("data empty!\n");
		return 1;
	}
	// CoordinateContainer previous =current;
	// current.clear();
	//current = CoordinateContainer(data);
	//currentBox2D.clear();
	currentBox2D = CoordinateContainer(data2fp);
	iteration++; //iteration set in getVelocity
	worldBuilder.iteration++;

	sprintf(bodyFile, "/tmp/bodies%04i.txt", iteration);
	sprintf(worldBuilder.bodyFile, "%s",bodyFile);
	FILE *f;
	if (debugOn){
		f = fopen(bodyFile, "w");
		fclose(f);
	}
	//BENCHMARK + FIND TRUE SAMPLING RATE
	auto now =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>diff= now - previousTimeScan; //in seconds
	timeElapsed=float(diff.count())/1000; //express in seconds
	//totalTime += timeElapsed; //for debugging
	previousTimeScan=now; //update the time of sampling

	if (timerOff){
		timeElapsed = .2;
	}

	//CREATE BOX2D ENVIRONMENT
	b2Vec2 gravity = {0.0, 0.0};
	b2World world= b2World(gravity);
	char name[256];

	//CALCULATE VELOCITY 
	b2Transform velocity;
	 if (currentTask.action.getOmega()==0){
		float dataRange=0.25;
	 	velocity= sensorTools.affineTransEstimate(std::vector <Pointf>(data.begin(), data.end()), currentTask.action, timeElapsed, dataRange);
		//GetRealVelocity(current, previous); //closed loop, sensor feedback for velocity
	 }
	else{
		velocity = b2Transform(currentTask.getAction().getTransform()); //open loop
	}
	currentTask.action.setRecSpeed(SignedVectorLength(velocity.p));
	currentTask.action.setRecOmega(velocity.q.GetAngle());

	//IF WE  ALREADY ARE IN AN OBSTACLE-AVOIDING STATE, ROUGHLY ESTIMATE WHERE THE OBSTACLE IS NOW
	//bool isObstacleStillThrere = worldBuilder.buildWorld(world, currentBox2D, currentTask.start, currentTask.direction, &currentTask).first;
	if (controlGoal.change){
		Disturbance loopD(PURSUE, -(ogGoal.p));
		controlGoal=Task(loopD,DEFAULT);
	}

	//CHECK IF WITH THE CURRENT currentTask THE ROBOT WILL CRASH
	//isSameTask = wasAvoiding == currentTask.disturbance.isValid();
	simResult result;
	Direction dir;

	auto startTime =std::chrono::high_resolution_clock::now();
	if (planning){ //|| !planError.m_vertices.empty())
		vertexDescriptor startVertex=0; //used to see what would happen if task execution woudl stop now
		vertexDescriptor bestLeaf = startVertex;
		collisionGraph[startVertex].endPose=b2Transform(b2Vec2(0,0), b2Rot(0));
		if (currentVertex==0){
			currentTask.change=1;
			currentTask.H(collisionGraph[currentVertex].disturbance, STOP, 1);
			collisionGraph[currentVertex].fill(simResult());
		}
		if (startVertex !=currentVertex){
			edgeDescriptor e = boost::add_edge(startVertex, currentVertex, collisionGraph).first;
			collisionGraph[e].direction=currentTask.direction;
		}
		//printf("executing = %i", executing);
		//collisionGraph[currentVertex].nObs++;
		//collisionGraph[currentVertex].outcome = simResult::successful;
		explorer(startVertex, collisionGraph, currentTask, world, bestLeaf);
		planVertices= planner(collisionGraph, bestLeaf);
		//boost::remove_edge(startVertex, currentVertex, collisionGraph);
		///currentTask.change=1;
	}
	else if (!planning){
		result = simulate(collisionGraph[currentVertex],collisionGraph[currentVertex],currentTask, world, simulationStep);
		currentTask.change = collisionGraph[currentVertex].outcome==simResult::crashed;
	}
	float duration=0;
	if (benchmark){
	 	auto endTime =std::chrono::high_resolution_clock::now();
	 	std::chrono::duration<float, std::milli>d= startTime- endTime; //in seconds
	 	float duration=abs(float(d.count())/1000); //express in seconds
		FILE * f = fopen(statFile, "a+");
		printf("open stat\n");
		fprintf(f,"%i\t%i\t%f\n", worldBuilder.getBodies(), collisionGraph.m_vertices.size(), duration);
		fclose(f);
		//return 0; //stops when finished and doesn't execute

	}
	worldBuilder.resetBodies();
	//CHOOSE BEXT NEXT Task BASED ON LOOKING AHEAD OF THE PRESENT OBSTACLE

	//IF THE TASK DIDN'T CHANGE, CORRECT PATH
//	if (currentTask.motorStep<pl){
	currentTask.controller(timeElapsed);
//	}

	return 1;
}


std::pair <bool, Direction> Configurator::getOppositeDirection(Direction d){
	std::pair <bool, Direction> result(false, DEFAULT);
		switch (d){
		case Direction::LEFT: result.first = true; result.second = RIGHT;break;
		case Direction::RIGHT: result.first = true; result.second = LEFT;break;
		default:
		break;
	}
	return result;
}

// DeltaPose Configurator::GetRealVelocity(CoordinateContainer &_current, CoordinateContainer &_previous){	 //does not modify current vector, creates copy
// 		DeltaPose result;
// 		float theta=0;
// 		if (iteration==1){
// 			return result;
// 		}
// 	 	theta = currentTask.getAction().getOmega()* timeElapsed;
// 		result.p ={currentTask.getAction().getLinearSpeed()*cos(theta),currentTask.getAction().getLinearSpeed()*sin(theta)};
// 		result.q.Set(currentTask.getAction().getOmega());

//         //adjust for discrepancies in vector size		//int diff = currSize-prevSize;
// 		std::vector <cv::Point2f> currentTmp, previousTmp;
// 		//MAKE OPENCV VECTORS
// 		for (cv::Point2f p:_current){
// 			if (length(p)<.25){
// 				currentTmp.push_back(cv::Point2f(p.x, p.y));
// 			}
// 		}
// 		for (cv::Point2f p: _previous){
// 			if (length(p)<.25){
// 				previousTmp.push_back(cv::Point2f(p.x, p.y));
// 			}
// 		}
// 		int diff = currentTmp.size()-previousTmp.size(); //if +ve,current is bigger, if -ve, previous is bigger

// 		if(diff>0){
// 			if (previousTmp.empty()){
// 				//previousTmp = currentTmp;
// 				return result;
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
// 		else if (diff<0){
// 			if (currentTmp.empty()){
// 				printf("no data\n");
// 					return result;
// 				}
// 			else{
// 				for (int i=0; i<abs(diff); i++){
// 					currentTmp.push_back(currentTmp[0]);
// 				}
// 			}
// 		}
// 	//use partial affine transformation to estimate displacement
// 	cv::Mat transformMatrix =cv::estimateAffinePartial2D(previousTmp, currentTmp, cv::noArray(), cv::LMEDS);
// 	if (!transformMatrix.empty()){
// 		result.p.x= -(transformMatrix.at<double>(0,2))/timeElapsed;
// 		result.p.y = -(transformMatrix.at<double>(1,2))/timeElapsed;
// 		result.q.Set(acos(transformMatrix.at<double>(0,0))/timeElapsed);
// 		float posAngle = atan(result.p.y/result.p.x); //atan2 gives results between pi and -pi, atan gives pi/2 to -pi/2
// 		if (result.p.y ==0 && result.p.x ==0){
// 			posAngle =0;
// 		}
// 		if (result.p.Length()>MAX_SPEED){
// 			result.p.x = currentTask.getAction().getLinearSpeed() *cos(posAngle);
// 			result.p.y = currentTask.getAction().getLinearSpeed() *sin(posAngle);
// 		}

// 	}
// 	return result;
// 	}



// void Configurator::reactiveAvoidance(b2World & world, simResult &r, Task &s){ //returns true if disturbance needs to be eliminated
// 	r =s.willCollide(world, iteration, debugOn, SIM_DURATION, simulationStep);
// 	if (r.resultCode == simResult::crashed){
// 		printf("crashed\n");
// 		//IF THERE IS NO PLAN OR THE Disturbance WE CRASHED INTO IS NOT ALREADY BEING AVOIDED ADD NEW Task TO THE PLAN
// 		Point p(r.collision.getPosition());
// 		if ((!s.disturbance.isValid()|| !(p.isInRadius(s.disturbance.getPosition())))){
// 			s = Task(r.collision, Direction::DEFAULT);
// 		}
// 	}
// }


simResult Configurator::simulate(State& state, State src, Task  t, b2World & w, float _simulationStep){
		//EVALUATE NODE()
	simResult result;
	float remaining = BOX2DRANGE/controlGoal.action.getLinearSpeed();
	//IDENTIFY SOURCE NODE, IF ANY
		if(t.direction == Direction::DEFAULT){
		remaining= (BOX2DRANGE-fabs(src.endPose.p.y))/controlGoal.getAction().getLinearSpeed();			//remaining = (controlGoal.disturbance.getPosition()-g[srcVertex].endPose.p).Length()/controlGoal.getAction().getLinearSpeed();
		}
		if (remaining<0.01){
			remaining=0;
		}
	result =t.willCollide(w, iteration, debugOn, remaining, _simulationStep); //default start from 0
	//FILL IN CURRENT NODE WITH ANY COLLISION AND END POSE
	if (b2Vec2(result.endPose.p -src.endPose.p).Length() <=.01){ //CYCLE PREVENTING HEURISTICS
		state.nodesInSameSpot = src.nodesInSameSpot+1; //keep track of how many times the robot is spinning on the spot
	}
	else{
		state.nodesInSameSpot =0; //reset if robot is moving
	}
	//SET ORIENTATION OF POINT RELATED TO ITS NEIGHBOURS
	return result;
	}


// void Configurator::backtrackingBuildTree(vertexDescriptor v, CollisionGraph& g, Task s, b2World & w, std::vector <vertexDescriptor> &_leaves){
// 	//PRINT DEBUG
// 	//END DEBUG FILE
// 	vertexDescriptor v1=v; 
// 	Direction dir = s.direction;
// 	vertexDescriptor v1Src=v;
//     do{
// 		v= v1;
// 		//evaluate
// 		int ct = w.GetBodyCount();
// 		if (!g[v].filled){
// 			simulate(v, g,s, w);
// 		}
// 		EndedResult er = controlGoal.checkEnded(g[v].endPose);
// 		// if (!hasStickingPoint(g, v, er)&&  betterThanLeaves(g, v, _leaves, er, dir) ){
// 		if (betterThanLeaves(g, v, _leaves, er, dir) ){
// 			applyTransitionMatrix(g,v, s.direction, er.ended, _leaves);
// 		}
// 		if (g[v].options.size()==0){
// 			g[v].heuristic = er.estimatedCost;
// 			_leaves.push_back(v);
// 			backtrack(g, v);
// 		}
// 		if (!addVertex(v, v1, g)){
// 			return;
// 		}
// 		edgeDescriptor v1InEdge = boost::in_edges(v1, g).first.dereference();
// 		v1Src = v1InEdge.m_source;
// 		dir = g[v1InEdge].direction;
// 		s = Task(g[v1Src].disturbance, dir, g[v1Src].endPose);
// 		worldBuilder.buildWorld(w, currentBox2D, g[v1Src].endPose, dir); //was g[v].endPose
// 		// if (benchmark){
// 		// 	printf("bodies in construct= %i\n", w.GetBodyCount());
// 		// }
// 	}while (v1!= v);
// 	//return !g[0].disturbance.safeForNow;
// }


void Configurator::explorer(vertexDescriptor v, CollisionGraph& g, Task t, b2World & w, vertexDescriptor & bestNext){
	vertexDescriptor v1, v0;
	Direction direction= t.direction;
	std::vector <std::pair<vertexDescriptor, float>> priorityQueue = {std::pair(bestNext,0)};
	b2Transform start= b2Transform(b2Vec2(0,0), b2Rot(0));
	do{
		v=bestNext;
		priorityQueue.erase(priorityQueue.begin());
		EndedResult er = controlGoal.checkEnded(g[v]);
		applyTransitionMatrix(g, v, direction, er.ended);
		for (Direction d: g[v].options){ //add and evaluate all vertices
			v0=v; //node being expanded
			v1 =v0; //frontier
			do {
			State s;
			bool topDown=1;
			changeStart(start, v0, g);
			t = Task(g[v0].disturbance, g[v0].options[0], start, topDown);
			float _simulationStep=simulationStep;
			adjustStep(v0, g, t.direction, _simulationStep);
			worldBuilder.buildWorld(w, currentBox2D, t.start, g[v0].options[0]); //was g[v].endPose
			s.fill(simulate(s, g[v0], t, w, _simulationStep)); //find simulation result
			er  = estimateCost(s, g[v0].endPose, t.direction);
			//std:pair<bool, vertexDescriptor> match=matcher.isPerfectMatch(g, v0, t.direction, s);
			std::pair<bool, vertexDescriptor> match=findExactMatch(s, g);			
			if (!match.first){
				addVertex(v0, v1,g, Disturbance(),t.direction);
				g[v1].set(s);
			}
			else{
				g[v0].options.erase(g[v0].options.begin());
				v1=match.second; //frontier
				if (!(v0==v1)){
					edgeDescriptor e= (boost::add_edge(v0, v1, g)).first;
					g[e].direction=t.direction;
				}
				//if there is no e
			}
			applyTransitionMatrix(g, v1, t.direction, er.ended);
			std::vector<std::pair<vertexDescriptor, vertexDescriptor>> toPrune =propagateD(v1, v0, g); //og v1 v0
			pruneTarget(toPrune, g, v);
			v0=v1;
			//check if states need to be pruned retroactively
			}while(t.direction !=DEFAULT & g[v0].options.size()!=0);
			//float phi = er.evaluationFunction();
			addToPriorityQueue(v1, priorityQueue, evaluationFunction(er));
		}
		bestNext=priorityQueue[0].first;
		direction = g[boost::in_edges(bestNext, g).first.dereference()].direction;
	}while(g[bestNext].options.size()!=0);
}


std::vector<std::pair<vertexDescriptor, vertexDescriptor>> Configurator::propagateD(vertexDescriptor v, vertexDescriptor src, CollisionGraph&g){
	std::vector<std::pair<vertexDescriptor, vertexDescriptor>> deletion;
	if (g[v].outcome == simResult::successful ){
		return deletion;
	}
	std::pair <edgeDescriptor, bool> ep= boost::edge(src, v, g);
	Disturbance dist = g[v].disturbance;
	while (ep.second){
		if(g[ep.first].direction!=DEFAULT){
			break;
		}
		if (ep.first.m_target!=v){
			g[ep.first.m_target].disturbance = dist;
			std::pair <bool, vertexDescriptor> match= findExactMatch(ep.first.m_target, g);
			if ( match.first){
			 	deletion.push_back(std::pair<vertexDescriptor, vertexDescriptor>(ep.first.m_target, match.second));
			}
		}
			ep.second= boost::in_degree(ep.first.m_source, g)>0;
			if (!ep.second){
				return deletion;
			}
			ep.first= *(boost::in_edges(ep.first.m_source, g).first);
	}	
	return deletion;
}

void Configurator::pruneTarget(std::vector<std::pair<vertexDescriptor, vertexDescriptor>> vertices, CollisionGraph&g, vertexDescriptor& src, std::vector <std::pair<vertexDescriptor, float>> pq){
	for (std::pair<vertexDescriptor, vertexDescriptor> pair:vertices){
		if (pair.first==src){
			src=pair.second;
			g[pair.second].options= g[pair.first].options;
		}
		edgeDescriptor e = inEdges(g, pair.second, DEFAULT)[0]; //first vertex that satisfies that edge requirement
		boost::clear_in_edges(pair.first, g);
		boost::clear_out_edges(pair.first, g);
		boost::remove_vertex(pair.first, g);
		for (auto i=pq.begin(); i!=pq.end(); i++){ //REMOVE FROM PQ
			if((*i).first==pair.first){
				pq.erase(i);
			}
		}
		adjustProbability(g, e);
	}
}

bool Configurator::edgeExists(vertexDescriptor src, vertexDescriptor target, CollisionGraph& g){
	auto es=boost::in_edges(target, g);
	for (auto ei=es.first; ei!=es.second; ei++){
		if ((*ei).m_source==src){
			return true;
		}
	}
	return false;
}


// Sequence Configurator::getCleanSequence(CollisionGraph&g, vertexDescriptor leaf, vertexDescriptor root){
// 	Sequence p;
// 	if (leaf <root){
// 		throw std::invalid_argument("wrong order of vertices for iteration\n");
// 	}
// 	while (leaf !=root){
// 		if (boost::in_degree(leaf, g)<1){
// 			break;
// 		}
// 		else{
// 		edgeDescriptor e = boost::in_edges(leaf, g).first.dereference(); //get edge
// 		vertexDescriptor src = boost::source(e,g);
// 		if (g[leaf].endPose != g[src].endPose){ //if the node was successful
// 			TaskSummary ts(g[src].disturbance, g[e].direction, g[leaf].step);
// 			p.insert(p.begin(), ts);
// 		}
// 		leaf = src; //go back
// 		}
// 	}
// 	p.insert(p.begin(), TaskSummary(g[0].disturbance, Direction::STOP, 0));
// 	return p;

// }

std::vector <vertexDescriptor> Configurator::planner(CollisionGraph& g, vertexDescriptor leaf, vertexDescriptor root){
	std::vector <vertexDescriptor> vertices;
	if (leaf ==root){
		throw std::invalid_argument("wrong order of vertices for iteration\n");
	}
	while(leaf !=root){
		if (boost::in_degree(leaf, g)<1){
			break;
		}
		else{
			edgeDescriptor e = boost::in_edges(leaf, g).first.dereference(); //get edge
			vertexDescriptor src = boost::source(e,g);
			// if (g[leaf].endPose != g[src].endPose){ //if the node was successful
			if (g[leaf].step>0){
				vertices.insert(vertices.begin(), leaf);
			}
		leaf = src; //go back
		}
	}
	//vertices.insert(vertices.begin(), root);
	return vertices;
}



// Sequence Configurator::getUnprocessedSequence(CollisionGraph&g, vertexDescriptor leaf, vertexDescriptor root){
// 	Sequence p;
// 	if (leaf <root){
// 		throw std::invalid_argument("wrong order of vertices for iteration\n");
// 	}
// 	while (leaf !=root){
// 		if (boost::in_degree(leaf, g)<1){
// 			break;
// 		}
// 		else{
// 		edgeDescriptor e = boost::in_edges(leaf, g).first.dereference(); //get edge
// 		vertexDescriptor src = boost::source(e,g);
// 		Task::Action a;
// 		a.init(g[e].direction);
// 		//float step = motorStep(a);
// 		TaskSummary ts(g[src].disturbance, g[e].direction, g[leaf].step);
// 		p.insert(p.begin(), ts);
// 		leaf = src; //go back
// 		}
// 	}
// 	return p;

// }


EndedResult Configurator::estimateCost(State &state, b2Transform start, Direction d){
	EndedResult er = controlGoal.checkEnded(state);
	Task t(state.disturbance, d, start);
	er.cost += t.checkEnded(state.endPose).estimatedCost;
	return er;
}

// float Configurator::evaluationFunction(EndedResult er){
// 	return abs(er.estimatedCost)+abs(er.cost);
// }

float Configurator::evaluationFunction(EndedResult er){
	return abs(er.estimatedCost)+abs(er.cost);
}

// EndedResult Configurator::estimateCost(vertexDescriptor v,CollisionGraph& g, Direction d){
// 	EndedResult er = controlGoal.checkEnded(g[v]);
// 	//g[v].heuristic = er.estimatedCost;
// 	b2Transform start= b2Transform(b2Vec2(0, 0), b2Rot(0));
// 	edgeDescriptor e;
// 	if(boost::in_degree(v,g)>0){
// 		e = boost::in_edges(v, g). first.dereference();
// 		start =g[e.m_source].endPose;
// 		d = g[e].direction;
// 	}
// 	Task s(g[v].disturbance, d, start);
// 	er.cost += s.checkEnded(g[v].endPose).estimatedCost;
// 	return er;
// }


// Sequence Configurator::getPlan(CollisionGraph &g, vertexDescriptor best){
// 	Sequence p;
// 	edgeDescriptor e;
// 	while (boost::in_degree(best, g)){
// 		best = e.m_source;
// 		Task::Action a;
// 		a.init(g[e].direction);
// 		//float step = motorStep(a);
// 		TaskSummary ts(g[best].disturbance, g[e].direction, g[best].step);
// 		p.insert(p.begin(), ts);

// 	}
// 	return p;
// }

// void Configurator::printPlan(Sequence p){
// 	for (TaskSummary ts: p){
// 		switch (ts.direction){
// 			case DEFAULT: printf("DEFAULT: %i", ts.step);break;
// 			case LEFT: printf("LEFT: %i", ts.step); break;
// 			case RIGHT: printf("RIGHT: %i", ts.step); break;
// 			case BACK: printf("BACK: %i", ts.step); break;
// 			case STOP: printf("STOP");break;
// 			default:break;
// 		}
// 		printf(", ");
// 	}
// 	printf("\n");
// }


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
	//ci->ts = TaskSummary(controlGoal.disturbance, controlGoal.direction, motorStep(controlGoal.action));
}

void Configurator::run(Configurator * c){
	//printf("run\n");
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
			c->ci->ready=0;
			c->Spawner(c->ci->data, c->ci->data2fp);
			//c->ci->ts = TaskSummary(c->currentTask.disturbance, c->currentTask.direction, c->currentTask.motorStep);
		}
	}

}


void Configurator::transitionMatrix(State& state, Direction d){
	Task temp(controlGoal.disturbance, DEFAULT, state.endPose); //reflex to disturbance
	//switch (numberOfM){
	//	case (THREE_M):{
	if (state.outcome != simResult::successful){ //accounts for simulation also being safe for now
		if (d ==DEFAULT ||d==STOP){
			if (state.nodesInSameSpot<maxNodesOnSpot){
				//in order, try the task which represents the reflex towards the goal
				if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
					state.options.push_back(temp.direction);
					state.options.push_back(getOppositeDirection(temp.direction).second);
				}
				else{
					state.options = {LEFT, RIGHT};
				}
			}
			}
	}
	else { //will only enter if successful
		if (d== LEFT || d == RIGHT){
			state.options = {DEFAULT};
		}
		else {
			if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
					state.options.push_back(temp.direction);
					state.options.push_back(getOppositeDirection(temp.direction).second);
				state.options.push_back(DEFAULT);
			}
			else{
					state.options = {DEFAULT, LEFT, RIGHT};
			}

		}

	}
	//	}
	//	break;
	// 	case (FOUR_M):{
	// 		if (g[vd].outcome != simResult::successful){ //accounts for simulation also being safe for now
	// 			if (d ==DEFAULT){
	// 				if (g[vd].nodesInSameSpot<maxNodesOnSpot){
	// 						g[vd].options= {LEFT, RIGHT, BACK};
	// 				}
	// 			}
	// 		}
	// 		else { //will only enter if successful
	// 			if (d== LEFT || d == RIGHT){
	// 				g[vd].options = {DEFAULT};
	// 			}
	// 			else if (d == BACK){
	// 					g[vd].options= {LEFT, RIGHT};
	// 			}
	// 		}
	// 	}
	// 	break;
	// 	default:
	// 	break;
	// }
}

void Configurator::applyTransitionMatrix(CollisionGraph&g, vertexDescriptor v, Direction d, bool ended){
	if (!g[v].options.empty()){
		return;
	}
	if (controlGoal.endCriteria.hasEnd()){
		if (ended){
			return;
		}
	}
	else if(round(g[v].endPose.p.Length()*100)/100>=BOX2DRANGE){ // OR g[vd].totDs>4
		return;
	}
	// for (int i=0; i<planVertices.size(); i++){
	// 	if (v==planVertices[i] & i+1<planVertices.size()){
	// 		auto es = boost::out_edges(v, g);
	// 		for (auto ei=es.first; ei!=es.second; ei++){
	// 			if ((*ei).m_target== planVertices[i+1]){
	// 				g[v].options.push_back(g[*ei].direction);
	// 				return;
	// 			}
	// 		}
	// 	}
	// }
	transitionMatrix(g[v], d);
}



// bool Configurator::betterThanLeaves(CollisionGraph &g, vertexDescriptor v, std::vector <vertexDescriptor> _leaves, EndedResult& er, Direction d){
// 	bool better =1;
// 	er = controlGoal.checkEnded(g[v].endPose); //heuristic
// 	g[v].heuristic = er.estimatedCost;
// 	//expands node if it leads to less error than leaf
// 	for (vertexDescriptor l: _leaves){
// 		if (d==DEFAULT){
// 			if (abs(g[v].evaluationFunction())< abs(g[l].evaluationFunction())){ //if error lower, regardless of distance, keep expanding
// 				if (!controlGoal.endCriteria.hasEnd()){
// 					if (g[v].endPose.p.Length() <= g[l].endPose.p.Length() ){//&& (g[v].outcome == g[l.vertex].outcome && g[v].totDs>g[l.vertex].totDs)){
// 						if (g[v].totDs>=g[l].totDs){
// 							better=0;
// 							break;
// 						}
// 					}
// 				}
// 			}
// 			else{
// 				better =0;
// 				break;
// 			}
// 		}
// 	}
// 	return better;
// }

// bool Configurator::hasStickingPoint(CollisionGraph& g, vertexDescriptor v, EndedResult & er){
// 	bool has =0;
// 	vertexDescriptor src =v;
// 	Point dPosition(g[v].disturbance.getPosition());
// 	//check for repetition along the branch
// 	while (boost::in_degree(src, g)>0 & g[v].disturbance.isValid()){ //&has two step
// 		src =boost::source(boost::in_edges(src, g).first.dereference(), g);
// 		if(dPosition.isInRadius(g[src].disturbance.getPosition(), 0.03)){ //if the current disturbance is within a 3cm radius from a previous one
// 			has=1;
// 			er = controlGoal.checkEnded(g[src].endPose);
// 			break;
// 		}
// 	}
// 	return has;

// }


// void Configurator::backtrack(CollisionGraph&g, vertexDescriptor &v){
// 	while (g[v].options.size()==0){ //keep going back until it finds an incomplete node
// 		if(boost::in_degree(v, g)>0){
// 			edgeDescriptor inEdge = boost::in_edges(v, g).first.dereference();
// 			v = inEdge.m_source;
// 			if (g[v].options.size()>0){ //if if the vertex exiting the while loop is incomplete add a new node
// 				return;
// 			}
// 		}
// 		else{
// 			return;
// 		}
//     }
// }

void Configurator::addToPriorityQueue(vertexDescriptor v, std::vector <std::pair<vertexDescriptor, float>>& queue, float phi){
	for (auto i =queue.begin(); i!=queue.end(); i++){
		if (abs(phi) <abs((*i).second)){
			queue.insert(i, std::pair(v, phi));
			return;
		}
	}
	queue.push_back(std::pair(v, phi));
}


// std::pair <bool, b2Vec2> Configurator::findNeighbourPoint(b2Vec2 v, float radius){ //more accurate orientation
// 	std::pair <bool, b2Vec2> result(false, b2Vec2());
// 	for (Point p: current){
// 		if (p.isInRadius(v, radius)){
// 			return result=std::pair<bool, b2Vec2>(true, p.getb2Vec2());
// 		}
// 	}
// 	return result;
// }

// std::pair <bool, float> Configurator::findOrientation(b2Vec2 v, float radius){
// 	int count=0;
// 	float sumY=0, sumX=0;
// 	float avgY=0, avgX=0;
// 	std::pair <bool, float>result(false, 0);
// 	for (cv::Point2f p:current){
// 		if (p.isInRadius(v, radius)){
// 			auto pIt =current.find(p);
// 			CoordinateContainer::iterator pItNext = pIt++;
// 			float deltaY =pItNext->y- pIt->y;
// 			float deltaX = pItNext->x - pIt->x;
// 				result.first=true; //is there a neighbouring point?
// 				count++;
// 				sumY+=deltaY;
// 				sumX+=deltaX;
// 				avgY = sumY/count;
// 				avgX = sumX/count;
// 		}
// 	}
// 	result.second=atan(avgY/avgX);
// 	return result;
// }


// void Configurator::checkDisturbance(Point p, bool& obStillThere, Task * curr){
// 	if (NULL!=curr){ //
// 		if (p.isInRadius(curr->disturbance.getPosition())){
// 			obStillThere =1;
// 		}
// 	}
// }

void Configurator::adjustProbability(CollisionGraph &g, edgeDescriptor& e){
	//g[e.m_source].nObs++;
	g[e.m_target].nObs++;
	auto es= out_edges(e.m_source, g);
	float totObs=0;
	std::vector <edgeDescriptor> sameTask;
	//find total observations
	for (auto ei= es.first; ei!=es.second; ei++){
		if (g[(*ei)].direction==g[e].direction){
			totObs+=g[(*ei).m_target].nObs;
			sameTask.push_back(*ei);
			//g[*ei].probability=g[e.m_target].nObs/g[e.m_source].nObs;
		}
	}
	//adjust
	// if (sameTask.size()==1){
	// 	return;
	// }
	for (edgeDescriptor ed: sameTask){
		g[ed].probability=g[ed.m_target].nObs/totObs;
	}
}

// std::vector <vertexDescriptor> Configurator::checkPlan(b2World& world, std::vector <vertexDescriptor> & p, CollisionGraph &g, b2Transform start){
// 	std::vector <vertexDescriptor> graphError;
// 	if (p.empty()){
// 		return graphError;
// 	}
// 	Task t= currentTask;
// 	t.check=1;
// 	int it=-1;//this represents the source vertex in edge e
// 	edgeDescriptor e=boost::in_edges(p[0], g).first.dereference();
// 	do {
// 		float stepDistance=BOX2DRANGE;
// 		CoordinateContainer dCloud;
// 		worldBuilder.buildWorld(world, currentBox2D, start, t.direction, &t, &dCloud);
// 		State s;
// 		b2Transform endPose=skip(e,g,it, &t, stepDistance);
// 		s.fill(t.willCollide(world, iteration, debugOn, SIM_DURATION, stepDistance)); //check if plan is successful, simulate
// 		if (s.endPose.p.Length()>endPose.p.Length()){
// 			s.endPose=endPose;
// 			s.outcome=simResult::successful;
// 		}
// 		start = s.endPose;
// 		DistanceVector distance = matcher.getDistance(g[planVertices[it]], s);
// 		if (!matcher.isPerfectMatch(distance)){
// 			vertexDescriptor v;
// 			addVertex(planVertices[it-1], v,g, Disturbance(), g[e].direction, 1);
// 			g[v].set(s);
// 		}
// 		else{
// 			g[planVertices[it]].nObs++;
// 		}
// 		if (s.outcome == simResult::crashed){ //has to replan
// 			for (int i=it; i<p.size();i++){
// 				graphError.push_back(p[i]);
// 			}
// 			break;
// 		}
// 		//it++;
// 		//e=boost::in_edges(p[it], g).first.dereference();
// 		t= Task(g[e.m_source].disturbance, g[e].direction, start, true);
// 		t.check=1;
// 	}while (it+2<p.size());
// 	return graphError;
// }

std::vector <edgeDescriptor> Configurator::outEdges(CollisionGraph&g, vertexDescriptor v, Direction d){
	std::vector <edgeDescriptor> result;
	auto es = boost::out_edges(v, g);
	for (auto ei = es.first; ei!=es.second; ++ei){
		if (g[(*ei)].direction == d){
			result.push_back(*ei);
		}
	}
	return result;
}

std::vector <edgeDescriptor> Configurator::inEdges(CollisionGraph&g, vertexDescriptor v, Direction d){
	std::vector <edgeDescriptor> result;
	auto es = boost::in_edges(v, g);
	for (auto ei = es.first; ei!=es.second; ++ei){
		if (g[(*ei)].direction == d){
			result.push_back(*ei);
		}
	}
	return result;
}

std::pair <edgeDescriptor, bool> Configurator::maxProbability(std::vector<edgeDescriptor> ev, CollisionGraph& g){
	std::pair <edgeDescriptor, bool> result;
	if (ev.empty()){
		result.second=false;
		return result;
	}
	result.first = ev[0];
	result.second=true;
	for (edgeDescriptor e :ev){
		if (g[e].probability>g[result.first].probability){
			result.first =e;
		}
	}
	return result;
}



void Configurator::adjustStep(vertexDescriptor v, CollisionGraph &g, Direction d, float& step){
	// if (boost::out_degree(v, g)==0 || boost::in_degree(v,g)==0 || planVertices.empty()){
	// 	return;
	// }
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, g);
	if(!ep.second){
		return;
	}
	edgeDescriptor e= ep.first;
	if (g[e].direction!=d){
		return;
	}
	int stepsTraversed= collisionGraph[currentVertex].step-currentTask.motorStep;
	if (currentTask.getAction().getOmega()!=0){
		float remainingAngle = currentTask.endCriteria.angle.get()-abs(stepsTraversed*currentTask.action.getOmega());
		//remainingAngle+=fabs(g[e.m_source].endPose.q.GetAngle() -g[e.m_target].endPose.q.GetAngle());
		currentTask.setEndCriteria(Angle(remainingAngle));
	}
	if(currentTask.getAction().getLinearSpeed()>0){
		step-= (stepsTraversed*MOTOR_CALLBACK)*currentTask.action.getLinearSpeed();
	}			// -estimated distance covered

}

std::vector <edgeDescriptor> Configurator::inEdgesRecursive(vertexDescriptor v, CollisionGraph& g, Direction d){
	 	std::vector <edgeDescriptor> result;
		edgeDescriptor e; 
		do{
			std::vector <edgeDescriptor> directionEdges=inEdges(g, v, d);
			if (directionEdges.empty()){
				break;
			}
			auto eit=std::max_element(directionEdges.begin(), directionEdges.end()); //choosing the most recent edge
			e = *eit;
			result.push_back(e);
			v=e.m_source;
		}while (g[e].direction==d);
	return result;
} 

std::pair <bool, vertexDescriptor> Configurator::findExactMatch(State s, CollisionGraph& g){
	std::pair <bool, vertexDescriptor> result(false, CollisionGraph::null_vertex());
	auto vs= boost::vertices(g);
	for (auto vi=vs.first; vi!= vs.second; vi++){
		if (matcher.isPerfectMatch(g[*vi], s)){
			result.first=true;
			result.second=*vi;
			break;
		}

	}
	return result;
}

std::pair <bool, vertexDescriptor> Configurator::findExactMatch(vertexDescriptor v, CollisionGraph& g){
	std::pair <bool, vertexDescriptor> result(false, CollisionGraph::null_vertex());
	auto vs= boost::vertices(g);
	for (auto vi=vs.first; vi!= vs.second; vi++){
		if (*vi!=v){
			if (matcher.isPerfectMatch(g[*vi], g[v])){
			result.first=true;
			result.second=*vi;
			break;
		}
		}


	}
	return result;
}

void Configurator::changeStart(b2Transform& start, vertexDescriptor v, CollisionGraph& g){
	if (g[v].outcome == simResult::crashed && boost::in_degree(v, g)>0){
		edgeDescriptor e = boost::in_edges(v, g).first.dereference();
		start = g[e.m_source].endPose;
	}
	else{
		start=g[v].endPose;
	}
}



// b2Transform Configurator::skip(edgeDescriptor& e, CollisionGraph &g, Task* t, float& step){
// 	//	step=BOX2DRANGE; //set to range in order to discover other disturbances

// 	// if (g[e.m_source].disturbance.isValid()){
// 	// 	step=b2Vec2(g[e.m_source].endPose.p-g[e.m_source].disturbance.pose.p).Length();
// 	// }
// 	while (g[e].direction==t->direction & i+1<planVertices.size()){
// 		//if (t->endCriteria.angle.isValid()){
// 		i++;
// 		if (boost::out_degree(e.m_target,g)>0){
// 			auto es = boost::out_edges(e.m_target, g);
// 			for (auto ei = es.first; ei!=es.second; ++ei){
// 				if ((*ei).m_target == planVertices[i+1]){
// 					e= (*ei);
// 					break;
// 				}
// 			}
// 		}
// 		else{
// 			break;
// 		}
// 		}

// 	return g[planVertices[i]].endPose;

// }

// std::pair<bool, vertexDescriptor> Configurator::findBestMatch(State s){
// 	//std::vector <vertexDescriptor> matches;
// 	//vertexDescriptor v = boost::add_vertex(g);
// 	//g[v].fill(sim);
// 	std::pair<bool, vertexDescriptor> result(0, -1);
// 	float bestDistance=10000;
// 	for (vertexDescriptor stateV: collisionGraph.m_vertices){
// 		DistanceVector dv = matcher.getDistance(collisionGraph[stateV], s);
// 		if (matcher.isPerfectMatch(dv) & matcher.sumVector(dv)<bestDistance){
// 			result.second= stateV;
// 			result.first=true;
// 		}
// 	}
// 	return result;
// }


void Configurator::trackTaskExecution(Task & t){
	//if (t.endCriteria.hasEnd()){
		//printf("task in %i has end\n", iteration);
		if (t.motorStep>0){
			t.motorStep--;
			printf("step =%i\n", t.motorStep);
		}
		if(t.motorStep==0){
			t.change=1;
		}
	updateGraph(collisionGraph);
}

DeltaPose Configurator::assignDeltaPose(Task::Action a, float timeElapsed){
	DeltaPose result;
	float theta = a.getOmega()* timeElapsed;
	result.p ={a.getLinearSpeed()*cos(theta),a.getLinearSpeed()*sin(theta)};
	result.q.Set(a.getOmega());
	return result;
}

// int Configurator::motorStep(Task::Action a, EndCriteria ec){
// 	int result=0, angleResult=0, distanceResult=0;
// 	if (ec.angle.isValid()){
// 		angleResult = ec.angle.get()/(MOTOR_CALLBACK * a.getOmega());
// 	}
// 	if (ec.distance.isValid()){
// 		distanceResult = ec.distance.get()/(MOTOR_CALLBACK * a.getLinearSpeed());
// 	}
// 	result =std::max(angleResult, distanceResult);
// 	printf("task has %i steps\n", result);
// 	return result;
// }

int Configurator::motorStep(Task::Action a){
	int result=0;
        if (a.getOmega()>0){ //LEFT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
		    //result *=FRICTION_DAMPENING;
			//result =12;
        }
		else if (a.getOmega()<0){ //RIGHT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
		    //result *=FRICTION_DAMPENING;
			//result=12;
		}
		else if (a.getLinearSpeed()>0){
			result = (simulationStep)/(MOTOR_CALLBACK*a.getLinearSpeed());
		}
	    return abs(result);
    }


// void Configurator::changeTask(bool b, Sequence & p, State n, int&ogStep){
// 	// if (currentTask.motorStep==0){
// 	// 	b=1;
// 	// }
// 	if (!b){
// 		return;
// 	}
// 	if (!p.empty()){
// 		if (p.size()==1){
// 			running=0;
// 		}
// 		p.erase(p.begin());
// 	}
// 	if (planning){
// 		if (plan.empty()){
// 			return;
// 		}
// 		currentTask = Task(p[0].disturbance, p[0].direction);
// 		currentTask.motorStep = p[0].step;
// 		//p.erase(p.begin());
// 		printf("canged to next in plan, new task has %i steps\n", currentTask.motorStep);
// 	}
// 	else{
// 		if (n.disturbance.isValid()){
// 			currentTask = Task(n.disturbance, DEFAULT); //reactive
// 		}
// 		else if(currentTask.direction!=DEFAULT){
// 				currentTask = Task(n.disturbance, DEFAULT); //reactive
// 		}
// 		else{
// 			currentTask = Task(controlGoal.disturbance, DEFAULT); //reactive
// 		}

// 		currentTask.motorStep = motorStep(currentTask.getAction()); //reflex
// 		printf("changed to reactive, %i steps\n", currentTask.motorStep);
// 	}
// 	//currentTask.motorStep = motorStep(currentTask.getAction());
// 	ogStep = currentTask.motorStep;
// 	//printf("set step\n");
// }

void Configurator::changeTask(bool b, int &ogStep){
	// if (currentTask.motorStep==0){
	// 	b=1;
	// }
	if (!b){
		//printf("do not change\n");
		return;
	}
	// if (!planVertices.empty()){
	// 	// if (planVertices.size()==1){
	// 	// 	running=0;
	// 	// }
	// 	planVertices.erase(planVertices.begin());
	// }
	if (planning){
		if (planVertices.empty()){
			currentVertex=0;
			//currentTask = controlGoal;
			return;
		}
		currentVertex= planVertices[0];
		std::pair<edgeDescriptor, bool> ep=boost::edge(0, currentVertex, collisionGraph);
		if (!ep.second){
			throw std::invalid_argument("no plan was formed");
			return;
		}
		edgeDescriptor e= ep.first;
		boost::clear_out_edges(0, collisionGraph);
		planVertices.erase(planVertices.begin());
		currentTask = Task(collisionGraph[e.m_source].disturbance, collisionGraph[e].direction, b2Transform(b2Vec2(0,0), b2Rot(0)), true);
		currentTask.motorStep = collisionGraph[currentVertex].step;
		// if (currentVertex!=0){
		// 	boost::clear_out_edges(0, collisionGraph);
		// }
		// //planVertices.erase(planVertices.begin());
		//printf("canged to next in plan, new task has %i steps\n", currentTask.motorStep);
	}
	else{
		if (collisionGraph[0].disturbance.isValid()){
			currentTask = Task(collisionGraph[0].disturbance, DEFAULT); //reactive
			//currentTask.motorStep = motorStep(currentTask.getAction());
		}
		else if(currentTask.direction!=DEFAULT){
				currentTask = Task(collisionGraph[0].disturbance, DEFAULT); //reactive
		}
		else{
			currentTask = Task(controlGoal.disturbance, DEFAULT); //reactive
		}

		currentTask.motorStep = motorStep(currentTask.getAction());
		printf("changed to reactive\n");
	}
	//currentTask.motorStep = motorStep(currentTask.getAction());
	ogStep = currentTask.motorStep;
	//printf("set step\n");
}

void Configurator::updateGraph(CollisionGraph&g){
	b2Transform deltaPose(b2Vec2(getTask()->getAction().getLinearVelocity().x*MOTOR_CALLBACK,
						getTask()->getAction().getLinearVelocity().y*MOTOR_CALLBACK), 
						b2Rot(getTask()->getAction().getOmega()*MOTOR_CALLBACK));
	auto vPair =boost::vertices(g);
	for (auto vIt= vPair.first; vIt!=vPair.second; ++vIt){
		g[*vIt].endPose-=deltaPose;
		g[*vIt].disturbance.pose-=deltaPose;
	}
}