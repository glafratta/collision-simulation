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
		edgeDescriptor movingEdge;
		if (currentVertex==movingVertex){
			currentVertex=boost::add_vertex(transitionSystem);
			currentTask.action.setVelocities(0,0);
		}
		movingEdge = boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
		transitionSystem[movingVertex].disturbance=transitionSystem[currentVertex].disturbance;
		transitionSystem[movingVertex].disturbance.invalidate();
		if (debugOn){
			printf("moving edge= %i -> %i\n", movingEdge.m_source, movingEdge.m_target);
		}
			//errorMap.emplace(transitionSystem[currentEdge].ID , 0);
		transitionSystem[movingEdge].direction=currentTask.direction;
		transitionSystem[movingEdge].step=currentTask.motorStep;
		std::vector <std::pair <vertexDescriptor, vertexDescriptor>> toRemove;
		vertexDescriptor src;
		if (iteration>1){
			src=movingVertex;
		}
		else{
		 	currentTask.action.setVelocities(0,0);
			src=currentVertex;
		}
		resetPhi(transitionSystem);
		toRemove=explorer(src, transitionSystem, currentTask, world);
		clearFromMap(toRemove, transitionSystem, errorMap);
		Connected connected(&transitionSystem);
		FilteredTS fts(transitionSystem, boost::keep_all(), connected);
		TransitionSystem tmp;
		boost::copy_graph(fts, tmp);
		transitionSystem.clear();
		transitionSystem.swap(tmp);
		planVertices= planner(transitionSystem, src);
		if (debugOn){
			printPlan();
			//boost::print_graph(transitionSystem);
		}

	}
	else if (!planning){
		result = simulate(transitionSystem[currentVertex],transitionSystem[currentVertex],currentTask, world, simulationStep);
		currentTask.change = transitionSystem[currentVertex].outcome==simResult::crashed;
	}
	float duration=0;
	auto endTime =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>d= startTime- endTime; //in seconds
 	duration=abs(float(d.count())/1000); //express in seconds
	printf("took %f seconds\n", duration);
	if (benchmark){
		FILE * f = fopen(statFile, "a+");
		printf("open stat\n");
		fprintf(f,"%i\t%i\t%f\n", worldBuilder.getBodies(), transitionSystem.m_vertices.size(), duration);
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

void Configurator::resetPhi(TransitionSystem&g){
	auto vs=boost::vertices(g);
	for (auto vi=vs.first; vi!=vs.second; vi++){
		g[*vi].resetVisited();
	}
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

Disturbance Configurator::getDisturbance(TransitionSystem&g, vertexDescriptor v){
	if (!g[v].disturbance.isValid()){
		return controlGoal.disturbance;
	}
	return g[v].disturbance;
}


simResult Configurator::simulate(State& state, State src, Task  t, b2World & w, float _simulationStep){
		//EVALUATE NODE()
	simResult result;
	float distance=BOX2DRANGE;
	if (controlGoal.disturbance.isValid()){
		distance= controlGoal.disturbance.getPosition().Length();
	}
	float remaining =distance/controlGoal.action.getLinearSpeed();
	//IDENTIFY SOURCE NODE, IF ANY
		if(t.direction == Direction::DEFAULT){
		remaining= (distance-fabs(src.endPose.p.y))/controlGoal.getAction().getLinearSpeed();			//remaining = (controlGoal.disturbance.getPosition()-g[srcVertex].endPose.p).Length()/controlGoal.getAction().getLinearSpeed();
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
	if(!result.collision.isValid()){
		return result;
	}
	std::vector <Pointf> nb=neighbours(result.collision.getPosition(), 0.025);
	cv::Rect2f rect =worldBuilder.getRect(nb);
	std::pair<bool, float> orientation =findOrientation(nb);
	result.collision.bf.halfLength=rect.width/2;
	result.collision.bf.halfLength=rect.height/2;
	result.collision.setOrientation(orientation.second);
	return result;
	}


// void Configurator::backtrackingBuildTree(vertexDescriptor v, TransitionSystem& g, Task s, b2World & w, std::vector <vertexDescriptor> &_leaves){
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


std::vector <std::pair<vertexDescriptor, vertexDescriptor>>Configurator::explorer(vertexDescriptor v, TransitionSystem& g, Task t, b2World & w){
	vertexDescriptor v1, v0, bestNext=v;
	Direction direction= t.direction;
	std::vector <std::pair<vertexDescriptor, float>> priorityQueue = {std::pair(bestNext,0)};
	b2Transform start= b2Transform(b2Vec2(0,0), b2Rot(0));
	std::vector<std::pair<vertexDescriptor, vertexDescriptor>> toRemove;
	// if (debugOn){
	// 	auto vs=boost::vertices(g);
	// 	printf("previously in graph:");
	// 	for (auto vi=vs.first; vi!=vs.second; vi++){
	// 		printf(" v%i", (*vi));
	// 	}
	// 	printf("\n");
	// }
	do{
		v=bestNext;
		priorityQueue.erase(priorityQueue.begin());
		EndedResult er = controlGoal.checkEnded(g[v]);
		applyTransitionMatrix(g, v, direction, er.ended);
		for (Direction d: g[v].options){ //add and evaluate all vertices
			v0=v; //node being expanded
			v1 =v0; //frontier
			do {
			std::pair <State, Edge> sk;
			bool topDown=1;
			changeStart(start, v0, g);
			t = Task(getDisturbance(g, v0), g[v0].options[0], start, topDown);
			float _simulationStep=simulationStep;
			adjustStepDistance(v0, g, t.direction, _simulationStep);
			Disturbance expectedD=gt::getExpectedDisturbance(g, v0, t.direction);
			worldBuilder.buildWorld(w, currentBox2D, t.start, t.direction); //was g[v].endPose
			setStateLabel(sk.first, g[v0].ID, t.direction); //new
			sk =gt::fill(simulate(sk.first, g[v0], t, w, _simulationStep)); //find simulation result
			sk.second.direction=t.direction;
			er  = estimateCost(sk.first, g[v0].endPose, t.direction);
			State * source=NULL;
			bool vm= matcher.isPerfectMatch(g[v], g[currentEdge.m_source]);
			if (v0==movingVertex & vm){
				source= g[currentVertex].ID;
			}
			else{
				source=g[v0].ID;
			}
			std::pair<bool, vertexDescriptor> match=findExactMatch(sk.first, g, source, t.direction);			
			std::pair <edgeDescriptor, bool> edge(edgeDescriptor(), false);
			if (!match.first){
				edge= addVertex(v0, v1,g, Disturbance(),sk.second); //new edge, valid
			}
			else{
				g[v0].options.erase(g[v0].options.begin());
				v1=match.second; //frontier
				if (!(v0==v1)){
					edge= (boost::edge(v0, v1, g)); //assumes edge added
					edge.second=true; //just means that the edge is valid
					g[edge.first]=sk.second;//t.direction;
				}
			}
			if(edge.second){
				gt::set(edge.first, sk, g, v1==currentVertex, errorMap);
				gt::adjustProbability(g, edge.first);
			}
			// if (debugOn){
			// 	printf("v= %i, start = (%f, %f, %f\n)", v1, start.p.x, start.p.y, start.q.GetAngle());
			// }
			applyTransitionMatrix(g, v1, t.direction, er.ended);
			g[v1].phi=evaluationFunction(er);
			std::vector<std::pair<vertexDescriptor, vertexDescriptor>> toPrune =(propagateD(v1, v0, v,g)); //og v1 v0
			v0=v1;
			pruneEdges(toPrune,g, v, priorityQueue, toRemove);
			}while(t.direction !=DEFAULT & g[v0].options.size()!=0);
			addToPriorityQueue(v1, priorityQueue, g[v1].phi);
		}
		bestNext=priorityQueue[0].first;
		direction = g[boost::in_edges(bestNext, g).first.dereference()].direction;
	}while(g[bestNext].options.size()>0);
	return toRemove;
}


std::vector<std::pair<vertexDescriptor, vertexDescriptor>> Configurator::propagateD(vertexDescriptor v1, vertexDescriptor v0, vertexDescriptor&v,TransitionSystem&g){
	std::vector<std::pair<vertexDescriptor, vertexDescriptor>> deletion;
	if (g[v1].outcome == simResult::successful ){
		return deletion;
	}
	std::pair <edgeDescriptor, bool> ep= boost::edge(v0, v1, g);
	Disturbance dist = g[v1].disturbance;
	while (ep.second){
		if(g[ep.first].direction!=DEFAULT){
			break;
		}
		if (ep.first.m_target!=v1){
			g[ep.first.m_target].disturbance = dist;
			std::pair <bool, vertexDescriptor> match= findExactMatch(ep.first.m_target, g, g[ep.first].direction);
			if ( match.first){
				std::pair<vertexDescriptor, vertexDescriptor>pair(ep.first.m_target, match.second);
				deletion.push_back(pair);			}
		}
		ep.second= boost::in_degree(ep.first.m_source, g)>0;
		if (!ep.second){
			return deletion;
		}
		ep.first= *(boost::in_edges(ep.first.m_source, g).first);
	}	
	return deletion;
}

void Configurator::pruneEdges(std::vector<std::pair<vertexDescriptor, vertexDescriptor>> vertices, TransitionSystem& g, vertexDescriptor& src, std::vector <std::pair<vertexDescriptor, float>>& pq, std::vector<std::pair<vertexDescriptor, vertexDescriptor>>&toRemove){ //clears edges out of redundant vertices, removes the vertices from PQ, returns vertices to remove at the end
	for (std::pair<vertexDescriptor, vertexDescriptor> pair:vertices){
		if (pair.first==src){
			src=pair.second;
		}
		edgeDescriptor e =inEdges(g, pair.second, DEFAULT)[0]; //first vertex that satisfies that edge requirement
		std::vector <edgeDescriptor> toReassign=inEdges(g, pair.first, DEFAULT);
		edgeDescriptor e2 = gt::visitedEdge(toReassign, g);
		toReassign.push_back(e);
		gt::update(e, std::pair <State, Edge>(g[pair.first], g[e2]),g, pair.second==currentVertex, errorMap);
		for (edgeDescriptor r:toReassign){ //reassigning edges
			std::pair <edgeDescriptor, bool> ep =boost::add_edge(r.m_source, pair.second, g);
			//adjustProbability(g, ep.first);
		}
		boost::clear_vertex(pair.first, g);
		toRemove.push_back(pair);
		for (int i=0; i<pq.size(); i++){ //REMOVE FROM PQ
			if(pq[i].first==pair.first){
				pq.erase(pq.begin()+i);
			}
		}
		gt::adjustProbability(g, e);
	}
}

void Configurator::clearFromMap(std::vector<std::pair<vertexDescriptor, vertexDescriptor>> matches , TransitionSystem&g, std::unordered_map<State*, float>map){
	//auto es=boost::edges(g);
	//for (auto ei=es.first; ei!=es.second; ei++){
		for (std::pair<vertexDescriptor, vertexDescriptor> pair:matches){
			if (auto it=map.find(g[pair.first].ID); it!=map.end()){
				float value = map.at(transitionSystem[pair.first].ID);
				map.insert_or_assign(transitionSystem[pair.second].ID, value);
				map.erase(it);
				break;
			}

		}
	//}
}

bool Configurator::edgeExists(vertexDescriptor src, vertexDescriptor target, TransitionSystem& g){
	auto es=boost::in_edges(target, g);
	for (auto ei=es.first; ei!=es.second; ei++){
		if ((*ei).m_source==src){
			return true;
		}
	}
	return false;
}


// Sequence Configurator::getCleanSequence(TransitionSystem&g, vertexDescriptor leaf, vertexDescriptor root){
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

std::vector <vertexDescriptor> Configurator::planner(TransitionSystem& g, vertexDescriptor src){
	std::vector <vertexDescriptor> plan;
	vertexDescriptor connecting; //og: src=currentV
	std::vector <edgeDescriptor> frontier;
	bool run=true;
	do{
		frontier=frontierVertices(src, g, DEFAULT);
		connecting=TransitionSystem::null_vertex();
		float phi=2; //very large phi, will get overwritten
		bool changed_src=false;
		for (edgeDescriptor e:frontier){
			planPriority(g, e.m_target);
			if (g[e.m_target].phi<phi){
					phi=g[e.m_target].phi;
					if (e.m_source!=src){
						connecting=e.m_source;
					}
					//if (e.m_source !=currentVertex){
					src=e.m_target;
					changed_src=true;
					//}
			}
			else if (g[e.m_target].label==ESCAPE){
				boost::clear_vertex(e.m_target, g);
			}
			else if (g[e.m_source].label==ESCAPE){
				boost::clear_vertex(e.m_source, g);
			}
		}
		if (connecting!=TransitionSystem::null_vertex()){
			g[connecting].label=VERTEX_LABEL::UNLABELED;
			plan.push_back(connecting);
		}
		//if (!frontier.empty()){
		if (changed_src){
			if (src!=currentVertex){
				plan.push_back(src);
				g[src].label=UNLABELED;
			}
		}
		else{
			run=false;
			break;
		}
	}while(run);
	return plan;
}



std::vector <vertexDescriptor> Configurator::back_planner(TransitionSystem& g, vertexDescriptor leaf, vertexDescriptor root){
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
			if (g[e].step>0){
				vertices.insert(vertices.begin(), leaf);
			}
		leaf = src; //go back
		}
	}
	//vertices.insert(vertices.begin(), root);
	return vertices;
}



// Sequence Configurator::getUnprocessedSequence(TransitionSystem&g, vertexDescriptor leaf, vertexDescriptor root){
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


float Configurator::evaluationFunction(EndedResult er){
	// float planPriority=0.0;
    // for (vertexDescriptor p:planVertices){
	// 	if (p==v){
    //    		planPriority=0.0;
	// 		break;
	// 	}
    // } 
	return (abs(er.estimatedCost)+abs(er.cost))/2; //normalised to 1
}

// EndedResult Configurator::estimateCost(vertexDescriptor v,TransitionSystem& g, Direction d){
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


// Sequence Configurator::getPlan(TransitionSystem &g, vertexDescriptor best){
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

void Configurator::printPlan(){
	std::map<Direction, char*> dirmap={{DEFAULT, "DEFAULT"}, {LEFT, "LEFT"}, {RIGHT, "RIGHT"}, {STOP, "STOP"}, {UNDEFINED, "UNDEFINED"}, {BACK, "BACK"}};
	vertexDescriptor pre=movingVertex;
	for (vertexDescriptor v: planVertices){
		std::pair <edgeDescriptor, bool> edge=boost::edge(pre, v, transitionSystem);
		//auto a=dirmap.find(transitionSystem[edge.first].direction);
		if (!edge.second){
			//throw std::exception();
			//auto a=dirmap.find(transitionSystem[edge.first].direction);
			printf("no edge: %i-> ", edge.first.m_source);
		}
		else{
			auto a=dirmap.find(transitionSystem[edge.first].direction);
			printf("%i, %s, ", edge.first.m_target, (*a).second);
		}
		pre=edge.first.m_target;
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

void Configurator::applyTransitionMatrix(TransitionSystem&g, vertexDescriptor v, Direction d, bool ended){
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
	transitionMatrix(g[v], d);
}



// bool Configurator::betterThanLeaves(TransitionSystem &g, vertexDescriptor v, std::vector <vertexDescriptor> _leaves, EndedResult& er, Direction d){
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

// bool Configurator::hasStickingPoint(TransitionSystem& g, vertexDescriptor v, EndedResult & er){
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


// void Configurator::backtrack(TransitionSystem&g, vertexDescriptor &v){
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


std::vector <Pointf> Configurator::neighbours(b2Vec2 pos, float radius){ //more accurate orientation
	std::vector <Pointf> result;
	for (Pointf p: sensorTools.previous){
		cv::Rect2f rect(pos.x-radius, pos.y+radius, radius, radius);//tl, br, w, h
		if (p.inside(rect)){
			result.push_back(p);
		}
	}
	return result;
}

std::pair <bool, float> Configurator::findOrientation(std::vector<Pointf> vec){
	int count=0;
	float sumY=0, sumX=0;
	float avgY=0, avgX=0;
	std::pair <bool, float>result(false, 0);
	for (Pointf p:vec){
		//cv::Rect2f rect(pos.x-radius, pos.y+radius, radius, radius);//tl, br, w, h
		//if (p.inside(rect)){
			std::set <Pointf>set=vec2set(vec);
			auto pIt =set.find(p);
			CoordinateContainer::iterator pItNext = pIt++;
			float deltaY =pItNext->y- pIt->y;
			float deltaX = pItNext->x - pIt->x;
				result.first=true; //is there a neighbouring point?
				count++;
				sumY+=deltaY;
				sumX+=deltaX;
				avgY = sumY/count;
				avgX = sumX/count;
		//}
	}
	result.second=atan(avgY/avgX);
	return result;
}


// void Configurator::checkDisturbance(Point p, bool& obStillThere, Task * curr){
// 	if (NULL!=curr){ //
// 		if (p.isInRadius(curr->disturbance.getPosition())){
// 			obStillThere =1;
// 		}
// 	}
// }

// void Configurator::adjustProbability(TransitionSystem &g, edgeDescriptor& e){
// 	//g[e.m_source].nObs++;
// 	//g[e.m_target].nObs++;
// 	auto es= out_edges(e.m_source, g);
// 	float totObs=0;
// 	std::vector <edgeDescriptor> sameTask;
// 	//find total observations
// 	for (auto ei= es.first; ei!=es.second; ei++){
// 		if (g[(*ei)].direction==g[e].direction){
// 			totObs+=g[(*ei).m_target].nObs;
// 			sameTask.push_back(*ei);
// 			//g[*ei].probability=g[e.m_target].nObs/g[e.m_source].nObs;
// 		}
// 	}
// 	//adjust
// 	// if (sameTask.size()==1){
// 	// 	return;
// 	// }
// 	for (edgeDescriptor ed: sameTask){
// 		g[ed].probability=g[ed.m_target].nObs/totObs;
// 	}
// }

// std::vector <vertexDescriptor> Configurator::checkPlan(b2World& world, std::vector <vertexDescriptor> & p, TransitionSystem &g, b2Transform start){
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
// 		DistanceVector distance = matcher.getDistance(g[planVerti:ces[it]], s);
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



std::vector <edgeDescriptor> Configurator::inEdges(TransitionSystem&g, vertexDescriptor v, Direction d){
	std::vector <edgeDescriptor> result;
	auto es = boost::in_edges(v, g);
	for (auto ei = es.first; ei!=es.second; ++ei){
		if (g[(*ei)].direction == d){
			result.push_back(*ei);
		}
	}
	return result;
}

std::pair <edgeDescriptor, bool> Configurator::maxProbability(std::vector<edgeDescriptor> ev, TransitionSystem& g){
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



void Configurator::adjustStepDistance(vertexDescriptor v, TransitionSystem &g, Direction d, float& step){
	// if (boost::out_degree(v, g)==0 || boost::in_degree(v,g)==0 || planVertices.empty()){
	// 	return;
	// }
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, g);
	if(!ep.second){
		return;
	}
	auto eb=boost::edge(currentEdge.m_source,currentEdge.m_target, transitionSystem);
	// if (g[currentEdge].direction!=d){
	// 	return;
	// }
	int stepsTraversed= g[eb.first].step-currentTask.motorStep;
	if (currentTask.getAction().getOmega()!=0){
		float remainingAngle = currentTask.endCriteria.angle.get()-abs(stepsTraversed*currentTask.action.getOmega());
		//remainingAngle+=fabs(g[e.m_source].endPose.q.GetAngle() -g[e.m_target].endPose.q.GetAngle());
		currentTask.setEndCriteria(Angle(remainingAngle));
	}
	if(currentTask.getAction().getLinearSpeed()>0){
		step-= (stepsTraversed*MOTOR_CALLBACK)*currentTask.action.getLinearSpeed();
	}			// -estimated distance covered

}

std::vector <edgeDescriptor> Configurator::inEdgesRecursive(vertexDescriptor v, TransitionSystem& g, Direction d){
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

std::vector <edgeDescriptor> Configurator::frontierVertices(vertexDescriptor v, TransitionSystem& g, Direction d){
	std::vector <edgeDescriptor> result;
	std::pair<edgeDescriptor, bool> ep=boost::edge(movingVertex, v, g); 
	std::vector <vertexDescriptor>connecting;
	do{
		if ((controlGoal.disturbance.getPosition()-g[v].endPose.p).Length() < DISTANCE_ERROR_TOLERANCE){
			break;
		}
		auto es=boost::out_edges(v, g);
		for (auto ei=es.first; ei!=es.second; ei++){
			if (g[(*ei).m_target].visited()){
				if (g[*ei].direction==d){
					result.push_back((*ei)); //assumes maximum depth of 2 vertices traversed
				}
				else{
					connecting.push_back((*ei).m_target);
				}
			}
		}
		if(v==currentVertex & result.empty() 
			& es.first!=es.second & ep.second){
			v=ep.first.m_source;
		}
		else if (result.empty()&connecting.empty()){
			break;
		}
		if (!connecting.empty()){
			v=connecting[0];
			connecting.erase(connecting.begin());
		}
		else{
			break;
		}
	}while (true);
	return result;
}

std::pair <bool, vertexDescriptor> Configurator::findExactMatch(State s, TransitionSystem& g, State * src, Direction dir){
	std::pair <bool, vertexDescriptor> result(false, TransitionSystem::null_vertex());
	auto vs= boost::vertices(g);
	//MoreLikely more_likely;
	//float nObs=0;
	float prob=0;
	for (auto vi=vs.first; vi!= vs.second; vi++){
		vertexDescriptor v=*vi;
		bool Tmatch=true;
		std::vector <edgeDescriptor> ie=inEdges(g, v, dir);
		//if (dir!=Direction::UNDEFINED){
		Tmatch=!ie.empty()||dir==Direction::UNDEFINED;
		//}
		if (matcher.isPerfectMatch(s, g[v], src) & v!=movingVertex &Tmatch){ 
			std::pair<bool, edgeDescriptor> most_likely=gt::getMostLikely(g, ie);
			if (!most_likely.first){
			}
			else if (g[most_likely.second].probability>prob){
			//if(g[v].nObs>nObs){	
				result.first=true;
				result.second=v;
				prob=g[most_likely.second].probability;

			}
			//break;
		}
	}
	return result;
}

// std::pair <bool, vertexDescriptor> Configurator::exactPolicyMatch(State s, TransitionSystem& g, Direction d){
// 	std::pair <bool, vertexDescriptor> result(false, TransitionSystem::null_vertex());
// 	auto vs= boost::vertices(g);
// 	for (auto vi=vs.first; vi!= vs.second; vi++){
// 		vertexDescriptor v=*vi;
// 		if (matcher.isPerfectMatch(g[v], s) & v!=movingVertex & inEdges(g, v, d).empty()){
// 			result.first=true;
// 			result.second=v;
// 			break;
// 		}

// 	}
// 	return result;
// }

std::pair <bool, vertexDescriptor> Configurator::findExactMatch(vertexDescriptor v, TransitionSystem& g, Direction dir){
	std::pair <bool, vertexDescriptor> result(false, TransitionSystem::null_vertex());
	auto vs= boost::vertices(g);
	//float prob=0;
	int nObs=0;
	for (auto vi=vs.first; vi!= vs.second; vi++){
		if (*vi!=v){
			std::vector <edgeDescriptor> ie=inEdges(g, v, dir);
			bool Tmatch=true;
			//if (dir!=Direction::UNDEFINED){
			Tmatch=!ie.empty()||dir==Direction::UNDEFINED;
			//}
			if (matcher.isPerfectMatch(g[v], g[*vi])&*vi!=movingVertex &Tmatch){
			//std::pair<bool, edgeDescriptor> most_likely=gt::getMostLikely(g, ie);
			//if (!most_likely.first){
			//}
			//else if (g[most_likely.second].probability>prob){
				if(g[v].nObs>nObs){
				result.first=true;
				result.second=*vi;
				//prob=g[most_likely.second].probability;
				nObs=g[v].nObs;
			}
		}
		}
	}
	return result;
}

// std::pair <bool, vertexDescriptor> Configurator::exactPolicyMatch(vertexDescriptor v, TransitionSystem& g, Direction d){
// 	std::pair <bool, vertexDescriptor> result(false, TransitionSystem::null_vertex());
// 	auto vs= boost::vertices(g);
// 	for (auto vi=vs.first; vi!= vs.second; vi++){
// 		if (*vi!=v){
// 			if (matcher.isPerfectMatch(g[*vi], g[v])&*vi!=movingVertex&& !inEdges(g, *vi, d).empty()){
// 			result.first=true;
// 			result.second=*vi;
// 			break;
// 		}
// 		}


// 	}
// 	return result;
// }

void Configurator::changeStart(b2Transform& start, vertexDescriptor v, TransitionSystem& g){
	if (g[v].outcome == simResult::crashed && boost::in_degree(v, g)>0){
		edgeDescriptor e = boost::in_edges(v, g).first.dereference();
		start = g[e.m_source].endPose;
	}
	else{
		start=g[v].endPose;
	}
}


// std::pair<bool, vertexDescriptor> Configurator::findBestMatch(State s){
// 	//std::vector <vertexDescriptor> matches;
// 	//vertexDescriptor v = boost::add_vertex(g);
// 	//g[v].fill(sim);
// 	std::pair<bool, vertexDescriptor> result(0, -1);
// 	float bestDistance=10000;
// 	for (vertexDescriptor stateV: transitionSystem.m_vertices){
// 		DistanceVector dv = matcher.getDistance(transitionSystem[stateV], s);
// 		if (matcher.isPerfectMatch(dv) & matcher.sumVector(dv)<bestDistance){
// 			result.second= stateV;
// 			result.first=true;
// 		}
// 	}
// 	return result;
// }


float Configurator::trackTaskExecution(Task & t){
	float error=0;
	if (planVertices.empty() & planning){
		return error;
	}
	std::unordered_map<State*, float>::iterator it;
	if (it=errorMap.find(transitionSystem[currentVertex].ID); it!=errorMap.end()){
		error=it->second;
		it->second=0;
	}
	if (t.motorStep>0 & fabs(error)<TRACKING_ERROR_TOLERANCE){
		t.motorStep--;
		printf("step =%i\n", t.motorStep);
	}
	else if (fabs(error)>=TRACKING_ERROR_TOLERANCE){
		int correction=-std::floor(error/(t.action.getLinearSpeed()*timeElapsed)+0.5);
		t.motorStep+=correction; //reflex
		// auto eb=boost::edge(currentEdge.m_source,currentEdge.m_target, transitionSystem);
		// if (eb.second){
		// 	transitionSystem[eb.first].step+=correction;
		// }
	}
	if(t.motorStep==0){
		t.change=1;
	}
	updateGraph(transitionSystem, error);
	return error;
}

b2Transform Configurator::assignDeltaPose(Task::Action a, float timeElapsed){
	b2Transform result;
	float theta = a.getOmega()* timeElapsed;
	result.p ={a.getLinearSpeed()*cos(theta),a.getLinearSpeed()*sin(theta)};
	result.q.Set(a.getOmega());
	return result;
}


int Configurator::motorStep(Task::Action a){
	int result=0;
        if (a.getOmega()>0){ //LEFT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
        }
		else if (a.getOmega()<0){ //RIGHT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
		}
		else if (a.getLinearSpeed()>0){
			result = (simulationStep)/(MOTOR_CALLBACK*a.getLinearSpeed());
		}
	    return abs(result);
    }

void Configurator::changeTask(bool b, int &ogStep){
	if (!b){
		return;
	}
	if (planning){
		if (planVertices.empty()){
			currentVertex=movingVertex;
			return;
		}
		if (currentVertex!=movingVertex){
			std::pair<edgeDescriptor, bool> ep=boost::add_edge(currentVertex, planVertices[0], transitionSystem);
			currentVertex= planVertices[0];
		//edgeDescriptor e= ep.first;
			currentEdge=ep.first;
		}
		Direction direction=Direction::DEFAULT;
		if (auto edge= boost::edge(movingVertex, planVertices[0], transitionSystem); edge.second){
			direction= transitionSystem[edge.first].direction;
		}
		transitionSystem[currentEdge].direction=direction;
		// if (!ep.second){
		// 	// throw std::invalid_argument("no plan was formed");
		// 	// return;
		// 	currentEdge= boost::add_edge(currentVertex, pl)
		// }
		boost::clear_vertex(movingVertex, transitionSystem);

		planVertices.erase(planVertices.begin());
		currentTask = Task(transitionSystem[currentEdge.m_source].disturbance, transitionSystem[currentEdge].direction, b2Transform(b2Vec2(0,0), b2Rot(0)), true);
		currentTask.motorStep = transitionSystem[currentEdge].step;
	}
	else{
		if (transitionSystem[0].disturbance.isValid()){
			currentTask = Task(transitionSystem[0].disturbance, DEFAULT); //reactive
		}
		else if(currentTask.direction!=DEFAULT){
				currentTask = Task(transitionSystem[0].disturbance, DEFAULT); //reactive
		}
		else{
			currentTask = Task(controlGoal.disturbance, DEFAULT); //reactive
		}

		currentTask.motorStep = motorStep(currentTask.getAction());
		printf("changed to reactive\n");
	}
	ogStep = currentTask.motorStep;
}

void Configurator::trackDisturbance(b2Transform & pose, Task::Action a, float error){
	float angleTurned =MOTOR_CALLBACK*a.getOmega();
	pose.q.Set(pose.q.GetAngle()-angleTurned);	
	float distanceTraversed = 0;
	float initialL = pose.p.Length();
	if(fabs(error)<TRACKING_ERROR_TOLERANCE){
		distanceTraversed= MOTOR_CALLBACK*a.getLinearSpeed();
	}
	else{
		distanceTraversed=error;
	}
	pose.p.x=cos(pose.q.GetAngle())*initialL-cos(angleTurned)*distanceTraversed;
	pose.p.y = sin(pose.q.GetAngle())*initialL-sin(angleTurned)*distanceTraversed;
}

void Configurator::planPriority(TransitionSystem&g, vertexDescriptor v){
    for (vertexDescriptor p:planVertices){
		if (p==v){
       		g[v].phi-=.1;
			break;
		}
    } 
}

void Configurator::updateGraph(TransitionSystem&g, float error){
	if (debugOn){
		printf("updating graph\n");
	}
	b2Rot rot(getTask()->getAction().getOmega()*MOTOR_CALLBACK);
	b2Transform deltaPose;
	//if (fabs(error)<TRACKING_ERROR_TOLERANCE){
	float xdistance=getTask()->getAction().getLinearVelocity().x*MOTOR_CALLBACK+error;
	deltaPose=b2Transform(b2Vec2(xdistance,
					getTask()->getAction().getLinearVelocity().y*MOTOR_CALLBACK), 
					rot);
	//}
	auto vPair =boost::vertices(g);
	for (auto vIt= vPair.first; vIt!=vPair.second; ++vIt){ //each node is adjusted in explorer, so now we update
		if (*vIt!=movingVertex){
			g[*vIt].endPose-=deltaPose;
			if (g[*vIt].disturbance.isValid()){
				g[*vIt].disturbance.bf.pose-=deltaPose;
			}
		}
	}
	// if (fabs(error)>TRACKING_ERROR_TOLERANCE){
	// 	deltaPose=b2Transform(b2Vec2(cos(rot.GetAngle())*error,
	// 					getTask()->getAction().getLinearVelocity().y*MOTOR_CALLBACK), 
	// 					rot);
	
	// }
	if(controlGoal.disturbance.isValid()){
		controlGoal.disturbance.bf.pose-=deltaPose;
	}
}