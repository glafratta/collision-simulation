#include "configurator.h"
#include <chrono>

void debug::graph_file(int it, TransitionSystem& g, Disturbance goal){
	char fileName[50];
	sprintf(fileName, "/tmp/graph%04i.txt", it);
	FILE * f=fopen(fileName, "w");
	auto vs=boost::vertices(g);
	for (auto vi=vs.first; vi!=vs.second; vi++){
		auto es=boost::out_edges(*vi, g);
		if (goal.getAffIndex()!=NONE){
			b2Vec2 v = goal.getPosition() - g[*vi].endPose.p; //distance between disturbance and robot
			if (v.Length()<DISTANCE_ERROR_TOLERANCE){
				fprintf(f, "*");
			}
		}
		fprintf(f,"%i -> ", *vi);
		for (auto ei=es.first; ei!=es.second; ei++){
			fprintf(f, "%i ", (*ei).m_target);
		}
		fprintf(f, "\t(x=%.3f, y= %.3f, theta= %.3f)\n", g[*vi].endPose.p.x, g[*vi].endPose.p.y, g[*vi].endPose.q.GetAngle());
	}
	fclose(f);
}

void ConfiguratorInterface::setReady(bool b){
	ready = b;
}


bool ConfiguratorInterface::isReady(){
	return ready;
}

void Configurator::dummy_vertex(vertexDescriptor src){
	vertexDescriptor prev_current=currentVertex;
	currentVertex=boost::add_vertex(transitionSystem);
	gt::fill(simResult(), &transitionSystem[currentVertex]);
	transitionSystem[currentVertex].nObs++;
	//transitionSystem[currentVertex].direction=STOP;
	currentTask=Task(Direction::STOP);
//	printf("currentTask l=%f, r=%f\n", currentTask.action.L, currentTask.action.R);
	movingEdge = boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
	currentEdge = boost::add_edge(src, currentVertex, transitionSystem).first;
	printf("dummy, current edge = %i, %i\n", src, currentVertex);
	transitionSystem[movingEdge].direction=STOP;
	transitionSystem[currentEdge].direction=STOP;
	errorMap.emplace((transitionSystem[currentVertex].ID), ExecutionError());

}


bool Configurator::Spawner(){ 
	//PREPARE VECTORS TO RECEIVE DATA
	if (data2fp.empty()){
		printf("data empty!\n");
		return 1;
	}
	//currentBox2D = CoordinateContainer(data2fp);
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
//	taskRotationError(); //to remove
	// b2Transform velocity;
	//  if (currentTask.action.getOmega()==0){
	// 	float dataRange=0.25;
	//  	velocity= sensorProc->affineTransEstimate(std::vector <Pointf>(data.begin(), data.end()), currentTask.action, timeElapsed, dataRange);
	// 	//GetRealVelocity(current, previous); //closed loop, sensor feedback for velocity
	//  }
	// else{
	// 	velocity = b2Transform(currentTask.getAction().getTransform()); //open loop
	// }
	// currentTask.action.setRecSpeed(SignedVectorLength(velocity.p));
	// currentTask.action.setRecOmega(velocity.q.GetAngle());
	//float rotation_Error=taskRotationError();
	//IF WE  ALREADY ARE IN AN OBSTACLE-AVOIDING STATE, ROUGHLY ESTIMATE WHERE THE OBSTACLE IS NOW
	//bool isObstacleStillThrere = worldBuilder.buildWorld(world, currentBox2D, currentTask.start, currentTask.direction, &currentTask).first;
	
	
	// if (controlGoal.change){
	// 	Disturbance loopD(PURSUE, -(ogGoal.p));
	// 	controlGoal=Task(loopD,DEFAULT);
	// }


	//CHECK IF WITH THE CURRENT currentTask THE ROBOT WILL CRASH
	// simResult result;
	//Direction dir;

	auto startTime =std::chrono::high_resolution_clock::now();
	if (planning){ //|| !planError.m_vertices.empty())
		auto checkedge = boost::edge(movingVertex, currentVertex, transitionSystem);
		transitionSystem[movingVertex].disturbance=transitionSystem[currentVertex].disturbance;
		transitionSystem[movingVertex].disturbance.invalidate();
		if (debugOn){
			printf("moving edge= %i -> %i\n", movingEdge.m_source, movingEdge.m_target);
		}
		std::pair<bool, vertexDescriptor> been= been_there(transitionSystem, controlGoal.disturbance);
		//printf("checked been = %i\n", been.first);
		std::vector <std::pair <vertexDescriptor, vertexDescriptor>> toRemove;
		vertexDescriptor src;
		if (!planVertices.empty()){
			src=movingVertex;
		}
		else{
			src=currentVertex;
		}
		std::vector <vertexDescriptor> plan_provisional=planVertices;
		if (been.first){
		//	printf("provisional plan\n");
			plan_provisional=planner(transitionSystem, src, been.second, been.first);
		}
		//printf("plan provisional size = %i\n", plan_provisional.size());
		bool plan_works=checkPlan(world, plan_provisional, transitionSystem);
		//printf("plan provisional size = %i, plan_works=%i", plan_provisional.size(), plan_works);
		if (!plan_provisional.empty() || plan_works){	//		
			planVertices=plan_provisional;
			//printf("going with old plan\n");
		}
		//if plan fails or not there, 
		else{
			is_not_v not_cv(currentVertex);
			planVertices.clear();
			boost::clear_vertex(movingVertex, transitionSystem);
			dummy_vertex(currentVertex);//currentEdge.m_source
			currentTask.change=1;
			// if (!planVertices.empty()){
			// 	src=movingVertex;
			// }
			// else{
			// 	src=currentVertex;
			// }
			src=currentVertex;
			resetPhi(transitionSystem);
			toRemove=explorer(src, transitionSystem, currentTask, world);
			clearFromMap(toRemove, transitionSystem, errorMap);
			Connected connected(&transitionSystem);
			FilteredTS fts(transitionSystem, boost::keep_all(), connected);
			TransitionSystem tmp;
			boost::copy_graph(fts, tmp);
			transitionSystem.clear();
			transitionSystem.swap(tmp);
			debug::graph_file(iteration, transitionSystem, controlGoal.disturbance);
			planVertices= planner(transitionSystem, src);
			boost::remove_out_edge_if(movingVertex, not_cv, transitionSystem);
		//	printf("after remoing out edges from 0->current=%i exists=%i\n", currentVertex, currentEdge !=edgeDescriptor());
			//boost::print_graph(transitionSystem);
		}
		// if (debugOn){
		// 	printPlan();
		// 	printf("graph size= %i\n", transitionSystem.m_vertices.size());
		// }

	}
	else if (!planning){
		simResult result = simulate(transitionSystem[currentVertex],transitionSystem[currentVertex],currentTask, world, simulationStep);
		currentTask.change = transitionSystem[currentVertex].outcome==simResult::crashed;
	}
	float duration=0;
	auto endTime =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>d= startTime- endTime; //in seconds
 	duration=abs(float(d.count())/1000); //express in seconds
	printf("took %f seconds\n", duration);
	if (benchmark){
		FILE * f = fopen(statFile, "a+");
		fprintf(f,"%i\t%i\t%f\n", worldBuilder.getBodies(), transitionSystem.m_vertices.size(), duration);
		fclose(f);
	}
	worldBuilder.resetBodies();
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
		if(t.direction == Direction::DEFAULT & 
		fabs(src.endPose.q.GetAngle())<fabs(controlGoal.disturbance.pose().q.GetAngle())+M_PI/6){
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
	// std::vector <Pointf> vec= set2vec(ci->data);
	//std::vector <Pointf> nb=pcProc.setDisturbanceOrientation(result.collision, ci->data); //pcProc.neighbours(result.collision.getPosition(), pcProc.NEIGHBOURHOOD, vec);
	// pcProc.findOrientation(nb);
	
	// cv::Rect2f rect =worldBuilder.getRect(nb);
	// result.collision.setAsBox(rect.width/2, rect.height/2);
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


// std::vector <std::pair<vertexDescriptor, vertexDescriptor>>Configurator::explorer_old(vertexDescriptor v, TransitionSystem& g, Task t, b2World & w){
// 	vertexDescriptor v1, v0, bestNext=v;
// 	//Direction direction= t.direction;
// 	Direction direction=g[v].direction;
// 	std::vector <std::pair<vertexDescriptor, float>> priorityQueue = {std::pair(bestNext,0)};
// 	b2Transform start= b2Transform(b2Vec2(0,0), b2Rot(0));
// 	std::vector<std::pair<vertexDescriptor, vertexDescriptor>> toRemove;
// 	printf("exploring\n");
// 	do{
// 		v=bestNext;
// 		priorityQueue.erase(priorityQueue.begin());
// 		EndedResult er = controlGoal.checkEnded(g[v]);
// 		if (er.ended){
// 			printf("ended %i\n", v);
// 		}
// 		applyTransitionMatrix(g, v, direction, er.ended, v);
// 		for (Direction d: g[v].options){ //add and evaluate all vertices
// 			v0=v; //node being expanded
// 			v1 =v0; //frontier
// 			do {
// 			std::pair <State, Edge> sk(State(g[v0].options[0]), Edge());
// 			bool topDown=1;
// 			changeStart(start, v0, g);
// 			t = Task(getDisturbance(g, v0), g[v0].options[0], start, topDown);
// 			float _simulationStep=simulationStep;
// 			adjustStepDistance(v0, g, &t, _simulationStep);
// 			//Disturbance expectedD=gt::getExpectedDisturbance(g, v0, t.direction, iteration);
// 			worldBuilder.buildWorld(w, currentBox2D, t.start, t.direction); //was g[v].endPose
// 			setStateLabel(sk.first, v0, t.direction); //new
// 			simResult sim=simulate(sk.first, g[v0], t, w, _simulationStep);
// 			gt::fill(sim, &sk.first, &sk.second); //find simulation result
// 			sk.first.direction=t.direction;
// 			er  = estimateCost(sk.first, g[v0].endPose);
// 			State * source=NULL;
// 			bool vm= matcher.isPerfectMatch(g[v], g[currentEdge.m_source]); //see if we are at the beginning of the exploration:
// 																			//v=0 and currentEdge =src will match so we try to prevent
// 																			//changing the movign vertex which is by default the origin
// 			if (v0==movingVertex & vm){
// 				source= g[currentEdge.m_source].ID;
// 			}
// 			else{
// 				source=g[v0].ID;
// 			}
// 			std::pair<bool, vertexDescriptor> match=findExactMatch(sk.first, g, source, t.direction);			
// 			std::pair <edgeDescriptor, bool> edge(edgeDescriptor(), false);
// 			if (!match.first){
// 				edge= addVertex(v0, v1,g, Disturbance(),sk.second);
// 				g[edge.first.m_target].label=sk.first.label; //new edge, valid
// 			}
// 			else{
// 				g[v0].options.erase(g[v0].options.begin());
// 				v1=match.second; //frontier
// 				if (!(v0==v1)){
// 					edge.first= boost::add_edge(v0, v1, g).first; //assumes edge added
// 					edge.second=true; //just means that the edge is valid
// 					g[edge.first]=sk.second;//t.direction;
// 				}
// 			}
// 			if(edge.second){
// 				gt::set(edge.first, sk, g, v1==currentVertex, errorMap, iteration);
// 				gt::adjustProbability(g, edge.first);
// 			}
// 			applyTransitionMatrix(g, v1, t.direction, er.ended, v);
// 			g[v1].phi=evaluationFunction(er);
// 			std::vector<std::pair<vertexDescriptor, vertexDescriptor>> toPrune =(propagateD(v1, v0, g)); //og v1 v0
// 			v0=v1;
// 			pruneEdges(toPrune,g, v, priorityQueue, toRemove);
// 			}while(t.direction !=DEFAULT & g[v0].options.size()!=0);
// 			addToPriorityQueue(v1, priorityQueue, g[v1].phi);
// 		}
// 		bestNext=priorityQueue[0].first;
// 		direction = g[boost::in_edges(bestNext, g).first.dereference().m_target].direction;
// 	}while(g[bestNext].options.size()>0);
// 	return toRemove;
// }

std::vector <std::pair<vertexDescriptor, vertexDescriptor>>Configurator::explorer(vertexDescriptor v, TransitionSystem& g, Task t, b2World & w){
	if (controlGoal.disturbance.isValid()){
		b2Vec2 v = controlGoal.disturbance.getPosition() - b2Vec2(0,0);
		printf("goal start: %f, %f, %f, distance = %f, valid =%i\n", controlGoal.start.p.x,controlGoal.start.p.y, controlGoal.start.q.GetAngle(), v.Length(), controlGoal.disturbance.isValid());
		printf("goal position= %f, %f, %f, valid =%i\n", controlGoal.disturbance.pose().p.x, controlGoal.disturbance.pose().p.y, controlGoal.disturbance.pose().q.GetAngle(), controlGoal.disturbance.isValid());
}
	vertexDescriptor v1=v, v0=v, bestNext=v, v0_exp=v;
	//Direction direction= t.direction;
	Direction direction=g[currentEdge].direction;
	std::vector <vertexDescriptor> priorityQueue = {bestNext};
	b2Transform start= b2Transform(b2Vec2(0,0), b2Rot(0));
	std::vector<std::pair<vertexDescriptor, vertexDescriptor>> toRemove;
	printf("EXPLORING\n");
	do{
		v=bestNext;
		priorityQueue.erase(priorityQueue.begin());
		EndedResult er = controlGoal.checkEnded(g[v], t.direction);
		if (er.ended){
		//	printf("ended %i\n", v);
		}
		applyTransitionMatrix(g, v, direction, er.ended, v);
		for (Direction d: g[v].options){ //add and evaluate all vertices
			v0_exp=v;
			std::vector <Direction> options=g[v0_exp].options;
			while (!options.empty()){
				options.erase(options.begin());
				v0=v0_exp; //node being expanded
				v1 =v0; //frontier
			do {
			std::pair <State, Edge> sk(State(), Edge(g[v0].options[0]));
			bool topDown=1;
			changeStart(start, v0, g);
				t = Task(getDisturbance(g, v0), g[v0].options[0], start, topDown);
				float _simulationStep=simulationStep;
				adjustStepDistance(v0, g, &t, _simulationStep);
				worldBuilder.buildWorld(w, data2fp, t.start, t.direction); //was g[v].endPose
			//	setStateLabel(sk.first, v0, t.direction); //new
				simResult sim=simulate(sk.first, g[v0], t, w, _simulationStep);
				gt::fill(sim, &sk.first, &sk.second); //find simulation result
				sk.second.direction=t.direction;
				er  = estimateCost(sk.first, g[v0].endPose, sk.second.direction);
				State * source=NULL;
				bool vm= matcher.isPerfectMatch(g[v], g[currentEdge.m_source]); //see if we are at the beginning of the exploration:
																				//v=0 and currentEdge =src will match so we try to prevent
																				//changing the movign vertex which is by default the origin
				if (v0==movingVertex & vm){
					source= g[currentEdge.m_source].ID;
				}
				else{
					source=g[v0].ID;
				}
				std::pair<bool, vertexDescriptor> match=findExactMatch(sk.first, g, source, t.direction);			
				std::pair <edgeDescriptor, bool> edge(edgeDescriptor(), false);
				if (!match.first){
					edge= addVertex(v0, v1,g, Disturbance(),sk.second);
					g[edge.first.m_target].label=sk.first.label; //new edge, valid
				}
				else{
					g[v0].options.erase(g[v0].options.begin());
					v1=match.second; //frontier
					if (!(v0==v1)){
						edge.first= boost::add_edge(v0, v1, g).first; //assumes edge added
						edge.second=true; //just means that the edge is valid
						g[edge.first]=sk.second;//t.direction;
					}
				}
				if(edge.second){
					gt::set(edge.first, sk, g, v1==currentVertex, errorMap, iteration);
					gt::adjustProbability(g, edge.first);
				}
				applyTransitionMatrix(g, v1, t.direction, er.ended, v0);
				g[v1].phi=evaluationFunction(er);
				std::vector<std::pair<vertexDescriptor, vertexDescriptor>> toPrune =(propagateD(v1, v0, g)); //og v1 v0
				v0_exp=v0;
				options=g[v0_exp].options;
				v0=v1;			
				pruneEdges(toPrune,g, v, v0_exp, priorityQueue, toRemove);
			
			}while(t.direction !=DEFAULT & int(g[v0].options.size())!=0);
			addToPriorityQueue(v1, priorityQueue, g);
			}
		}
		bestNext=priorityQueue[0];
		// if (controlGoal.getAffIndex()==PURSUE){
		// 	//printf("best=%i", bestNext);
		// }
		direction = g[boost::in_edges(bestNext, g).first.dereference()].direction;
	}while(g[bestNext].options.size()>0);
	return toRemove;
}



std::vector<std::pair<vertexDescriptor, vertexDescriptor>> Configurator::propagateD(vertexDescriptor v1, vertexDescriptor v0,TransitionSystem&g){
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
			EndedResult er=estimateCost(g[ep.first.m_target], g[ep.first.m_source].endPose,g[ep.first].direction); //reassign cost
			g[ep.first.m_target].phi =evaluationFunction(er);	
			std::pair <bool, vertexDescriptor> match= findExactMatch(ep.first.m_target, g, g[ep.first].direction);
			if ( match.first){
				std::pair<vertexDescriptor, vertexDescriptor>pair(ep.first.m_target, match.second);
				deletion.push_back(pair);			//first is eliminated, the second is its match
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

void Configurator::pruneEdges(std::vector<std::pair<vertexDescriptor, vertexDescriptor>> vertices, TransitionSystem& g, vertexDescriptor& src, vertexDescriptor& active_src, std::vector <vertexDescriptor>& pq, std::vector<std::pair<vertexDescriptor, vertexDescriptor>>&toRemove){ //clears edges out of redundant vertices, removes the vertices from PQ, returns vertices to remove at the end
	for (std::pair<vertexDescriptor, vertexDescriptor> pair:vertices){
		if (pair.first==src){
			src=pair.second;
		}
		if (pair.first==active_src){
			active_src=pair.second;
		}
		std::vector<edgeDescriptor> ie =gt::inEdges(g, pair.second, DEFAULT); //first vertex that satisfies that edge requirement
		std::vector <edgeDescriptor> toReassign=gt::inEdges(g, pair.first, DEFAULT);
		edgeDescriptor e =edgeDescriptor(), r_visited=edgeDescriptor();
		edgeDescriptor e2 = gt::visitedEdge(toReassign, g);
		if (ie.empty()){
		}
		else{
			e=ie[0];
			toReassign.push_back(e); 
		}
		gt::update(e, std::pair <State, Edge>(g[pair.first], g[e2]),g, pair.second==currentVertex, errorMap, iteration);
		float match_distance=10000;
		for (edgeDescriptor r:toReassign){ //reassigning edges
			auto new_edge= gt::add_edge(r.m_source, pair.second, g, iteration);
			// if (g[r.m_source].visited()){
			// 	EndedResult er=estimateCost(g[pair.second], g[r.m_source].endPose); //reassign cost
			// 	g[pair.second].phi =evaluationFunction(er);	
			// }
		}
		boost::clear_vertex(pair.first, g);
		toRemove.push_back(pair);
		for (int i=0; i<pq.size(); i++){ //REMOVE FROM PQ
			if(pq[i]==pair.first){
				pq.erase(pq.begin()+i);
			}
		}
		gt::adjustProbability(g, e);
	}
}

void Configurator::clearFromMap(std::vector<std::pair<vertexDescriptor, vertexDescriptor>> matches , TransitionSystem&g, std::unordered_map<State*, ExecutionError>map){
	//auto es=boost::edges(g);
	//for (auto ei=es.first; ei!=es.second; ei++){
		for (std::pair<vertexDescriptor, vertexDescriptor> pair:matches){
			if (auto it=map.find(g[pair.first].ID); it!=map.end()){
				ExecutionError exer = map.at(transitionSystem[pair.first].ID);
				map.insert_or_assign(transitionSystem[pair.second].ID, exer);
				map.erase(it);
				break;
			}

		}
	//}
}

std::vector <vertexDescriptor> Configurator::planner(TransitionSystem& g, vertexDescriptor src, vertexDescriptor goal, bool been){
	std::vector <vertexDescriptor> plan;
	std::vector<std::vector<vertexDescriptor>> paths;
	paths.push_back(std::vector<vertexDescriptor>()={src});
	std::vector <Frontier> frontier_v;
	bool run=true;
	std::vector <Frontier> priorityQueue;
	if (currentVertex==movingVertex){
		//printf("current %i =moving%i! return\n", currentVertex, movingVertex);
		return plan;
	}
	std::vector <vertexDescriptor> add;
	std::vector<std::vector<vertexDescriptor>>::reverse_iterator path= paths.rbegin();
	vertexDescriptor path_end=src;
	do{
		//find frontier (STRAIGHT)
		//printf("start\n");
		frontier_v=frontierVertices(src, g, DEFAULT, been);
		if (src==currentVertex){
		//	printf("planning from src =%i, out vertices n%i\n", src, frontier_v.size());
		}
		for (Frontier f: frontier_v){ //add to priority queue
			planPriority(g, f.first);
			addToPriorityQueue(f, priorityQueue, g);
		}
		if (priorityQueue.empty()){
		//	printf("emtpy pq\n");
			break;
		}
		src=priorityQueue.begin()->first;
		add=std::vector <vertexDescriptor>(priorityQueue.begin()->second.begin(), priorityQueue.begin()->second.end());
		add.push_back(src);
		std::pair<edgeDescriptor, bool> edge(edgeDescriptor(), false);
		std::vector<vertexDescriptor>::reverse_iterator pend=(path->rbegin());
		while (!edge.second ){//|| ((*(pend.base()-1)!=goal &goal!=TransitionSystem::null_vertex())&!controlGoal.checkEnded(g[*(pend.base()-1)]).ended)
		//	printf("possible paths:%i\n", paths.size());
			vertexDescriptor end=*(pend.base()-1);
			edge= boost::edge(end,add[0], g);
			if (!add.empty()&!edge.second & path!=paths.rend()){ //if this path does not have an edge and there are 
													//other possible paths, go to previous paths
				if (pend.base()-1!=(path->begin())){
					pend++;
				}
				else{
					path++;
					pend=(*path).rbegin();
				}
			}
			else if (edge.second & pend.base()!=path->rbegin().base()){
				paths.emplace_back(std::vector <vertexDescriptor>(path->begin(), pend.base()));
				path=paths.rbegin();
				break;
			}
			else{
				break;
			}
			if ( path==paths.rend()) { //if there are no other paths
				paths.push_back(std::vector<vertexDescriptor>()); //make a new one
				path=paths.rbegin();
				break;
			}	
		}
		priorityQueue.erase(priorityQueue.begin());
		for (vertexDescriptor c:add){
			g[c].label=VERTEX_LABEL::UNLABELED;
			path->push_back(c);	
			path_end=c;			
		}
		// printf("planning, path size= %i\n",path->size() );
		// printf("pq empty=%i, path end=%i, ended=%i\n", priorityQueue.empty(), path_end, controlGoal.checkEnded(g[path_end].endPose).ended);
		// printf("conf running=%i\n", running);
		
	}while(!priorityQueue.empty() & (path_end!=goal &!controlGoal.checkEnded(g[path_end].endPose, UNDEFINED, true).ended));
	//printf("exited while\n");
	auto vs=boost::vertices(g);
	float final_phi=10000;
	for (std::vector<vertexDescriptor> p: paths){
		vertexDescriptor end_plan= *(p.rbegin().base()-1);
		if (end_plan==goal){
			plan=std::vector(p.begin()+1, p.end());
			break;
		}
		else if (g[end_plan].phi<final_phi){
			plan=std::vector(p.begin()+1, p.end());
			final_phi=g[end_plan].phi;
		}
	}
	// for (auto vi=vs.first; vi!=vs.second; vi++){
	// 	if (g[*vi].label!=UNLABELED){
	// 		boost::clear_vertex(*vi, g);
	// 	}
	// }
	printf("PLANNED!\n");
	printPlan();
	return plan;
}



// std::vector <vertexDescriptor> Configurator::planner_old(TransitionSystem& g, vertexDescriptor src, vertexDescriptor match, bool been){
// 	std::vector <vertexDescriptor> plan;
// 	vertexDescriptor connecting; //og: src=currentV
// 	std::vector <edgeDescriptor> frontier;
// 	bool run=true;
// 	if (currentVertex==movingVertex){
// 		return plan;
// 	}
// 	//std::vector <std::pair<vertexDescriptor, float>> priorityQueue = {std::pair(src,0)};
// 	do{
// 		//priorityQueue.erase(priorityQueue.begin());
// 		frontier=frontierVertices(src, g, DEFAULT, been);
// 		connecting=TransitionSystem::null_vertex();
// 		float phi=2, src_prob=0; //very large phi, will get overwritten
// 		bool changed_src=false;
// 		int out_deg=0;
// 		for (edgeDescriptor e:frontier){
// 			planPriority(g, e.m_target);
// 			//addToPriorityQueue(e.m_target, priorityQueue, g[e.m_target].phi);
// 			if (g[e.m_target].phi<phi || boost::out_degree(e.m_target, g)>out_deg){ //& (g[e.m_target].direction==g[src].direction & g[e].weighted_probability(iteration)>=src_prob)){
// 					phi=g[e.m_target].phi;
// 					if (e.m_source!=src){
// 						connecting=e.m_source;
// 					}
// 					src=e.m_target;
// 					changed_src=true;
// 			}
// 			else if (g[e.m_source].label!=UNLABELED){
// 			 	boost::clear_vertex(e.m_source, g);
// 			}
// 			out_deg=boost::out_degree(e.m_target, g);
// 		}
// 		if (connecting!=TransitionSystem::null_vertex()){
// 			g[connecting].label=VERTEX_LABEL::UNLABELED;
// 			plan.push_back(connecting);
// 		}
// 		//if (!frontier.empty()){
// 		if (changed_src){
// 			if (src!=currentVertex){
// 				plan.push_back(src);
// 				g[src].label=UNLABELED;
// 			}
// 		}
// 		else{
// 			run=false;
// 			break;
// 		}
// 	}while(run);
// 	return plan;
// }

bool Configurator::checkPlan(b2World& world, std::vector <vertexDescriptor> &p, TransitionSystem &g, b2Transform start){
	bool result=true;
	int it=-1;//this represents currentv
	auto ep=boost::edge(movingVertex, currentVertex, g);	
	//printf("0->current=%i exists=%i\n", currentVertex, ep.second);
	printPlan();
	std::vector <vertexDescriptor> vpt=p;
	printPlan(&vpt);
	if (p.empty() && currentTask.motorStep==0){
	//	printf("plan empty=%i, motor step=%i\n", p.empty(), currentTask.motorStep);
		return false;
	}
	if (p.size()>0){
		if (p[0]==currentVertex){
			p.erase(p.begin());
		}		
	}	
	do {
		Task t= Task(g[ep.first.m_source].disturbance, g[ep.first].direction, start, true);
		float stepDistance=BOX2DRANGE;
		worldBuilder.buildWorld(world, data2fp, start, t.direction, t.disturbance);
		std::pair <State, Edge> sk(State(), Edge(t.direction));
		//sk.first.direction=t.direction;
	//	printf("skipping from %i, edge %i ->%i", it, ep.first.m_source, ep.first.m_target);
		b2Transform endPose=skip(ep.first,g,it, &t, stepDistance, p);
	//	printf("to it %i, edge %i ->%i\n", it, ep.first.m_source, ep.first.m_target);
		simResult sr=t.willCollide(world, iteration, debugOn, SIM_DURATION, stepDistance);
		gt::fill(sr, &sk.first, &sk.second); //this also takes an edge, but it'd set the step to the whole
									// simulation result step, so this needs to be adjusted
		if (sk.first.endPose.p.Length()>endPose.p.Length()){
			sk.first.endPose=endPose;
			sk.first.outcome=simResult::successful;
		}
		start = sk.first.endPose;
		// DistanceVector distance = matcher.getDistance(g[planVertices[it]], s);
		// if (!matcher.isPerfectMatch(distance)){
		bool ismatch=matcher.isPerfectMatch(g[ep.first.m_source], sk.first);
	//	printf("sk dist = %i, exp vertex dist=%i\n", sk.first.disturbance.getAffIndex(), g[ep.first.m_source].disturbance.getAffIndex());			
		vertexDescriptor v1=ep.first.m_source;
		std::pair <bool,edgeDescriptor> prev_edge= gt::getMostLikely(g, gt::inEdges(g, v1, t.direction),iteration);
		//vertexDescriptor v0;
		if (!prev_edge.first){
			//v0=prev_edge.second.m_source;
			auto _moving=boost::add_edge(movingVertex, v1, g);
			prev_edge.second=_moving.first;
			if (_moving.second){
			//	printf("new edge added moving\n");
			}
			// prev_edge.second.m_source=movingVertex;
			// prev_edge.second.m_target=v1;
		}
		if (!ismatch){
			printf("state end: x=%f, y=%f, theta=%f, d valid=%i\n", sk.first.endPose.p.x, sk.first.endPose.p.y, sk.first.endPose.q.GetAngle(), sk.first.disturbance.isValid());
			printf("NO MATCH with %i: x=%f, y=%f, theta=%f, d valid=%i\n", v1, g[v1].endPose.p.x, g[v1].endPose.p.y, g[v1].endPose.q.GetAngle(), g[v1].endPose.q.GetAngle());
			//printf("simulation duration step=%i, started from %f, %f, %f\n", sk.second.step, g[prev_edge.second.m_source].endPose.p.x, g[prev_edge.second.m_source].endPose.p.y,g[prev_edge.second.m_source].endPose.q.GetAngle());
			std::pair<bool, vertexDescriptor> match = findExactMatch(sk.first, g, g[prev_edge.second.m_source].ID, sk.second.direction);
			g[prev_edge.second.m_source].options.push_back(t.direction);
			if (!match.first){
				ep =addVertex(prev_edge.second.m_source, v1,g, Disturbance(), g[ep.first], 1);
				printf("added edge %i -> %i in check plan, dir %i\n",prev_edge.second.m_source, v1, g[ep.first].direction);
				// printf("dist valid = %i, end %f, %f, %f\n", g[v1].disturbance.isValid(), g[v1].endPose.p.x, g[v1].endPose.p.y, g[v1].endPose.q.GetAngle());			
			}
			else{
			//	printf("instead matched with %i:  x=%f, y=%f, theta=%f\n", match.second, g[match.second].endPose.p.x, g[match.second].endPose.p.y, g[match.second].endPose.q.GetAngle());
				gt::add_edge(prev_edge.second.m_source, match.second, g, iteration);
				p[it]=match.second;
				if (ep.first.m_target!=TransitionSystem::null_vertex()){
					ep=boost::edge(match.second, ep.first.m_target, g);
					if (!ep.second){
						ep=gt::add_edge(prev_edge.second.m_source, match.second, g, iteration);
					}
				}
				else{
				//	printf("target=%i\n", ep.first.m_target);
					ep.first.m_source=match.second;
				}
			}
			gt::set(ep.first, sk, g, it==currentVertex, errorMap, iteration);
			if (sk.first.outcome==simResult::crashed){
				p.clear();
				printf("plan crashes\n");
				result=false;
			}
		}
		else{
			//printf("updatign existing\n");
			gt::update(prev_edge.second, sk, g,true, errorMap, iteration);
		}
		propagateD(v1,prev_edge.second.m_source, g);
	//	printf("propagated\n");
		gt::adjustProbability(g, ep.first);
	//	printf("adjust prov, it %i,is it target null=%i p size =%i, result=%i\n", it,ep.first.m_target!=TransitionSystem::null_vertex(), p.size(), result);
		// t= Task(g[ep.first.m_source].disturbance, g[ep.first.m_target].direction, start, true);
	}while (ep.first.m_target!=TransitionSystem::null_vertex() & it <int(p.size()-1) & result );
//	printf("checked\n");
	return result;
}


b2Transform Configurator::skip(edgeDescriptor& e, TransitionSystem &g, int& i, Task* t, float& step, std::vector <vertexDescriptor> plan){ 
	b2Transform result;
	if (g[e.m_target].disturbance.isValid()){
		step=b2Vec2(g[e.m_target].endPose.p-g[e.m_target].disturbance.pose().p).Length();
	}
	else{
		adjustStepDistance(e.m_source,g, t, step);
	}
	do{
		i++;
		//printf("iterator = %i, e src= %i, e trgt= %i\n", i, e.m_source, e.m_target);
			auto es = boost::out_edges(e.m_target, g);
			if (es.first==es.second){
				vertexDescriptor new_src=e.m_target;
				e.m_source=new_src;
				e.m_target=TransitionSystem::null_vertex();
			}
			for (auto ei = es.first; ei!=es.second; ++ei){ //was ++ei
				if (plan.empty()){
					break;
				}
				if ((*ei).m_target == plan[i]){
					e= (*ei);
					break;
				}
			}
		result=g[e.m_source].endPose;


		}while (g[e].direction==t->direction & i<planVertices.size()& g[e].direction==DEFAULT);
//	printf("ended skip, result = %f, %f, %f\n", result.p.x, result.p.y, result.q.GetAngle());
	return result;
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





EndedResult Configurator::estimateCost(State &state, b2Transform start, Direction d){
	EndedResult er = controlGoal.checkEnded(state);
	Task t(state.disturbance, d, start);
	er.cost += t.checkEnded(state.endPose).estimatedCost;
	//controlGoal.disturbance.validate();
	return er;
}


float Configurator::evaluationFunction(EndedResult er){ 
	return (abs(er.estimatedCost)+abs(er.cost))/2; //normalised to 1
}



void Configurator::printPlan(std::vector <vertexDescriptor>* p){
	if (p==NULL){
		printf("currently executed plan:\t");
		p=&planVertices;
	}
	else{
		printf("provisional plan:\t");
	}
	std::vector <vertexDescriptor> plan= *p;
	vertexDescriptor pre=currentVertex;
	printf("current=%i\t", pre);
	for (vertexDescriptor v: plan){
		std::pair <edgeDescriptor, bool> edge=boost::edge(pre, v, transitionSystem);
		//auto a=dirmap.find(transitionSystem[edge.first].direction);
		if (!edge.second){
			//throw std::exception();
			//auto a=dirmap.find(transitionSystem[edge.first].direction);
			printf("no edge: %i-> %i", edge.first.m_source, edge.first.m_target);
		}
		else{
			auto a=dirmap.find(transitionSystem[edge.first].direction);
			printf("%i, %s, ", edge.first.m_target, (*a).second);
		}
		pre=edge.first.m_target;
		}
	printf("\n");
}

void Configurator::applyAffineTrans(const b2Transform& deltaPose, b2Transform& pose){
	pose.q.Set(pose.q.GetAngle()-deltaPose.q.GetAngle());
	float og_x= pose.p.x, og_y=pose.p.y;
	pose.p.x= og_x* cos(deltaPose.q.GetAngle())+ og_y*sin(deltaPose.q.GetAngle());
	pose.p.y= og_y* cos(deltaPose.q.GetAngle())- og_x*sin(deltaPose.q.GetAngle());
	pose.p.x-=deltaPose.p.x;
	pose.p.y-=deltaPose.p.y;
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
	//ci->pcProc=&pcProc;
	//ci->ts = TaskSummary(controlGoal.disturbance, controlGoal.direction, motorStep(controlGoal.action));
}

void Configurator::run(Configurator * c){
	//printf("run\n");
	while (c->running){
		if (c->ci->stop){
			c->ci=NULL;
		}
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
			c->data2fp= CoordinateContainer(c->ci->data2fp);
			c->Spawner();
			//c->pcProc.previous=set2vec(c->ci->data);
			//c->ci->ts = TaskSummary(c->currentTask.disturbance, c->currentTask.direction, c->currentTask.motorStep);
		}
	}

}


void Configurator::transitionMatrix(State& state, Direction d, vertexDescriptor src){
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
			if (src==currentVertex & controlGoal.getAffIndex()==PURSUE){
				state.options.push_back(d);
			}
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

void Configurator::applyTransitionMatrix(TransitionSystem&g, vertexDescriptor v0, Direction d, bool ended, vertexDescriptor src){
	if (!g[v0].options.empty()){
		return;
	}
	if (controlGoal.endCriteria.hasEnd()){
		if (ended){
			printf("no options added because %i ended\n", v0);
			return;
		}
	}
	else if(round(g[v0].endPose.p.Length()*100)/100>=BOX2DRANGE){ // OR g[vd].totDs>4
		if (controlGoal.getAffIndex()==PURSUE){
			printf("no options added because %i out of bounds\n", v0);
		}
		return;
	}
	transitionMatrix(g[v0], d, src);
}




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

void Configurator::addToPriorityQueue(vertexDescriptor v, std::vector<vertexDescriptor>& queue, TransitionSystem &g){
	for (auto i =queue.begin(); i!=queue.end(); i++){
		if (g[v].phi <abs(g[*i].phi)){
			queue.insert(i, v);
			return;
		}
	}
	queue.push_back(v);
}

void Configurator::addToPriorityQueue(Frontier f, std::vector<Frontier>& queue, TransitionSystem &g, vertexDescriptor goal){
	for (auto i =queue.begin(); i!=queue.end(); i++){
		if (g[f.first].phi <abs(g[(*i).first].phi)){
			queue.insert(i, f);
			return;
		}
	}
	queue.push_back(f);
}

std::pair <bool, vertexDescriptor> Configurator::been_there(TransitionSystem & g, Disturbance target){
	printf("ENTERED BEEN\n");
	std::pair <bool, vertexDescriptor> result(0, TransitionSystem::null_vertex());
	vertexDescriptor ve=TransitionSystem::null_vertex();
	if (!planVertices.empty()){
		ve= *(planVertices.rbegin().base()-1);
	}
	else {
		ve=currentVertex;
	}
	if (bool fin=controlGoal.checkEnded(g[ve], UNDEFINED, true).ended; (target.getAffIndex()!=PURSUE || fin) & currentVertex){
		printf("is target=%i, task ended = %i\n", target.getAffIndex()==PURSUE, fin);
		return result;
	}
	std::pair <float, vertexDescriptor> best(10000, TransitionSystem::null_vertex());
	auto vs=boost::vertices(g);
	for (auto vi=vs.first; vi!=vs.second; vi++ ){
		//b2Transform difference= g[*vi].endPose-target.disturbance.pose();
		DistanceVector difference={g[*vi].endPose.p.x-target.pose().p.x,
									g[*vi].endPose.p.y-target.pose().p.y,
									g[*vi].endPose.q.GetAngle()-target.pose().q.GetAngle()
									};
		float sum=matcher.sumVector(difference);
		if (abs(difference[0])<matcher.error.dPosition
			& abs(difference[1])<matcher.error.dPosition
			& abs(difference[2])<matcher.error.angle & sum<best.first){
				best.first=sum;
				best.second=*vi;
				result.first=true;
			}
	}
	result.second=best.second;
	printf("been=%i, vertex=%i\n", result.first, result.second);
	return result; //if been there, do not explore, extract a plan then check it
}



// std::vector <Pointf> Configurator::neighbours(b2Vec2 pos, float radius){ //more accurate orientation
// 	std::vector <Pointf> result;
// 	cv::Rect2f rect(pos.x-radius, pos.y-radius, radius*2, radius*2);//tl, br, w, h
// 	auto br=rect.br();
// 	auto tl=rect.tl();
// 	for (Pointf p: sensorTools.previous){
// 		if (p.inside(rect) & p!=getPointf(pos)){
// 			result.push_back(p);
// 		}
// 	}
// 	return result;
// }


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



void Configurator::adjustStepDistance(vertexDescriptor v, TransitionSystem &g, Task * t, float& step){
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, g);
	if(!ep.second){
		return;
	}
	auto eb=boost::edge(currentEdge.m_source,currentEdge.m_target, transitionSystem);
	int stepsTraversed= g[eb.first].step-currentTask.motorStep; //eb.first
	float theta_exp=stepsTraversed*MOTOR_CALLBACK*currentTask.action.getOmega();
	float theta_obs=theta_exp;//currentTask.correct.getError()-theta_exp;
	if (currentTask.getAction().getOmega()!=0){
		float remainingAngle = currentTask.endCriteria.angle.get()-abs(theta_obs);
	//	printf("step =%i/%i, remaining angle=%f\n", currentTask.motorStep, transitionSystem[currentEdge].step,remainingAngle);
		if (t->direction==getOppositeDirection(currentTask.direction).second){
			remainingAngle=M_PI-remainingAngle;
		}
		t->setEndCriteria(Angle(remainingAngle));
	}
	if(currentTask.getAction().getLinearSpeed()>0){
		step-= (stepsTraversed*MOTOR_CALLBACK)*currentTask.action.getLinearSpeed();
	}			// -estimated distance covered
	//printf("adjusted\n");
}


std::vector <edgeDescriptor> Configurator::inEdgesRecursive(vertexDescriptor v, TransitionSystem& g, Direction d){
	 	std::vector <edgeDescriptor> result;
		edgeDescriptor e; 
		do{
			std::vector <edgeDescriptor> directionEdges=gt::inEdges(g, v, d);
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

std::vector <Frontier> Configurator::frontierVertices(vertexDescriptor v, TransitionSystem& g, Direction d, bool been){
	std::vector <Frontier> result;
	std::pair<edgeDescriptor, bool> ep=boost::edge(movingVertex, v, g); 
	vertexDescriptor v0=v, v1=v, v0_exp;
	//do{
		if ((controlGoal.disturbance.getPosition()-g[v].endPose.p).Length() < DISTANCE_ERROR_TOLERANCE){
		}
		else{
			auto es=boost::out_edges(v, g);
			for (auto ei=es.first; ei!=es.second; ei++){
			std::vector <vertexDescriptor>connecting;
			auto ei2=ei, ei3=ei;
			auto es2=boost::out_edges((*ei).m_target, g);
			auto es3=es2;
			std::vector <vertexDescriptor>connecting2;
			do {
				//=connecting;
				if (g[(*ei3).m_target].visited() || been){
					if (!g[(*ei3).m_target].visited()){
						EndedResult er = estimateCost(g[(*ei3).m_target], g[(*ei3).m_source].endPose, g[*ei3].direction);
						g[(*ei3).m_target].phi=evaluationFunction(er);
					}
					if (g[(*ei3)].direction==d){
						Frontier f;
						//f_added=1;
						f.first= (*ei3).m_target;
						f.second=connecting2;
						result.push_back(f);
						if (ei3!=ei){
							ei3++;
							//connecting2.clear();
							//ei2=ei3;
						}
						else{
							connecting.clear();
							break;
						}
					}
					else if (ei3==ei){
						connecting.push_back((*ei3).m_target);
						connecting2=connecting;
						es3=boost::out_edges((*ei3).m_target,g);
						ei3=es3.first;
						ei2=ei3;
						es2=es3;
					}
					else if (ei2!=ei){
						connecting2.push_back((*ei3).m_target);
						es3=boost::out_edges((*ei3).m_target,g);
						ei3=es3.first;
						ei2++;
					}
				}
				else if (ei3!=ei){
					ei3++;
				}
				if(ei3==es3.second){
					if (ei3!=ei2 & ei2!=ei){
						ei2++;
						ei3=ei2;
						es3=es2;
					}					
				}
				//printf("is stuck, ei3=%i ->%i\n", (*ei).m_source, (*ei).m_target);
			}while (ei3!=es3.second);
	}
	}

	// if(v==currentVertex & result.empty() 
	// 		& es.first!=es.second & ep.second){
	// 		v=ep.first.m_source;
	// 	}
	// 	else if (result.empty()&connecting.empty()){
	// 		break;
	// 	}
	// 	if (!connecting.empty()){
	// 		v=connecting[0];
	// 		connecting.erase(connecting.begin());
	// 	}
	// 	else{
	// 		break;
	// 	}
	// }while (true);
	return result;
}



// std::vector <edgeDescriptor> Configurator::frontierVertices(vertexDescriptor v, TransitionSystem& g, Direction d, bool been){
// 	std::vector <edgeDescriptor> result;
// 	std::pair<edgeDescriptor, bool> ep=boost::edge(movingVertex, v, g); 
// 	std::vector <vertexDescriptor>connecting;
// 	do{
// 		if ((controlGoal.disturbance.getPosition()-g[v].endPose.p).Length() < DISTANCE_ERROR_TOLERANCE){
// 			break;
// 		}
// 		auto es=boost::out_edges(v, g);
// 		for (auto ei=es.first; ei!=es.second; ei++){
// 			if (g[(*ei).m_target].visited() || been){
// 				if (g[(*ei)].direction==d){
// 					result.push_back((*ei)); //assumes maximum depth of 2 vertices traversed
// 				}
// 				else{
// 					connecting.push_back((*ei).m_target);
// 				}
// 			}
// 		}
// 		if(v==currentVertex & result.empty() 
// 			& es.first!=es.second & ep.second){
// 			v=ep.first.m_source;
// 		}
// 		else if (result.empty()&connecting.empty()){
// 			break;
// 		}
// 		if (!connecting.empty()){
// 			v=connecting[0];
// 			connecting.erase(connecting.begin());
// 		}
// 		else{
// 			break;
// 		}
// 	}while (true);
// 	return result;
// }

std::pair <bool, vertexDescriptor> Configurator::findExactMatch(State s, TransitionSystem& g, State * src, Direction dir){
	std::pair <bool, vertexDescriptor> result(false, TransitionSystem::null_vertex());
	auto vs= boost::vertices(g);
	float prob=0;
	for (auto vi=vs.first; vi!= vs.second; vi++){
		vertexDescriptor v=*vi;
		bool Tmatch=true;
		std::vector <edgeDescriptor> ie=gt::inEdges(g, v, dir);
		Tmatch=!ie.empty()||dir==Direction::UNDEFINED;
		if (matcher.isPerfectMatch(s, g[v], src) & v!=movingVertex &Tmatch & boost::in_degree(v, g)>0){ 
			std::pair<bool, edgeDescriptor> most_likely=gt::getMostLikely(g, ie, iteration);
			if (!most_likely.first){
			}
			else if (g[most_likely.second].probability>prob){
				result.first=true;
				result.second=v;
				prob=g[most_likely.second].probability;

			}
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
			std::vector <edgeDescriptor> ie=gt::inEdges(g, v, dir);
			bool Tmatch=true;
			//if (dir!=Direction::UNDEFINED){
			Tmatch= !ie.empty()||dir==Direction::UNDEFINED;
			//}
			if (matcher.isPerfectMatch(g[v], g[*vi])&*vi!=movingVertex &Tmatch & boost::in_degree(*vi, g)>=ie.size()){ //
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


void Configurator::changeStart(b2Transform& start, vertexDescriptor v, TransitionSystem& g){
	if (g[v].outcome == simResult::crashed && boost::in_degree(v, g)>0){
		edgeDescriptor e = boost::in_edges(v, g).first.dereference();
		start = g[e.m_source].endPose;
	}
	else{
		start=g[v].endPose;
	}
}


ExecutionError Configurator::trackTaskExecution(Task & t){
	ExecutionError error;
	// if (planVertices.empty() & planning){
	// 	return error;
	// }
	//PROLONGING DURATION OF TASK IF NEEDED
	std::unordered_map<State*, ExecutionError>::iterator it;
	// if (it=errorMap.find(transitionSystem[currentVertex].ID); it!=errorMap.end()){
	// 	error=it->second;
	// 	it->second=ExecutionError();
	// }
	if (t.motorStep>0 & fabs(error.r())<TRACKING_ERROR_TOLERANCE & fabs(error.theta())<TRACKING_ANGLE_TOLERANCE){
		t.motorStep--;
		printf("step =%i\n", t.motorStep);
	}
	else if (fabs(error.r())>=TRACKING_ERROR_TOLERANCE){
		int correction=-std::floor(error.r()/(t.action.getLinearSpeed()*LIDAR_SAMPLING_RATE)+0.5);
		t.motorStep+=correction; //reflex
	}
	else if (fabs(error.theta())>=TRACKING_ANGLE_TOLERANCE & t.action.getOmega()!=0){
		int correction=-std::floor(error.theta()/(t.action.getOmega()*MOTOR_CALLBACK)+0.5);
		t.motorStep+=correction; //reflex
	}		

	// b2Transform deltaPose;
	// b2Rot angularDisplacement(getTask()->getAction().getOmega()*MOTOR_CALLBACK +error.theta());
	// float xdistance=angularDisplacement.c * getTask()->getAction().getLinearSpeed()*MOTOR_CALLBACK +error.theta();
	// float ydistance=angularDisplacement.s * getTask()->getAction().getLinearSpeed()*MOTOR_CALLBACK;
	// deltaPose=b2Transform(b2Vec2(xdistance,
	// 				ydistance), 
	// 				angularDisplacement); //og rot
	updateGraph(transitionSystem, error);//lateral error is hopefully noise and is ignored
	//printf("deltapose= %f, %f, %f\n", deltaPose.p.x, deltaPose.p.y, deltaPose.q.GetAngle());
	if(t.motorStep==0){
		t.change=1;
	}
	// else{
	// 	error.setTheta(taskRotationError()); //will be rest at the next callback
	// }
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

std::vector <vertexDescriptor> Configurator::changeTask(bool b, int &ogStep, std::vector <vertexDescriptor> pv){
	//printf("currentTask change= %i\n", currentTask.change);
	if (!b){
		return pv;
	}
	if (planning){
		if (pv.empty()){
			//printf("no plan, bas\n");
			return pv;
		}
		if (movingEdge==edgeDescriptor()){
			//printf("moving edge is null\n");
		}
		if (movingEdge.m_target==TransitionSystem::null_vertex()){
			//printf("null current\n");
		}
		//printf("change plan\n");
		std::pair<edgeDescriptor, bool> ep=boost::add_edge(currentVertex, pv[0], transitionSystem);
	//	printf("ep exists=%i, src=%i, tgt=%i\n", !ep.second, ep.first.m_source, ep.first.m_target);
		currentVertex= pv[0];
		pv.erase(pv.begin());
		currentEdge=ep.first;

		boost::clear_vertex(movingVertex, transitionSystem);
	//	printf("changed current %i + cleared 0\n", currentVertex);
		movingEdge=boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
		transitionSystem[movingEdge].direction=transitionSystem[ep.first].direction;
		transitionSystem[movingEdge].step=currentTask.motorStep;
		currentTask = Task(transitionSystem[currentEdge.m_source].disturbance, transitionSystem[currentEdge].direction, b2Transform(b2Vec2(0,0), b2Rot(0)), true);
		currentTask.motorStep = transitionSystem[currentEdge].step;
	//	printf("l=%f, r=%f, step=%i\n", currentTask.action.L, currentTask.action.R, currentTask.motorStep);
	}
	else{
		if (transitionSystem[0].disturbance.isValid()){
			currentTask = Task(transitionSystem[currentVertex].disturbance, DEFAULT); //reactive
		}
		else if(currentTask.direction!=DEFAULT){
				currentTask = Task(transitionSystem[currentVertex].disturbance, DEFAULT); //reactive
		}
		else{
			currentTask = Task(controlGoal.disturbance, DEFAULT); //reactive
		}

		currentTask.motorStep = motorStep(currentTask.getAction());
		printf("changed to reactive\n");
	}
	ogStep = currentTask.motorStep;
	return pv;
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

void Configurator::updateGraph(TransitionSystem&g, ExecutionError error){
	//b2Rot rot(getTask()->getAction().getOmega()*MOTOR_CALLBACK);
	// b2Transform deltaPose;
	// //if (fabs(error)<TRACKING_ERROR_TOLERANCE){
	// float xdistance=getTask()->getAction().getLinearVelocity().x*MOTOR_CALLBACK+error.r();
	// b2Rot angularDisplacement(getTask()->getAction().getOmega()*MOTOR_CALLBACK +error.theta());
	// deltaPose=b2Transform(b2Vec2(xdistance,
	// 				getTask()->getAction().getLinearVelocity().y*MOTOR_CALLBACK), 
	// 				angularDisplacement); //og rot
	// //}
	auto vPair =boost::vertices(g);
	float angularDisplacement= getTask()->getAction().getOmega()*MOTOR_CALLBACK +error.theta();
	float xdistance=cos(angularDisplacement) * getTask()->getAction().getLinearSpeed()*MOTOR_CALLBACK;
	float ydistance=sin(angularDisplacement) * getTask()->getAction().getLinearSpeed()*MOTOR_CALLBACK;
	b2Transform deltaPose=b2Transform(b2Vec2(xdistance,
					ydistance), 
					b2Rot(angularDisplacement));
	float linearDisplacement = SignedVectorLength(deltaPose.p);
	//printf("ang disp:%f, deltax=%f, deltay %f, linear disp%f\n", angularDisplacement, xdistance, ydistance, linearDisplacement );
	for (auto vIt= vPair.first; vIt!=vPair.second; ++vIt){ //each node is adjusted in explorer, so now we update
		if (*vIt!=movingVertex){
			// g[*vIt].endPose.q.Set(g[*vIt].endPose.q.GetAngle()-angularDisplacement);
			// g[*vIt].endPose.p.x-=xdistance;
			// g[*vIt].endPose.p.y-=ydistance;
			// g[*vIt].endPose.p.x= g[*vIt].endPose.p.x* cos(angularDisplacement)+ g[*vIt].endPose.p.y*sin(angularDisplacement);
			// g[*vIt].endPose.p.y= g[*vIt].endPose.p.y* cos(angularDisplacement)- g[*vIt].endPose.p.x*sin(angularDisplacement);
			applyAffineTrans(deltaPose, g[*vIt].endPose);
			if (g[*vIt].disturbance.getAffIndex()!=NONE){
				// g[*vIt].disturbance.pose().q.Set(g[*vIt].disturbance.pose().q.GetAngle()-angularDisplacement);
				// //float d_x= d_distance*cos(g[*vIt].disturbance.pose().q.GetAngle())-xdistance;
				// float d_y= d_distance*sin(g[*vIt].disturbance.pose().q.GetAngle())-ydistance;
				// g[*vIt].disturbance.pose().p.Set(d_x, d_y);
				applyAffineTrans(deltaPose, g[*vIt].disturbance.bf.pose);

			}
		
		}
	}

	// if (fabs(error)>TRACKING_ERROR_TOLERANCE){
	// 	deltaPose=b2Transform(b2Vec2(cos(rot.GetAngle())*error,
	// 					getTask()->getAction().getLinearVelocity().y*MOTOR_CALLBACK), 
	// 					rot);
	// }
	applyAffineTrans(deltaPose, controlGoal.start);
	if(controlGoal.getAffIndex()!=NONE){
		// //controlGoal.disturbance.subtractPose(deltaPose);
		// controlGoal.disturbance.pose().q.Set(controlGoal.disturbance.pose().q.GetAngle()-angularDisplacement);
		// float goal_distance=controlGoal.disturbance.pose().p.Length()- linearDisplacement;
		// float goal_x= goal_distance*cos(controlGoal.disturbance.pose().q.GetAngle());
		// float goal_y= goal_distance*sin(controlGoal.disturbance.pose().q.GetAngle());
		// controlGoal.disturbance.pose().p.Set(goal_x, goal_y);
		applyAffineTrans(deltaPose, controlGoal.disturbance.bf.pose);
	}
	//controlGoal.start-=deltaPose;



}