#include "configurator.h"
#include <chrono>

template <class T>
void debug::graph_file(int it, T& g, Disturbance goal, std::vector <vertexDescriptor> plan, vertexDescriptor c){
	char fileName[50];
	sprintf(fileName, "/tmp/graph%04i.txt", it);
	FILE * f=fopen(fileName, "w");
	auto vs=boost::vertices(g);
	for (auto vi=vs.first; vi!=vs.second; vi++){
		auto es=boost::out_edges(*vi, g);
		// if (goal.getAffIndex()!=NONE){
		// 	b2Vec2 v = goal.getPosition() - g[*vi].endPose.p; //distance between disturbance and robot
		// 	if (v.Length()<DISTANCE_ERROR_TOLERANCE){
		// 		fprintf(f, "*");
		// 	}
		// }
		if (*vi==c){
			fprintf(f,"!");
		}
		for (vertexDescriptor vp:plan){
			if (*vi==vp){
				fprintf(f,"*");
			}
		}
		fprintf(f,"%i -> ", *vi);
		for (auto ei=es.first; ei!=es.second; ei++){
			fprintf(f, "%i (%f) ", (*ei).m_target, g[(*ei)].probability);
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
	previousTimeScan=now; //update the time of sampling

	if (timerOff){
		timeElapsed = .2;
	}

	//CREATE BOX2D ENVIRONMENT
	b2Vec2 gravity = {0.0, 0.0};
	b2World world= b2World(gravity);
	char name[256];

	auto startTime =std::chrono::high_resolution_clock::now();
	bool explored=0;
	if (planning){ //|| !planError.m_vertices.empty())
		auto checkedge = boost::edge(movingVertex, currentVertex, transitionSystem);
		transitionSystem[movingVertex].disturbance=transitionSystem[currentVertex].disturbance;
		transitionSystem[movingVertex].disturbance.invalidate();
		if (debugOn){
			printf("moving edge= %i -> %i\n", movingEdge.m_source, movingEdge.m_target);
		}
		//have I seen this envronment configuration before?
		vertexDescriptor src; //ve=TransitionSystem::null_vertex(),
		if (!planVertices.empty()){
			//ve= *(planVertices.rbegin().base()-1);
			src=movingVertex;
		}
		else {
			//ve=currentVertex;
			src=currentVertex;
		}
	bool plan_works=true;
	if (transitionSystem.m_vertices.size()<2){
		plan_works=false;
	}
	std::vector <std::pair <vertexDescriptor, vertexDescriptor>> toRemove;
	std::vector <vertexDescriptor> plan_provisional=planVertices;
	done_that(src, plan_works, world, plan_provisional);
	printf("plan provisional size = %i, plan_works=%i", plan_provisional.size(), plan_works);
	if (!plan_works){	// boost::out_degree(src, transitionSystem) <1		
		is_not_v not_cv(currentVertex);
		planVertices.clear();
		boost::clear_vertex(movingVertex, transitionSystem);
		if (transitionSystem.m_vertices.size()==1){
			dummy_vertex(currentVertex);//currentEdge.m_source
		}
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
		planVertices= planner(transitionSystem, src);
		printPlan();
		debug::graph_file(iteration, transitionSystem, controlGoal.disturbance, planVertices, currentVertex);
		boost::remove_out_edge_if(movingVertex, not_cv, transitionSystem);
		explored=1;
	//	printf("after remoing out edges from 0->current=%i exists=%i\n", currentVertex, currentEdge !=edgeDescriptor());
		//boost::print_graph(transitionSystem);
	}
	else if (planVertices.empty()&&currentTask.motorStep==0){
		//reset to new src
		planVertices=plan_provisional;
		b2Transform deltaPose = transitionSystem[movingVertex].start - transitionSystem[src].start;
		//updateGraph(transitionSystem, ExecutionError(),& deltaPose);
		applyAffineTrans(deltaPose, transitionSystem);
	}
	//if plan fails or not there, 
	

	// if (debugOn){
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
	//printf("took %f seconds\n", duration);
	if (benchmark){
		FILE * f = fopen(statFile, "a+");
		if (explored){
			fprintf(f, "*");
		}
		fprintf(f,"%i\t%i\t%f\n", worldBuilder.getBodies(), transitionSystem.m_vertices.size(), duration);
		fclose(f);
	}
	worldBuilder.resetBodies();
	//printf("end explor\n");
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
	//set d orientation
	
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
// 			std::pair<bool, vertexDescriptor> match=findMatch(sk.first, g, source, t.direction);			
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
	Direction direction=currentTask.direction;
	std::vector <vertexDescriptor> priorityQueue = {bestNext};
	std::vector <vertexDescriptor> closed;
	b2Transform start= b2Transform(b2Vec2(0,0), b2Rot(0));
	std::vector<std::pair<vertexDescriptor, vertexDescriptor>> toRemove;
	printf("EXPLORING\n");
	do{
		v=bestNext;
		closed.push_back(*priorityQueue.begin());
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
			changeStart(start, v0, g);
			std::pair <State, Edge> sk(State(start), Edge(g[v0].options[0]));
			bool topDown=1;
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
			StateMatcher::MATCH_TYPE vm= matcher.isMatch(g[v], g[currentEdge.m_source]); //see if we are at the beginning of the exploration:
																				//v=0 and currentEdge =src will match so we try to prevent
																		//changing the movign vertex which is by default the origin
			if (v0==movingVertex & vm==StateMatcher::_TRUE){
				source= g[currentEdge.m_source].ID;
			}
			else{
				source=g[v0].ID;
			}
			std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> match=findMatch(sk.first, g, source, t.direction);			
			std::pair <edgeDescriptor, bool> edge(edgeDescriptor(), false);
			if (match.first==StateMatcher::MATCH_TYPE::_TRUE){
				g[v0].options.erase(g[v0].options.begin());
				v1=match.second; //frontier
				if ((v0!=v1)){
					edge.first= boost::add_edge(v0, v1, g).first; //assumes edge added
					edge.second=true; //just means that the edge is valid
					g[edge.first]=sk.second;//t.direction;
				}
				else{
					printf("same vertex %i\n", v1);
				}
			}
			else if (match.first==StateMatcher::DISTURBANCE){
					printf("SIMILAR to %i\n", match.second);
			}
			else{
				edge= addVertex(v0, v1,g, Disturbance(),sk.second);
				g[edge.first.m_target].label=sk.first.label; //new edge, valid
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
		addToPriorityQueue(v1, priorityQueue, g, closed);
		}
	}
	bestNext=priorityQueue[0];
		if (controlGoal.getAffIndex()==PURSUE){
			//printf("best=%i, options=%i\n", bestNext, g[bestNext].options);
	}
	direction = g[boost::in_edges(bestNext, g).first.dereference()].direction;
}while(g[bestNext].options.size()>0);
printf("finished exploring\n");
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
			std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> match= findMatch(ep.first.m_target, g, g[ep.first].direction);
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

std::vector <vertexDescriptor> Configurator::planner( TransitionSystem& g, vertexDescriptor src, vertexDescriptor goal, bool been, const Task* custom_ctrl_goal){
	std::vector <vertexDescriptor> plan;
	std::vector<std::vector<vertexDescriptor>> paths;
	paths.push_back(std::vector<vertexDescriptor>()={src});
	std::vector <Frontier> frontier_v;
	bool run=true;
	std::vector <Frontier> priorityQueue;
	if (currentVertex==movingVertex){
		printf("current %i =moving%i! return, src=%i\n", currentVertex, movingVertex, src);
		//return plan;
	}
	Task overarching_goal;
	if (NULL==custom_ctrl_goal){
		overarching_goal=controlGoal;
	}
	else{
		overarching_goal=*custom_ctrl_goal;
	}
	int no_out=0;

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
		while (!edge.second){//|| ((*(pend.base()-1)!=goal &goal!=TransitionSystem::null_vertex())&!controlGoal.checkEnded(g[*(pend.base()-1)]).ended)
			//printf("src = %i possible paths:%i, path length=%i, add length=%i, frontier l=%i\n", src, paths.size(), path->size(), add.size(), frontier_v.size());
			vertexDescriptor end=*(pend.base()-1);
			edge= boost::edge(end,add[0], g);
			//printf("edge %i->%i", end, add[0]);
			if (!add.empty()&!edge.second & path!=paths.rend()){ //if this path does not have an edge and there are 
													//other possible paths, go to previous paths
				if (pend.base()-1!=(path->begin())){ //if the current vertex is not the root of the path
				//	printf("from index %i ", *(pend.base()-1));
					pend++;
					//printf("going back in current path, path size = %i, current index =%i\n", path->size(), *(pend.base()-1));
				}
				else{
					path++; //go back a previously explored path
					pend=(*path).rbegin(); 
				//	printf("checking previous path\n");
				}
			}
			else if (edge.second & pend.base()!=path->rbegin().base()){  //if there is an edge with the end of current path
				bool found=0;
				for (auto _p=paths.rbegin(); _p!=paths.rend(); _p++ ){
					if (std::vector <vertexDescriptor>(path->begin(), pend.base())==*_p){
						path=_p;
						//printf("using existing\n");
						found=1;
					}
				}
				if (!found){
				paths.emplace_back(std::vector <vertexDescriptor>(path->begin(), pend.base()));
				path=paths.rbegin();				
				}
				break;
			}
			if ( path==paths.rend()) { //if there are no other paths
				paths.push_back(std::vector<vertexDescriptor>()); //make a new one
				path=paths.rbegin();
				printf("new empty path\n");
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
		printf("exited inner while\n");
		// bool fin = overarching_goal.checkEnded(g[path_end].endPose, UNDEFINED, true).ended;
		// if (fin){
		// 	goal=path_end;
		// }
	}while(!priorityQueue.empty() & (path_end!=goal &!overarching_goal.checkEnded(g[path_end].endPose, UNDEFINED, true).ended));
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
	//printPlan();
	return plan;
}

// std::vector <vertexDescriptor> Configurator::planner2( TransitionSystem& g, vertexDescriptor src, vertexDescriptor goal, bool been){
// 	std::vector <vertexDescriptor> plan;
// 	std::vector<std::vector<vertexDescriptor>> paths;
// 	paths.push_back(std::vector<vertexDescriptor>()={src});
// 	std::vector <Frontier> frontier_v;
// 	bool run=true;
// 	std::vector <Frontier> priorityQueue;
// 	if (currentVertex==movingVertex){
// 		//printf("current %i =moving%i! return\n", currentVertex, movingVertex);
// 		return plan;
// 	}
// 	int no_out=0;

// 	std::vector <vertexDescriptor> add;
// 	std::vector<std::vector<vertexDescriptor>>::iterator path= paths.begin();
// 	vertexDescriptor path_end=src;
// 	do{
// 		//find frontier (STRAIGHT)
// 		//printf("start\n");
// 		frontier_v=frontierVertices(src, g, DEFAULT, been);
// 		if (src==currentVertex){
// 		//	printf("planning from src =%i, out vertices n%i\n", src, frontier_v.size());
// 		}
// 		for (Frontier f: frontier_v){ //add to priority queue
// 			planPriority(g, f.first);
// 			addToPriorityQueue(f, priorityQueue, g);
// 		}
// 		if (priorityQueue.empty()){
// 		//	printf("emtpy pq\n");
// 			break;
// 		}
// 		src=priorityQueue.begin()->first;
// 		add=std::vector <vertexDescriptor>(priorityQueue.begin()->second.begin(), priorityQueue.begin()->second.end());
// 		add.push_back(src);
// 		std::pair<edgeDescriptor, bool> edge(edgeDescriptor(), false);
// 		std::vector<vertexDescriptor>::iterator v_index=(path->end()-1); //pend
// 		while (!edge.second){//|| ((*(pend.base()-1)!=goal &goal!=TransitionSystem::null_vertex())&!controlGoal.checkEnded(g[*(pend.base()-1)]).ended)
// 			printf("src = %i possible paths:%i, path length=%i\n", src, paths.size(), path->size());
// 			//vertexDescriptor end=*(p;
// 			edge= boost::edge(*v_index,add[0], g);
// 			printf("edge %i->%i\n", *v_index, add[0]);
// 			if (!add.empty()&!edge.second & path!=paths.end()){ //if this path does not have an edge and there are 
// 													//other possible paths, go to previous paths
// 				if (v_index !=path->rend().base()){ //if the current vertex is not the root of the path
// 					printf("from index %i ", *(v_index));
// 					v_index--;
// 					printf("going back in current path, path size = %i, current index =%i\n", path->size(), *(pend.base()-1));
// 				}
// 				else{
// 					path++; //go back a previously explored path
// 					pend=(*path).rbegin(); 
// 					printf("checking previous path\n");
// 				}
// 			}
// 			else if (edge.second & pend.base()!=path->rbegin().base()){  //if there is an edge with the end of current path
// 				for (auto _p=paths.rbegin(); _p!=paths.rend(); _p++ ){
// 					if (std::vector <vertexDescriptor>(path->begin(), pend.base())==*_p){
// 						path=_p;
// 						printf("using existing\n");
// 					}
// 					else{
// 						paths.emplace_back(std::vector <vertexDescriptor>(path->begin(), pend.base()));
// 						path=paths.rbegin();				
// 						printf("new path based on previous path\n");
// 					}
// 				}
// 				break;
// 			}
// 			else{
// 				printf("doing nothing\n");
// 				break;
// 			}
// 			if ( path==paths.rend()) { //if there are no other paths
// 				paths.push_back(std::vector<vertexDescriptor>()); //make a new one
// 				path=paths.rbegin();
// 				printf("new empty path\n");
// 				break;
// 			}	
// 		}
// 		priorityQueue.erase(priorityQueue.begin());
// 		for (vertexDescriptor c:add){
// 			g[c].label=VERTEX_LABEL::UNLABELED;
// 			path->push_back(c);	
// 			path_end=c;			
// 		}
// 		// printf("planning, path size= %i\n",path->size() );
// 		// printf("pq empty=%i, path end=%i, ended=%i\n", priorityQueue.empty(), path_end, controlGoal.checkEnded(g[path_end].endPose).ended);
// 		// printf("conf running=%i\n", running);
		
// 	}while(!priorityQueue.empty() & (path_end!=goal &!controlGoal.checkEnded(g[path_end].endPose, UNDEFINED, true).ended));
// 	//printf("exited while\n");
// 	auto vs=boost::vertices(g);
// 	float final_phi=10000;
// 	for (std::vector<vertexDescriptor> p: paths){
// 		vertexDescriptor end_plan= *(p.rbegin().base()-1);
// 		if (end_plan==goal){
// 			plan=std::vector(p.begin()+1, p.end());
// 			break;
// 		}
// 		else if (g[end_plan].phi<final_phi){
// 			plan=std::vector(p.begin()+1, p.end());
// 			final_phi=g[end_plan].phi;
// 		}
// 	}
// 	// for (auto vi=vs.first; vi!=vs.second; vi++){
// 	// 	if (g[*vi].label!=UNLABELED){
// 	// 		boost::clear_vertex(*vi, g);
// 	// 	}
// 	// }
// 	printf("PLANNED!\n");
// 	printPlan();
// 	return plan;
// }


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

bool Configurator::checkPlan(b2World& world, std::vector <vertexDescriptor> &p, TransitionSystem &g, b2Transform start, vertexDescriptor custom_start){
	b2Vec2 v = controlGoal.disturbance.getPosition() - b2Vec2(0,0);
	//printf("check goal start: %f, %f, %f, distance = %f, valid =%i\n", controlGoal.start.p.x,controlGoal.start.p.y, controlGoal.start.q.GetAngle(), v.Length(), controlGoal.disturbance.isValid());
	//printf("CHECK goal position= %f, %f, %f, valid =%i\n", controlGoal.disturbance.pose().p.x, controlGoal.disturbance.pose().p.y, controlGoal.disturbance.pose().q.GetAngle(), controlGoal.disturbance.isValid());
	bool result=true;
	int it=0;//this represents currentv
	//printPlan();
	std::vector <vertexDescriptor> vpt=p;
	if (p.empty() && currentTask.motorStep==0){
		return false;
	}
	if (p.size()>0){
		if (p[0]==currentVertex){
			p.erase(p.begin());
		}		
	}	
	if (custom_start==TransitionSystem::null_vertex()){
		custom_start=movingVertex;
		it=-1;
	}
	auto ep=boost::edge(custom_start, *p.begin(), g);	
	do {
		b2Transform shift=g[movingVertex].endPose-g[ep.first.m_target].endPose;
		Disturbance d_adjusted=g[ep.first.m_source].disturbance;
		applyAffineTrans(shift, d_adjusted);
		Task t= Task(d_adjusted, g[ep.first].direction, start, true);
		float stepDistance=BOX2DRANGE;
		worldBuilder.buildWorld(world, data2fp, start, t.direction, t.disturbance);
		std::pair <State, Edge> sk(State(start), Edge(t.direction));
		printf("from %i", ep.first.m_target);
		vertexDescriptor t_start_v=ep.first.m_target; //vertex denoting start of task
		b2Transform endPose=skip(ep.first,g,it, &t, stepDistance, p);
		printf("to it %i, edge %i ->%i, stepDistance %f, direction = %i\n", it, ep.first.m_source, ep.first.m_target, stepDistance, g[ep.first].direction);
		simResult sr=t.willCollide(world, iteration, debugOn, SIM_DURATION, stepDistance);
		gt::fill(sr, &sk.first, &sk.second); //this also takes an edge, but it'd set the step to the whole
									// simulation result step, so this needs to be adjusted
		b2Transform expected_deltaPose=(g[ep.first.m_source].start-endPose);
		if ((sk.first.start.p-sk.first.endPose.p).Length()> expected_deltaPose.p.Length()){
			sk.first.endPose=sk.first.start+expected_deltaPose;
			sk.first.outcome=simResult::successful;
		}
		start = sk.first.endPose;
		StateDifference sd;
		StateMatcher::MATCH_TYPE is_match=matcher.isMatch(g[ep.first.m_source], sk.first, NULL, &sd);
		vertexDescriptor v1=ep.first.m_source;
		std::pair <bool,edgeDescriptor> prev_edge= gt::getMostLikely(g, gt::inEdges(g, v1, t.direction),iteration);
		if (!prev_edge.first){
			auto _moving=boost::add_edge(movingVertex, v1, g);
			prev_edge.second=_moving.first;
		}
		if (!matcher.match_equal(is_match, StateMatcher::DISTURBANCE)){
			result=false;
			break;
		}
		else{
			gt::update(prev_edge.second, sk, g,true, errorMap, iteration);
		}
		propagateD(v1,prev_edge.second.m_source, g);
		gt::adjustProbability(g, ep.first);
		printf("skipping from %i, edge %i ->%i", it, ep.first.m_source, ep.first.m_target);
	}while (ep.first.m_target!=TransitionSystem::null_vertex() & it <int(p.size()-1) & result );
	return result;
}


b2Transform Configurator::skip(edgeDescriptor& e, TransitionSystem &g, int& i, Task* t, float& step, std::vector <vertexDescriptor> plan){ 
	b2Transform result;
	edgeDescriptor e_start=e;

	// if (e_start==movingEdge){
	// 	e_start=currentEdge;
	// }
	vertexDescriptor v_tgt= e.m_target;
//adjust here
	do{
		i++;
		//printf("iterator = %i, e src= %i, e trgt= %i\n", i, e.m_source, e.m_target);
			auto es = boost::out_edges(e.m_target, g);
			g[e].it_observed=iteration;
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
					v_tgt=e.m_source;
					break;
				}
			}
		result=g[e.m_source].endPose; 


		}while (g[e].direction==t->direction & i<planVertices.size()& g[e].direction==DEFAULT);
//	printf("ended skip, result = %f, %f, %f\n", result.p.x, result.p.y, result.q.GetAngle());
	if (g[e_start.m_target].disturbance.getAffIndex()!=NONE){
		printf("targ=%i, d index=%i, d hl=%f, d hw=%f\n", e_start.m_target, g[e_start.m_target].disturbance.getAffIndex(), g[e_start.m_target].disturbance.bodyFeatures().halfLength, g[e_start.m_target].disturbance.bodyFeatures().halfWidth);
		step=b2Vec2(g[e_start.m_source].endPose.p-g[e_start.m_target].disturbance.pose().p).Length();
		//was e.m_target
	}
	else{
		if (g[e_start].direction==DEFAULT){
			step = (g[e_start.m_source].endPose.p- g[v_tgt].endPose.p).Length();
			printf("step=%f\n", step);
		}
		adjustStepDistance(e_start.m_source,g, t, step, std::pair(true,v_tgt));
		printf("adjusted step=%f\n", step);
	}

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
	srand(unsigned(time(NULL)));
	if (state.outcome != simResult::successful){ //accounts for simulation also being safe for now
		if (d ==DEFAULT ||d==STOP){
			if (state.nodesInSameSpot<maxNodesOnSpot){
				//in order, try the task which represents the reflex towards the goal
				if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
					state.options.push_back(temp.direction);
					state.options.push_back(getOppositeDirection(temp.direction).second);
				}
				else{
					int random= rand();
					if (random%2==0){
						state.options = {LEFT, RIGHT};
					}
					else{
						state.options = {RIGHT, LEFT};
					}
				}
			}
			}
	}
	else { //will only enter if successful
		if (d== LEFT || d == RIGHT){
			state.options = {DEFAULT};
			if (src==currentVertex && controlGoal.getAffIndex()==PURSUE && SignedVectorLength(controlGoal.disturbance.pose().p)<0){
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
				int random= rand();
				if (random%2==0){
					state.options = {DEFAULT, LEFT, RIGHT};
				}
				else{
					state.options = {DEFAULT, RIGHT, LEFT};
				}
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
			//printf("no options added because %i ended\n", v0);
			return;
		}
	}
	else if(round(g[v0].endPose.p.Length()*100)/100>=BOX2DRANGE){ // OR g[vd].totDs>4
		if (controlGoal.getAffIndex()==PURSUE){
			//printf("no options added because %i out of bounds\n", v0);
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

void Configurator::addToPriorityQueue(vertexDescriptor v, std::vector<vertexDescriptor>& queue, TransitionSystem &g, std::vector <vertexDescriptor>& closed){
	for (auto i =queue.begin(); i!=queue.end(); i++){
		bool expanded=0;
		for (vertexDescriptor c: closed){
			if (v==c){
				expanded=true;
			}
		}
		if (g[v].phi <abs(g[*i].phi) & !expanded){
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

	std::pair <float, vertexDescriptor> best(10000, TransitionSystem::null_vertex());
	float best_prob=0;
	auto vs=boost::vertices(g);
	for (auto vi=vs.first; vi!=vs.second; vi++ ){
		// if (target.getAffIndex()==PURSUE){
		// 	//smallest difference to target position
		// 	// sd.D_position.x= g[*vi].endPose.p.x-target.pose().p.x;
		// 	// sd.D_position.y =g[*vi].endPose.p.y-target.pose().p.y;
		// 	// sd.D_angle = angle_subtract(g[*vi].endPose.q.GetAngle(), target.pose().q.GetAngle());
		// }
		if (target.getAffIndex()==NONE){
			//b2Transform reference;
			b2Vec2 reference = g[movingVertex].endPose.p-controlGoal.start.p;
			//reference.q.Set(reference.q.GetAngle()- g[movingVertex].endPose.q.GetAngle());
			float remaining= BOX2DRANGE-SignedVectorLength(reference);
			target.bf.pose.p.x=remaining *cos(g[*vi].endPose.q.GetAngle());
			target.bf.pose.p.y=remaining*sin(g[*vi].endPose.q.GetAngle());
			target.bf.pose.q.Set((g[*vi].endPose.q.GetAngle()));
			// sd.r_position.x=g[*vi].endPose.p.x-target_pose.p.x;
			// sd.r_position.y =g[*vi].endPose.p.y-target_pose.p.y;
			// sd.r_angle =g[*vi].endPose.q.GetAngle()-target_pose.q.GetAngle();
		}
		State s_target;
		s_target.disturbance=target;
		StateDifference sd(g[*vi], s_target);
		float sum=sd.sum_d_pos();
		auto in_edges = gt::inEdges(g, *vi);
		auto most_likely_edge=gt::getMostLikely(g, in_edges, iteration);
		printf("got likely edge of %i, %i, %i -> %i tot edges =%i\n",*vi, most_likely_edge.first, most_likely_edge.second.m_source, most_likely_edge.second.m_target, in_edges.size());
		float prob= 0;
		if (most_likely_edge.first ){
			prob=g[most_likely_edge.second].probability;//.weighted_probability(iteration);
			printf("set prob for %i:  %f\n", *vi, prob);
		}
		if (StateMatcher::StateMatch sm(sd, matcher.error); sm.disturbance_exact() & (sum<best.first || (sum==best.first & prob>best_prob))){
				best.first=sum;
				best.second=*vi;
				best_prob=prob;
				result.first=true;
			}
	}
	result.second=best.second;
	//printf("goal =%f, %f, %f", target.pose().p.x, target.pose().p.y, target.pose().q.GetAngle());
	printf("been=%i, vertex=%i, pose: %f, %f, %f\n", result.first, result.second, g[result.second].endPose.p.x,g[result.second].endPose.p.y, g[result.second].endPose.q.GetAngle() );
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



void Configurator::adjustStepDistance(vertexDescriptor v, TransitionSystem &g, Task * t, float& step, std::pair<bool,vertexDescriptor> tgt){
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, g);

	// if (tgt.first & !g[v].disturbance.isValid()){
	// 	step = (g[v].endPose.p- g[tgt.second].endPose.p).Length();
	// }
	if(!ep.second){ //no tgt	
		return; //check until needs to be checked
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

	return result;
}

void Configurator::recall_plan_from(const vertexDescriptor& v, TransitionSystem & g, b2World &world,  std::vector <vertexDescriptor>& plan_provisional, bool & plan_works){
    auto srcs= gt::inEdges(g, v);
    vertexDescriptor src=v;
    if(!srcs.empty()){
		src= srcs[0].m_source;
	}
	b2Transform o_shift= -g[src].endPose;
	Task controlGoal_adjusted= controlGoal;
	applyAffineTrans(o_shift, controlGoal_adjusted);
	plan_provisional=planner(g, src, TransitionSystem::null_vertex(), false, &controlGoal_adjusted); //been.second, been.first
	auto vi= (plan_provisional.end()-1);
	vertexDescriptor end =*(vi);
	bool ctrl_finished = controlGoal_adjusted.checkEnded(g[end]).ended;
	if (ctrl_finished){
		plan_works= checkPlan(world, plan_provisional, g,  g[movingVertex].start,src);
		if (plan_works){
			return;
		}
	}
	plan_provisional.clear();

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

std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> Configurator::findMatch(State s, TransitionSystem& g, State * src, Direction dir, StateMatcher::MATCH_TYPE match_type, std::vector <vertexDescriptor> * others, bool relax){
	std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> result(StateMatcher::MATCH_TYPE::_FALSE, TransitionSystem::null_vertex());
	auto vs= boost::vertices(g);
	float prob=0, sum=10000;
	//need to find best match too
	ComparePair comparePair;
	std::set <std::pair <vertexDescriptor, float>, ComparePair>others_set(comparePair);
	for (auto vi=vs.first; vi!= vs.second; vi++){
		vertexDescriptor v=*vi;
		bool Tmatch=true;
		std::vector <edgeDescriptor> ie=gt::inEdges(g, v, dir);
		Tmatch=!ie.empty()||dir==Direction::UNDEFINED;
		StateDifference sd(s, g[v]);
		bool condition=0;
		StateMatcher::MATCH_TYPE m;
		float sum_tmp=sd.get_sum(match_type);
		if (!relax){
			m= matcher.isMatch(s, g[v], src, &sd);
			condition=matcher.match_equal(m, match_type);
		}
		else{
			condition= sum_tmp<sum;
		}
		if (v==2 && others!=NULL){
			printf("v%i sum = %f, condition=%i, relax=%i, Tmatch=%i\n", v, sum_tmp, condition, relax, Tmatch);
		}
		if ( condition&& v!=movingVertex && boost::in_degree(v, g)>0 &&Tmatch ){ 
			sum=sum_tmp;
			//std::pair<bool, edgeDescriptor> most_likely=gt::getMostLikely(g, ie, iteration);
			if (NULL!=others){
			 	others_set.emplace(std::pair< vertexDescriptor, float>(v, sum));
			 }
			// if (g[most_likely.second].it_observed<1){
				
			// }
			// else if (most_likely.first ){ //&&g[most_likely.second].probability>prob
				if (!relax){
					result.first= m;
				}
				else{
					result.first=match_type;
				}
				result.second=v;
				//prob=g[most_likely.second].probability;
				
			//}
		}
	}
	if (others==NULL){
		return result;
	}
	printf("others not null\n");
	for (auto vp:others_set){
		others->push_back(vp.first);
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

std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> Configurator::findMatch(vertexDescriptor v, TransitionSystem& g, Direction dir, StateMatcher::MATCH_TYPE match_type, std::vector <vertexDescriptor>* others){
	std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> result(StateMatcher::_FALSE, TransitionSystem::null_vertex());
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
			if (StateMatcher::MATCH_TYPE m=matcher.isMatch(g[v], g[*vi]); matcher.match_equal(m, match_type) && *vi!=movingVertex &Tmatch & boost::in_degree(*vi, g)>=ie.size()){ //
			//std::pair<bool, edgeDescriptor> most_likely=gt::getMostLikely(g, ie);
			//if (!most_likely.first){
			//}
			//else if (g[most_likely.second].probability>prob){
				if(g[v].nObs>nObs){
				result.first=matcher.isMatch(g[v], g[*vi]);
				result.second=*vi;
				//prob=g[most_likely.second].probability;
				nObs=g[v].nObs;
				if (NULL!=others){
					others->push_back(result.first);
				}
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
			//if (currentVertex!=movingVertex){
				printf("I DON'T KNOW WHAT TO DO NOW\n");
				currentTask=Task(STOP);
			//}
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

void Configurator::applyAffineTrans(const b2Transform& deltaPose, Task& task){
	math::applyAffineTrans(deltaPose, task.start);
	// if (task.disturbance.getAffIndex()!=NONE){
	// 	math::applyAffineTrans(deltaPose, task.disturbance.bf.pose);
	// }
	applyAffineTrans(deltaPose, task.disturbance);
}

void Configurator::applyAffineTrans(const b2Transform& deltaPose, TransitionSystem& g){
	auto vPair =boost::vertices(g);
	for (auto vIt= vPair.first; vIt!=vPair.second; ++vIt){ //each node is adjusted in explorer, so now we update
	if (*vIt!=movingVertex){
		math::applyAffineTrans(deltaPose, g[*vIt]);
	}
}
}

void Configurator::applyAffineTrans(const b2Transform& deltaPose, Disturbance& d){
	if (d.getAffIndex()!=NONE){
		math::applyAffineTrans(deltaPose, d.bf.pose);
	}
}


void Configurator::updateGraph(TransitionSystem&g, ExecutionError error){
	b2Transform deltaPose;
	float angularDisplacement= getTask()->getAction().getOmega()*MOTOR_CALLBACK +error.theta();
	float xdistance=cos(angularDisplacement) * getTask()->getAction().getLinearSpeed()*MOTOR_CALLBACK;
	float ydistance=sin(angularDisplacement) * getTask()->getAction().getLinearSpeed()*MOTOR_CALLBACK;
	deltaPose=b2Transform(b2Vec2(xdistance,
					ydistance), 
					b2Rot(angularDisplacement));
	printf("currentVertex = %i, direction =%i\n", currentVertex, currentTask.direction);
	applyAffineTrans(deltaPose, g);
	applyAffineTrans(deltaPose, controlGoal);
}


