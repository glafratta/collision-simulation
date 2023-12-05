#pragma once
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
	CoordinateContainer previous =current;
	current.clear();
	current = CoordinateContainer(data);
	currentBox2D.clear();
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
	totalTime += timeElapsed; //for debugging
	previousTimeScan=now; //update the time of sampling

	if (timerOff){
		timeElapsed = .2;
	}


	//CREATE BOX2D ENVIRONMENT
	b2Vec2 gravity = {0.0, 0.0};
	b2World world= b2World(gravity);
	char name[256];

	//CALCULATE VELOCITY 
	DeltaPose deltaPose;
	 if (currentTask.action.getOmega()==0){
	 	deltaPose= GetRealVelocity(current, previous); //closed loop, sensor feedback for velocity
	 }
	else{
		deltaPose = assignDeltaPose(currentTask.getAction(), timeElapsed); //open loop
	}
		currentTask.action.setRecSpeed(SignedVectorLength(deltaPose.p));
		currentTask.action.setRecOmega(deltaPose.q.GetAngle());

	//MAKE NOTE OF WHAT STATE WE'RE IN BEFORE RECHECKING FOR COLLISIONS
	bool wasAvoiding = currentTask.disturbance.isValid();
	bool isSameTask = 1;
	//UPDATE ABSOLUTE POSITION (SLAM-ISH for checking accuracy of velocity measurements)

	//IF WE  ALREADY ARE IN AN OBSTACLE-AVOIDING STATE, ROUGHLY ESTIMATE WHERE THE OBSTACLE IS NOW
	bool isObstacleStillThrere = worldBuilder.buildWorld(world, currentBox2D, currentTask.start, currentTask.direction, &currentTask).first;
	if (controlGoal.change){
		currentTask=Task(Disturbance(), STOP);
		return 0;
	}

	//CHECK IF WITH THE CURRENT currentTask THE ROBOT WILL CRASH
	isSameTask = wasAvoiding == currentTask.disturbance.isValid();
	simResult result;
	collisionGraph.clear();
	//creating decision tree Disturbance
	vertexDescriptor v0 = boost::add_vertex(collisionGraph);
	std::vector <vertexDescriptor> leaves;
	Direction dir;

	auto startTime =std::chrono::high_resolution_clock::now();
	vertexDescriptor bestLeaf = v0;
	if (planning & ( planBuild!=STATIC || plan.empty())){ //og. collisionGraph[v0].outcome !=simResult::successful || 
		collisionGraph[v0].filled =1;
		collisionGraph[v0].disturbance = controlGoal.disturbance;
		collisionGraph[v0].outcome = simResult::successful;
		if (graphConstruction ==A_STAR){
			classicalAStar(v0, collisionGraph, currentTask, world, bestLeaf);
		}
		plan = getCleanSequence(collisionGraph, bestLeaf);
		currentTask.change=1;

	}
	else if (!planning){
		result = evaluateNode(v0,collisionGraph, currentTask, world);
		currentTask.change = collisionGraph[v0].outcome==simResult::crashed;
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
		return 0; //stops when finished and doesn't execute

	}
	worldBuilder.resetBodies();
	//CHOOSE BEXT NEXT Task BASED ON LOOKING AHEAD OF THE PRESENT OBSTACLE

	//IF THE TASK DIDN'T CHANGE, CORRECT PATH 
	if (isSameTask){
		currentTask.controller();
	}

	//graph should be saved and can check, if plan actually executed successfully, the probability to transition to that state increases. Read on belief update
	return 1;
}


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
					return result;
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
		remaining =fabs(M_PI_2/s.getAction().getOmega());
	}
	//IDENTIFY SOURCE NODE, IF ANY
	if (notRoot){
		inEdge = boost::in_edges(v, g).first.dereference();
		srcVertex = boost::source(inEdge, g);
		//find remaining distance to calculate
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
	result.collision.setOrientation(atan(result.endPose.q.c/result.endPose.q.s)); //90 deg turn
	g[v].fill(result);	
	return result;
	}


void Configurator::classicalAStar(vertexDescriptor v, CollisionGraph& g, Task s, b2World & w, vertexDescriptor & bestNext){
	vertexDescriptor v1, v0;
	float error;
	bool added;
	Direction direction = s.direction;
	std::vector <vertexDescriptor> priorityQueue = {v};	
	do{	
		v=bestNext;
		priorityQueue.erase(priorityQueue.begin());
		if (!(g[v].filled)){ //for the first vertex
			evaluateNode(v, g, s, w);			
		}
		EndedResult er = findError(v, g, direction);
		applyTransitionMatrix(g, v, direction, er.ended);
		for (Direction d: g[v].options){ //add and evaluate all vertices
			v0=v;
			v1 =v0;
			do {
			added =addVertex(v0, v1, g, g[v0].disturbance); //add
			edgeDescriptor e = boost::in_edges(v1, g).first.dereference();
			s = Task(g[v0].disturbance, g[e].direction, g[v0].endPose);
			worldBuilder.buildWorld(w, currentBox2D, s.start, g[e].direction); //was g[v].endPose
			evaluateNode(v1, g, s, w); //find simulation result
			bool end = controlGoal.checkEnded(g[v1]).ended;
			applyTransitionMatrix(g, v1, g[e].direction, end);
			v0=v1;
			}while(s.direction !=DEFAULT & added);
			g[v1].error = findError(v1, g, s.direction).errorFloat;
			addToPriorityQueue(g,v1, priorityQueue);
		}
		bestNext=priorityQueue[0];
		direction = g[boost::in_edges(bestNext, g).first.dereference()].direction;
	}while(g[bestNext].options.size()!=0);
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
			Task::Action a;
			a.init(g[e].direction);
			float step = motorStep(a);
			TaskSummary ts(g[src].disturbance, g[e].direction, step);
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
		Task::Action a;
		a.init(g[e].direction);
		float step = motorStep(a);
		TaskSummary ts(g[src].disturbance, g[e].direction, step);
		p.insert(p.begin(), ts);	
		leaf = src; //go back
		}
	}
	return p;
	
}


vertexDescriptor Configurator::findBestLeaf(CollisionGraph &g, std::vector <vertexDescriptor> _leaves, vertexDescriptor v, EndCriteria * refEnd){
	if (_leaves.empty()){
		return v;
	}
	vertexDescriptor best = _leaves[0];
	if (refEnd==NULL){
		refEnd = &controlGoal.endCriteria;
	}
	for (vertexDescriptor leaf: _leaves){
			if (abs(g[leaf].evaluationFunction())<abs(g[best].evaluationFunction())){
				best=leaf;
				g[best].error= g[leaf].error;
				g[best].cost = g[leaf].cost;
			}
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
	Sequence p;
	edgeDescriptor e;
	while (boost::in_degree(best, g)){
		best = e.m_source;
		Task::Action a;
		a.init(g[e].direction);
		float step = motorStep(a);
		TaskSummary ts(g[best].disturbance, g[e].direction, step);
		p.insert(p.begin(), ts);

	}
	return p;
}

void Configurator::printPlan(Sequence p){
	for (TaskSummary ts: p){
		switch (ts.direction){
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
	ci->ts = TaskSummary(controlGoal.disturbance, controlGoal.direction, motorStep(controlGoal.action));
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
			c->ci->ts = TaskSummary(c->currentTask.disturbance, c->currentTask.direction, c->currentTask.step);
		}
	}

}


void Configurator::transitionMatrix(CollisionGraph&g, vertexDescriptor vd, Direction d){
	Task temp(controlGoal.disturbance, DEFAULT, g[vd].endPose); //reflex to disturbance
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
							if (graphConstruction != E){
								g[vd].options.push_back(temp.direction);
								g[vd].options.push_back(getOppositeDirection(temp.direction).second);
							}
							g[vd].options.push_back(DEFAULT);
						}
						else{
							if (graphConstruction != E){
								g[vd].options = {DEFAULT, LEFT, RIGHT};
							}
							else{
								g[vd].options ={DEFAULT};
							}
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
	else if(round(g[vd].endPose.p.Length()*100)/100>=BOX2DRANGE){ // OR g[vd].totDs>4 
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
	while (boost::in_degree(src, g)>0 & g[v].disturbance.isValid()){ //&has two step
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
				result.first=true; //is there a neighbouring point?
				count++;
				sumY+=deltaY;
				sumX+=deltaX;
				avgY = sumY/count;
				avgX = sumX/count;
		}
	}
	result.second=atan(avgY/avgX); 
	return result;
}


void Configurator::checkDisturbance(Point p, bool& obStillThere, Task * curr){
	if (NULL!=curr){ //
		if (p.isInRadius(curr->disturbance.getPosition())){
			obStillThere =1;
		}
	}
}

std::pair <bool, int>  Configurator::checkPlan(b2World& world, Sequence & seq, Task t, b2Transform start){
	std::pair <bool, int> result(0, -1); //1 is fials, 0 is ok
	for (TaskSummary ts: seq){
		CoordinateContainer dCloud;
		result.second++;
		t = Task(ts.disturbance, ts.direction, start);
		std::pair <bool, b2Vec2> dData = worldBuilder.buildWorld(world, currentBox2D, start, ts.direction, &t, &dCloud);
		simResult sim = t.willCollide(world, iteration); //check if plan is successful
		start = sim.endPose;
		b2Vec2 differenceVector = ts.disturbance.pose.p - dData.second;
		ts.disturbance.pose.p = dData.second;
		int additionalSteps = SignedVectorLength(differenceVector);
		ts.step +=additionalSteps;
		if (sim.resultCode != simResult::successful){
			result.first = 1;
			break;
		}
	}
	return result;
}


void Configurator::trackTaskExecution(Task & t){
		if (t.step>0){
			t.step--;
		}
		if(t.step==0){
			t.change=1;
		}
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
	return result;
}

int Configurator::motorStep(Task::Action a){
	int result=0;
        if (a.getOmega()>0){ //LEFT
            result = SAFE_ANGLE/(MOTOR_CALLBACK * a.getOmega());
		    result *=FRICTION_DAMPENING;
        }
		else if (a.getOmega()<0){ //RIGHT
            result = SAFE_ANGLE/(MOTOR_CALLBACK * a.getOmega());
		    result *=FRICTION_DAMPENING;
		}
		else if (a.getLinearSpeed()>0){
			result = simulationStep/(MOTOR_CALLBACK*a.getLinearSpeed())*FRICTION_DAMPENING;
		}
	    return abs(result);
    }


void Configurator::changeTask(bool b, Sequence & p, Node n, int&ogStep){
	if (!b){
		return;
	}
	if (planning){
		if (plan.empty()){
			return;
		}
		currentTask = Task(p[0].disturbance, p[0].direction);
		currentTask.step = p[0].step;
		p.erase(p.begin());
	}
	else{
		if (n.disturbance.isValid()){
			currentTask = Task(n.disturbance, DEFAULT); //reactive
		}
		else if(currentTask.direction!=DEFAULT){
				currentTask = Task(n.disturbance, DEFAULT); //reactive
		}
		else{
			currentTask = Task(controlGoal.disturbance, DEFAULT); //reactive
		}
		
		currentTask.step = motorStep(currentTask.getAction());
	}
	ogStep = currentTask.step;
}

