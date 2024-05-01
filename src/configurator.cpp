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
	// bool wasAvoiding = currentTask.disturbance.isValid();
	// bool isSameTask = 1;
	//UPDATE ABSOLUTE POSITION (SLAM-ISH for checking accuracy of velocity measurements)

	//IF WE  ALREADY ARE IN AN OBSTACLE-AVOIDING STATE, ROUGHLY ESTIMATE WHERE THE OBSTACLE IS NOW
	worldBuilder.buildWorld(world, currentBox2D, currentTask.start, currentTask.direction, &currentTask).first;
	if (controlGoal.change){
		currentTask=Task(Disturbance(), STOP);
		running=0;
		return 0;
	}

	//CHECK IF WITH THE CURRENT currentTask THE ROBOT WILL CRASH
	//isSameTask = wasAvoiding == currentTask.disturbance.isValid();
	simResult result;
	collisionGraph.clear();
	//creating decision tree Disturbance
	vertexDescriptor v0 = boost::add_vertex(collisionGraph);
	//std::vector <vertexDescriptor> leaves;
	//Direction dir;

	auto startTime =std::chrono::high_resolution_clock::now();
	vertexDescriptor bestLeaf = v0;
	int ogStep=collisionGraph[v0].step;
	if (planning & (plan.empty())){ 
		currentTask.change=1;
		collisionGraph[v0].filled =1;
		collisionGraph[v0].disturbance = controlGoal.disturbance;
		collisionGraph[v0].outcome = simResult::successful;
		classicalAStar(v0, collisionGraph, currentTask, world, bestLeaf);
		plan = getCleanSequence(collisionGraph, bestLeaf);
		printPlan(plan);
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
		fprintf(f,"%i\t%i\t%f\n", worldBuilder.getBodies(), collisionGraph.m_vertices.size(), duration);
		fclose(f);
		printf("HZ=%f\n", HZ);
	}
	worldBuilder.resetBodies();
	//CHOOSE BEXT NEXT Task BASED ON LOOKING AHEAD OF THE PRESENT OBSTACLE
	if (!plan.empty()){
		ogStep = plan[0].step;
	}
	//IF THE TASK DIDN'T CHANGE, CORRECT PATH
	//if (isSameTask){
		//currentTask.controller(timeElapsed, ogStep-currentTask.motorStep);
	//}

	//graph should be saved and can check, if plan actually executed successfully, the probability to transition to that state increases. Read on belief update
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
	r =s.willCollide(world, iteration, debugOn, SIM_DURATION, simulationStep);
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
	float remaining = BOX2DRANGE/controlGoal.action.getLinearSpeed();
	//IDENTIFY SOURCE NODE, IF ANY
	if (notRoot){
		inEdge = boost::in_edges(v, g).first.dereference();
		srcVertex = boost::source(inEdge, g);
		//find remaining distance to calculate
		if(g[inEdge].direction == Direction::DEFAULT){
		remaining= (BOX2DRANGE-fabs(g[srcVertex].endPose.p.y))/controlGoal.getAction().getLinearSpeed();
		}
		if (remaining<0.01){
			remaining=0;
		}
	}
	result =s.willCollide(w, iteration, debugOn, remaining, simulationStep); //default start from 0
	//FILL IN CURRENT NODE WITH ANY COLLISION AND END POSE
	if (result.distanceCovered <=.01){ //CYCLE PREVENTING HEURISTICS
		g[v].nodesInSameSpot = g[srcVertex].nodesInSameSpot+1; //keep track of how many times the robot is spinning on the spot
	}
	else{
		g[v].nodesInSameSpot =0; //reset if robot is moving
	}
	//SET ORIENTATION OF POINT RELATED TO ITS NEIGHBOURS
	g[v].fill(result);
	return result;
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
			TaskSummary ts(g[src].disturbance, g[e].direction, g[leaf].step);
			p.insert(p.begin(), ts);
		}
		leaf = src; //go back
		}
	}
	p.insert(p.begin(), TaskSummary(g[0].disturbance, Direction::STOP, 0));
	return p;

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
			bool topDown=1;
			s = Task(g[v1].disturbance, g[e].direction, g[v0].endPose, 1);
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
		TaskSummary ts(g[best].disturbance, g[e].direction, g[best].step);
		p.insert(p.begin(), ts);

	}
	return p;
}

void Configurator::printPlan(Sequence p){
	for (TaskSummary ts: p){
		switch (ts.direction){
			case DEFAULT: printf("DEFAULT: %i", ts.step);break;
			case LEFT: printf("LEFT: %i", ts.step); break;
			case RIGHT: printf("RIGHT: %i", ts.step); break;
			case BACK: printf("BACK: %i", ts.step); break;
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
			c->ci->ts = TaskSummary(c->currentTask.disturbance, c->currentTask.direction, c->currentTask.motorStep);
		}
	}

}


void Configurator::transitionMatrix(CollisionGraph&g, vertexDescriptor vd, Direction d){
	Task temp(controlGoal.disturbance, DEFAULT, g[vd].endPose); //reflex to disturbance
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


void Configurator::addToPriorityQueue(CollisionGraph& g, vertexDescriptor v, std::vector <vertexDescriptor>& queue){
	for (auto i =queue.begin(); i!=queue.end(); i++){
		if (abs(g[*i].evaluationFunction()) >abs(g[v].evaluationFunction())){
			queue.insert(i, v);
			return;
		}
	}
	queue.push_back(v);
}



void Configurator::checkDisturbance(Point p, bool& obStillThere, Task * curr){
	if (NULL!=curr){ //
		if (p.isInRadius(curr->disturbance.getPosition())){
			obStillThere =1;
		}
	}
}



void Configurator::trackTaskExecution(Task & t){
		if (t.motorStep>0){
			t.motorStep--;
			printf("step =%i\n", t.motorStep);
		}
		if(t.motorStep==0){
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


void Configurator::changeTask(bool b, Sequence & p, Node n, int&ogStep){
	if (!b){
		return;
	}
	if (!p.empty()){
		if (p.size()==1){
			running=0;
		}
		p.erase(p.begin());
	}
	if (planning){
		if (plan.empty()){
			return;
		}
		currentTask = Task(p[0].disturbance, p[0].direction);
		currentTask.motorStep = p[0].step;
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
		currentTask.motorStep = motorStep(currentTask.getAction()); //reflex
	}
	ogStep = currentTask.motorStep;
}

