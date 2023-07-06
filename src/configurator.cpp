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
	// for (auto d: current){
	// 	previous.insert(d);
	// }
	previous =current;
	current.clear();
	// for (auto d:data){
	// 	current.insert(d);
	// }
	current = data;
	currentBox2D.clear();
	// for (auto d:data2fp){
	// 	currentBox2D.insert(d);
	// }
	currentBox2D = data2fp;
	printf("updated coordinate vectors\n");

	//BENCHMARK + FIND TRUE SAMPLING RATE
	auto now =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>diff= now - previousTimeScan; //in seconds
	timeElapsed=float(diff.count())/1000; //express in seconds
	totalTime += timeElapsed; //for debugging
	previousTimeScan=now; //update the time of sampling

	if ( timeElapsed< .19){		
		if (timerOff){ //was debugon
			timeElapsed = .2;
		}
	}
	else if (timeElapsed >.21){
		//printf("took too long! %f\n", timeElapsed);
		if (timerOff){
			timeElapsed = .2;
		}
		
	}

//	printf("calculated time elapsed\n");

	//CREATE BOX2D ENVIRONMENT
	b2World world = b2World({0.0f,0.0f});
	char name[256];
	//int bodies=0;
	//printf("made world\n");

	//CALCULATE VELOCITY 
	printf("current = %i\t previous = %i\n", current.size(), previous.size());
	DeltaPose deltaPose= GetRealVelocity(current, previous);
	if (currentTask.direction ==DEFAULT){
		currentTask.action.setOmega(deltaPose.q.GetAngle());
		currentTask.action.setLinearSpeed(SignedVectorLength(deltaPose.p));
	}
	//printf("calculated velocity\n");

	//MAKE NOTE OF WHAT STATE WE'RE IN BEFORE RECHECKING FOR COLLISIONS
	bool wasAvoiding = currentTask.disturbance.isValid();
	bool isSameTask = 1;
	//UPDATE ABSOLUTE POSITION (SLAM-ISH for checking accuracy of velocity measurements)

	//IF WE  ALREADY ARE IN AN OBSTACLE-AVOIDING STATE, ROUGHLY ESTIMATE WHERE THE OBSTACLE IS NOW
	//if (currentTask.disturbance.isValid()){
	//	wasAvoiding =1; //remembesfr that the robot was avoiding an obstacle
	currentTask.trackDisturbance(currentTask.disturbance, timeElapsed, deltaPose.p, b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0))); //robot default position is 0,0
	bool isObstacleStillThere=constructWorldRepresentation(world, currentTask.direction, b2Transform(b2Vec2(0.0, 0.0), b2Rot(0)), &currentTask); 
	bool ended = currentTask.checkEnded();
	if(ended|| !isObstacleStillThere){
		if (!plan.empty()){
			plan.erase(plan.begin());
			currentTask = Task(plan[0].first, plan[0].second);
		}
		else{
			currentTask = desiredTask;
		}
	}
	//}
//	printf("tracked disturbance");

	//MAKE BOX2D BODIES 

	// if (!isObstacleStillThere){ 
	// 	currentTask = desiredTask;
	// }

	//CREATE ANOTHER PARALLEL PLAN TO COMPARE IT AGAINST THE ONE BEING EXECUTED: currently working on greedy algorithm so local minimum wins
	//b2Vec2 start(0.0f, 0.0f);
	//float theta=0;

	//printf("made bodies\n");

	//CHECK IF WITH THE CURRENT currentTask THE ROBOT WILL CRASH
	isSameTask = wasAvoiding == currentTask.disturbance.isValid();
	Task::simResult result;
	//currentTask.setRecordedVelocity(deltaPose.p);

	//creating decision tree Disturbance
	CollisionGraph g;
	vertexDescriptor v0 = boost::add_vertex(g);
	std::vector <vertexDescriptor> leaves;
	//edgeDescriptor e;
	Direction dir;

	//auto startTime =std::chrono::high_resolution_clock::now();
	//printf("set velocity and created empty graph\n");

	/////////////REACTIVE AVOIDANCE: substitute the currentTask
		if (!planning){
			printf("reacting\n");
			reactiveAvoidance(world, result, currentTask);
		}	
		else{
			build_tree(v0, g, currentTask, world, leaves); //for now should produce the same behaviour because the tree is not being pruned. original build_tree returned bool, now currentTask.change is changed directly
			printf("built tree\n");
			vertexDescriptor bestLeaf = findBestLeaf(g, leaves);
			//printf("best leaf = %i\n", bestLeaf);
			Plan debugPlan = getUnprocessedSequence(g, bestLeaf);
			printf("debug plan = ");
			printPlan(debugPlan);
			//removeIdleNodes(g, bestLeaf);
			//plan = getPlan(g, bestLeaf);
			printf("cleaned plan");
			plan = getCleanSequence(g, bestLeaf);
			printf("done plan\n");
			printPlan(plan);
			if (g[v0].outcome == Task::simResult::crashed){ //only change task if outcome is crashed
				//see search algorithms for bidirectional graphs (is this like incorrect bonkerballs are mathematicians going to roast me)
				//FIND BEST OPTION FOR CHANGING
				// if(currentTask.direction ==BACK){
				// printf("will crash with object: %f, %f\n", g[v0].disturbance.position.x, g[v0].disturbance.position.y);
				// }
				// if (g.m_vertices.size()>1){
				// 	dir =g[e].direction;
				// 	//printf("bestdirection = %i\n", dir);
				// 	currentTask = Task(g[v0].disturbance, dir); //new currentTask has the obstacle of the previous and the direction of the edge remaining 
				// }
				// else{ //FALLBACK, ensure it's still operating even if tree building fails
				//  	currentTask = Task(g[v0].disturbance, Direction::STOP); //was stop
				//  	printf("I am stuck!\n");
				// }
				if (!plan.empty()){
					currentTask = Task(plan[0].first, plan[0].second);
				}
			}
		}
	printf("tree size = %i\n", g.m_vertices.size());
	// if (debugOn){
	// 	auto endTime =std::chrono::high_resolution_clock::now();
	// 	std::chrono::duration<float, std::milli>d= startTime- endTime; //in seconds
	// 	float duration=float(d.count())/1000; //express in seconds
	// 	FILE * f = fopen(statFile, "a+");
	// 	fprintf(f,"%i\t%i\t%f\n", bodies, g.m_vertices.size(), duration);
	// 	fclose(f);
	// }
	bodies =0;
	//CHOOSE BEXT NEXT Task BASED ON LOOKING AHEAD OF THE PRESENT OBSTACLE

	//printf("new Task wheel speeds: L= %f, R=%f\n", currentTask.getAction().L, currentTask.getAction().R);

	//IF THE TASK DIDN'T CHANGE, CORRECT PATH 
	applyController(isSameTask, currentTask);
	printf("applied controller\n");

	//graph should be saved and can check, if plan actually executed successfully, the probability to transition to that state increases. Read on belief update

}


void Configurator::applyController(bool isSameTask, Task & task){
	if (isSameTask){
		if (task.controller()==Task::controlResult::DONE){
			task = desiredTask;
		}
	}
}

DeltaPose Configurator::GetRealVelocity(CoordinateContainer &_current, CoordinateContainer &_previous){	 //does not modify current vector, creates copy	
		DeltaPose result;

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
		else if (diff<0){
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
	//use partial affine transformation to estimate displacement
	cv::Mat transformMatrix =cv::estimateAffinePartial2D(previousTmp, currentTmp, cv::noArray(), cv::LMEDS);
	float theta;
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
	else if (transformMatrix.empty()){ //if the plan is empty look at the default wheel speed
		theta = currentTask.getAction().getOmega()* timeElapsed;
		result.p ={currentTask.getAction().getLinearSpeed()*cos(theta),currentTask.getAction().getLinearSpeed()*sin(theta)};
		result.q.Set(currentTask.getAction().getOmega());
	}
	else{
		printf("could not find velocity\n");
	}	
	return result;
	}



void Configurator::reactiveAvoidance(b2World & world, Task::simResult &r, Task &s){ //returns true if disturbance needs to be eliminated	
	r =s.willCollide(world, iteration, debugOn, SIM_DURATION);
	if (r.resultCode == Task::simResult::crashed){
		printf("crashed\n");
		//IF THERE IS NO PLAN OR THE Disturbance WE CRASHED INTO IS NOT ALREADY BEING AVOIDED ADD NEW Task TO THE PLAN
		Point p(r.collision.getPosition());
		if ((!s.disturbance.isValid()|| !(p.isInRadius(s.disturbance.getPosition())))){ 
			s = Task(r.collision, Direction::DEFAULT);
		}			
	}
}


vertexDescriptor Configurator::nextNode(vertexDescriptor v, CollisionGraph&g, Task  s, b2World & w, std::vector <vertexDescriptor> &_leaves){
	//PREPARE TO LOOK AT BACK EDGES
	edgeDescriptor inEdge;
	vertexDescriptor srcVertex=v; //default
	bool notRoot = boost::in_degree(v, g)>0;
	bool isLeaf=0;
	vertexDescriptor v1 = v; //by default if no vertices need to be added the function returns the startingVertex

		//FIND IF THE PRESENT STATE WILL COLLIDE
	Task::simResult result; 
	float remaining=SIM_DURATION;
	//IDENTIFY SOURCE NODE, IF ANY
	if (notRoot){
		inEdge = boost::in_edges(v, g).first.dereference();
		srcVertex = boost::source(inEdge, g);
		//find remaining distance to calculate
		if(g[inEdge].direction == Direction::DEFAULT){
			remaining= (BOX2DRANGE-g[srcVertex].distanceSoFar)*2/MAX_SPEED;
		} 
		if (remaining<0){
			remaining=0;
		}
		result =s.willCollide(w, iteration, debugOn, remaining); //start from where the last Task ended (if previous Task crashes)
		g[inEdge].distanceCovered= result.distanceCovered; //assign data to edge
		g[v].predecessors = g[srcVertex].predecessors +1;
	}
	else{
		result =s.willCollide(w, iteration, debugOn, remaining); //default start from 0
	}


	//FILL IN CURRENT NODE WITH ANY COLLISION AND END POSE
	if (result.distanceCovered <=.01){
		g[v].nodesInSameSpot = g[srcVertex].nodesInSameSpot+1; //keep track of how many times the robot is spinning on the spot
	}
	else{
		g[v].nodesInSameSpot =0; //reset if robot is moving
	}
	if (result.collision.isValid()){
		g[v].totDs++;
	}
	g[v].disturbance = result.collision;
	g[v].endPose = result.endPose;
	g[v].distanceSoFar = g[srcVertex].distanceSoFar + (round(result.distanceCovered*100))/100; //rounding to 2 decimals to eliminate floating point errors
	g[v].outcome = result.resultCode;
	//g[v].step = result.step;
	//IS THIS NODE LEAF? to be a leaf 1) either the maximum distance has been covered or 2) avoiding an obstacle causes the robot to crash
	bool fl = g[v].distanceSoFar >= BOX2DRANGE; //full length
	bool fullMemory = g[v].totDs >=4;
	bool growBranch =1; 

	//ABANDON EARLY IF CURRENT PATH IS MORE COSTLY THAN THE LAST LEAF: if this vertex is the result of more branching while traversing a smaller distance than other leaves, it is more costly
	for (auto l: _leaves){
		if (g[v].distanceSoFar <= g[l].distanceSoFar && (g[v].outcome == g[l].outcome && g[v].totDs>g[l].totDs)){
			growBranch =0;
		}

	}
	//ADD OPTIONS FOR CURRENT ACTIONS BASED ON THE OUTCOME OF THE Task/TASK/MOTORPLAN ETC i haven't decided a name yet
	if(!fl&& growBranch && !fullMemory){//} && ((v==srcVertex) || (g[srcVertex].endPose !=g[v].endPose))){
	if (result.resultCode != Task::simResult::successful){ //accounts for simulation also being safe for now
			if (s.getAffIndex()==int(InnateAffordances::NONE)){
				if (g[v].nodesInSameSpot<maxNodesOnSpot){
					for (Direction d :Avoid.options){
						g[v].options.push_back(d);
					}
				}
			}
		}
		else { //will only enter if successful
		if (s.getAffIndex()==int(InnateAffordances::AVOID)){
			if (s.direction == LEFT || s.direction == RIGHT){
				g[v].options.push_back(DEFAULT);
			}
			else if (s.direction == BACK){
				for (Direction d: Avoid.options){
					if (d !=BACK){
						g[v].options.push_back(d);
					}
				}
			}
		}
		}	
	}
//	}


	isLeaf = (g[v].options.size() ==0);

	//IF THE Task COLLIDES CREATE A PLAN, DEPTH-FIRST
			//DEFINE POSSIBLE NEXT TaskS DEPENDING ON OUTCOME, only if it's not a leaf
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
	int bodyCount=0;
	sprintf(n, "/tmp/bodies%04i.txt", iteration);
	//PRINT DEBUG
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
	if (debugOn){
		printf("planfile = robot%04i.txt\n", iteration);
	}
	//END DEBUG FILE
	vertexDescriptor v1 = nextNode(v, g,s,w, _leaves); 
	bodyCount += w.GetBodyCount();
	treeSize= g.m_vertices.size();
		//destroying world causes segfault even if it's no longer required so skipping for now
    while (v1!= v){
		b2World newWorld({0.0f, 0.0f});
		edgeDescriptor v1InEdge = boost::in_edges(v1, g).first.dereference();
		vertexDescriptor v1Src = v1InEdge.m_source;
		Direction dir = g[v1InEdge].direction;
		s = Task(g[v1Src].disturbance, dir, g[v1Src].endPose);
		constructWorldRepresentation(newWorld, dir, g[v1Src].endPose); //was g[v].endPose
		bodyCount =newWorld.GetBodyCount();
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
		v1 = nextNode(v,g,s, newWorld, _leaves);
		treeSize= g.m_vertices.size();
	}
	//return !g[0].disturbance.safeForNow;
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


vertexDescriptor Configurator::findBestLeaf(CollisionGraph &g, std::vector <vertexDescriptor> _leaves){
	//FIND BEST LEAF
	vertexDescriptor best = _leaves[0];
	for (vertexDescriptor leaf: _leaves){
		if (g[leaf].distanceSoFar>g[best].distanceSoFar){
			best = leaf;
		}
		else if (g[leaf].distanceSoFar==g[best].distanceSoFar){
			if (g[leaf].totDs< g[best].totDs){ //the fact that this leaf has fewer predecessors implies fewer collisions
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
			case BACK: printf("RIGHT"); break;
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
				//printf("ci ready = %i, c->ci->data == c->current = %i\n", c->ci->isReady(), c->ci->data == c->current );

				c->Spawner(c->ci->data, c->ci->data2fp);
			}
			// else if (c->ci->data.empty()){
			// 	c->currentTask = c->desiredTask;
			// }

		}
	}
}

void Configurator::addOptionsToNode(CollisionGraph & g, vertexDescriptor v){}