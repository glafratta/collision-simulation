#pragma once
#include "configurator.h"
#include <chrono>
#include <iostream>

void Configurator::NewScan(std::vector <Point> & data){ 
	//PREPARE VECTORS TO RECEIVE DATA
	iteration++; //iteration set in getVelocity
	std::vector <Point> previous;
	for (Point p:current){
		previous.push_back(p);
	}
	current.clear();
	for (Point p:data){
		current.push_back(p);
	}
	// previous = current;
	// current.clear();
	// current = data;

	//BENCHMARK + FIND TRUE SAMPLING RATE
	auto now =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>diff= now - previousTimeScan; //in seconds
	timeElapsed=float(diff.count())/1000; //express in seconds
	totalTime += timeElapsed; //for debugging
	previousTimeScan=now; //update the time of sampling

	if (debugOn){
		printf("time elapsed: %f\n", timeElapsed);
	}

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

	
	//CALCULATE VELOCITY 
	
	Configurator::getVelocityResult affineTransResult= GetRealVelocity(current, previous);
	b2Vec2 estimatedVelocity;

	if (affineTransResult.valid){
		estimatedVelocity = affineTransResult.vector;
	}
	else{
		estimatedVelocity = currentDMP.getAction().getLinearVelocity();
		//printf("invalid affine trans result\n");
	}

	//MAKE NOTE OF WHAT STATE WE'RE IN BEFORE RECHECKING FOR COLLISIONS
	bool wasAvoiding = 0;
	bool isSameDMP = 1;
	//UPDATE ABSOLUTE POSITION (SLAM-ISH for checking accuracy of velocity measurements)

	absPosition.x += estimatedVelocity.x*timeElapsed;
	absPosition.y += estimatedVelocity.y*timeElapsed;

	//IF WE  ALREADY ARE IN AN OBSTACLE-AVOIDING STATE, ROUGHLY ESTIMATE WHERE THE OBSTACLE IS NOW
	if (currentDMP.obstacle.isValid()){
		wasAvoiding =1; //remember that the robot was avoiding an obstacle
		//printf("old untracked obstacle position: %f\t%f\n", plan[0].obstacle.getPosition().x, plan[0].obstacle.getPosition().y);
		currentDMP.trackObject(currentDMP.obstacle, timeElapsed, estimatedVelocity, {0.0f, 0.0f}); //robot default position is 0,0
		if (currentDMP.obstacle.getAngle(estimatedVelocity) >= M_PI_2){ 		//if obstacle (pos) and robot (vel) are perpendicular
			currentDMP.obstacle.invalidate();
			currentDMP = desiredDMP;
		}
		else{
			Primitive::Object temp = currentDMP.obstacle;			//otherwise update current state with new obstacle position
			currentDMP = Primitive(temp);
		}

	}

	//MAKE BOX2D BODIES 

	bool isObstacleStillThere=0;
	for (Point &p:current){
		if (p != *(&p-1) && p != *(&p-2)&& p.x >=0 && p.r < BOX2DRANGE){
			b2Body * body;
			b2BodyDef bodyDef;
			b2FixtureDef fixtureDef;
			bodyDef.type = b2_dynamicBody;
			b2PolygonShape fixture; //giving the point the shape of a box
			fixtureDef.shape = &fixture;
			fixture.SetAsBox(.001f, .001f); 

		//CHECK IF BODIES ARE OBSERVED IN THE GENERAL AREA WHERE THE OBSTACLE SHOULD BE 
			if (currentDMP.obstacle.isValid()){
				if (p.isInRadius(currentDMP.obstacle.getPosition())){
					isObstacleStillThere =1;
				}
			}
			if (p.x !=0 && p.y!=0){
				bodyDef.position.Set(p.x, p.y); 
				body = world.CreateBody(&bodyDef);
				body->CreateFixture(&fixtureDef);
			}

		}
	}
	if (!isObstacleStillThere){ 
		currentDMP = desiredDMP;
	}
	int bodyCount = world.GetBodyCount();

	//CREATE ANOTHER PARALLEL PLAN TO COMPARE IT AGAINST THE ONE BEING EXECUTED: currently working on greedy algorithm so local minimum wins
	b2Vec2 start(0.0f, 0.0f);
	float theta=0;



	//CHECK IF WITH THE CURRENT currentDMP THE ROBOT WILL CRASH
	isSameDMP = wasAvoiding == currentDMP.obstacle.isValid();
	Primitive::simResult result;
	currentDMP.setRecordedVelocity(estimatedVelocity);

	//creating decision tree object
	CollisionGraph g;
	vertexDescriptor v0 = boost::add_vertex(g);
	std::vector <vertexDescriptor> leaves;
	edgeDescriptor e;
	Primitive::Direction dir;

	/////////////REACTIVE AVOIDANCE: substitute the currentDMP
	switch (planning){
		case 0:
			printf("reacting\n");
			reactiveAvoidance(world, result, currentDMP, start, theta);
			break;
		case 1:
			currentDMP.change = build_tree(v0, g, currentDMP, world, leaves); //for now should produce the same behaviour because the tree is not being pruned. original build_tree returned bool, now currentDMP.change is changed directly
			e = findBestBranch(g, leaves);
//			dir = g[e].direction;
			if (currentDMP.change){
				printf("planning, change=1");
				//see search algorithms for bidirectional graphs (is this like incorrect bonkerballs are mathematicians going to roast me)
				//FIND BEST OPTION FOR CHANGING
				currentDMP = Primitive(g[v0].obstacle, g[e].direction); //new currentDMP has the obstacle of the previous and the direction of the edge remaining 
			}
			break;
		default: 
		break;

	}
	if (debugOn){
		printf("tree size = %i\n", g.m_vertices.size());
	}
	

	//CHOOSE BEXT NEXT currentDMP BASED ON LOOKING AHEAD OF THE PRESENT OBSTACLE

	printf("new currentDMP wheel speeds: %f, %f\n", currentDMP.getAction().LeftWheelSpeed, currentDMP.getAction().RightWheelSpeed);

	//IF THE currentDMP DIDN'T CHANGE, CORRECT PATH 
	applyController(isSameDMP, currentDMP);


}


void Configurator::applyController(bool isSameDMP, Primitive & dmp){
	if (isSameDMP){
		if (dmp.controller()==Primitive::controlResult::DONE){
			dmp = desiredDMP;
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
				for (cv::Point2f p:previousTmp){
					printf("no data\n");
					return result;
				} 
				}
			else{
				for (int i=0; i<abs(diff); i++){
			currentTmp.push_back(currentTmp[0]);
				if (currentTmp[-1].x == 0 && currentTmp[-1].y ==0){
				}

				}
		}
		}

	//use partial affine transformation to estimate displacement
	cv::Mat transformMatrix = cv::estimateAffinePartial2D(previousTmp, currentTmp, cv::noArray(), cv::LMEDS);
	float theta;
		if (!transformMatrix.empty()){
			b2Vec2 tmp;
			tmp.x= -(transformMatrix.at<double>(0,2))/timeElapsed;
			tmp.y = -(transformMatrix.at<double>(1,2))/timeElapsed;
			float tmpAngle = atan(tmp.y/tmp.x); //atan2 gives results between pi and -pi, atan gives pi/2 to -pi/2
			if (tmp.y ==0 && tmp.x ==0){
				tmpAngle =0;
			}
			if (tmp.Length()>currentDMP.getMaxSpeed()){
				affineTransError += tmp.Length()-currentDMP.getMaxSpeed();
				tmp.x = currentDMP.getAction().getLinearSpeed() *cos(tmpAngle);
				tmp.y = currentDMP.getAction().getLinearSpeed() *sin(tmpAngle);
			}
			return getVelocityResult(tmp);
		}
		else if (transformMatrix.empty()){ //if the plan is empty look at the default wheel speed
			b2Vec2 estimatedVel;
			theta = currentDMP.getAction().getOmega()* timeElapsed;
			estimatedVel ={currentDMP.getAction().getLinearSpeed()*cos(theta),currentDMP.getAction().getLinearSpeed()*sin(theta)};
			result = getVelocityResult(estimatedVel);
			return result;
		}
		else{
			printf("could not find velocity\n");
			return result;
		}
	}



void Configurator::reactiveAvoidance(b2World & world, Primitive::simResult &r, Primitive &s, b2Vec2 & start, float & angle){ //returns true if disturbance needs to be eliminated	
	r =s.willCollide(world, iteration, debugOn, start, angle);
	if (r.resultCode == Primitive::simResult::crashed){
		printf("crashed\n");
		//IF THERE IS NO PLAN OR THE OBJECT WE CRASHED INTO IS NOT ALREADY BEING AVOIDED ADD NEW Primitive TO THE PLAN
		Point p(r.collision.getPosition());
		if ((!s.obstacle.isValid()|| !(p.isInRadius(s.obstacle.getPosition())))){ 
			s = Primitive(r.collision, Primitive::Direction::NONE);
		}			
	}
}


vertexDescriptor Configurator::eliminateDisturbance(vertexDescriptor v, CollisionGraph&g, Primitive  s, b2World & w, std::vector <vertexDescriptor> &_leaves){
	//PREPARE TO LOOK AT BACK EDGES
	edgeDescriptor inEdge;
	vertexDescriptor srcVertex=v; //default
	bool notRoot = boost::in_degree(v, g)>0;
	bool isLeaf=0;
	vertexDescriptor v1 = v; //by default if no vertices need to be added the function returns the startingVertex

		//FIND IF THE PRESENT STATE WILL COLLIDE
	Primitive::simResult result; 

	//IDENTIFY SOURCE NODE, IF ANY
	if (notRoot){
		inEdge = boost::in_edges(v, g).first.dereference();
		srcVertex = boost::source(inEdge, g);
		result =s.willCollide(w, iteration, debugOn, g[srcVertex].endPose.p, g[srcVertex].endPose.q.GetAngle()); //start from where the last Primitive ended (if previous Primitive crashes)
		g[inEdge].distanceCovered= result.distanceCovered; //assign data to edge
		g[v].predecessors = g[srcVertex].predecessors +1;
	}
	else{
		result =s.willCollide(w, iteration, debugOn, {0.0, 0.0}, 0); //default start from 0
	}

	//FILL IN CURRENT NODE WITH ANY COLLISION AND END POSE
	if (result.distanceCovered <=.01){
		g[v].nodesInSameSpot = g[srcVertex].nodesInSameSpot+1; //keep track of how many times the robot is spinning on the spot
	}
	else{
		g[v].nodesInSameSpot =0; //reset if robot is moving
	}
	g[v].obstacle = result.collision;
	g[v].endPose = result.endPose;
	g[v].distanceSoFar = g[srcVertex].distanceSoFar + result.distanceCovered;
	g[v].outcome = result.resultCode;
	//IS THIS NODE LEAF? to be a leaf 1) either the maximum distance has been covered or 2) avoiding an obstacle causes the robot to crash
	bool fl = g[v].distanceSoFar >= BOX2DRANGE;
	bool moreCostlyThanLeaf =0; 

	//ABANDON EARLY IF CURRENT PATH IS MORE COSTLY THAN THE LAST LEAF: if this vertex is the result of more branching while traversing a smaller distance than other leaves, it is more costly
	for (auto l: _leaves){
		if (g[v].distanceSoFar <= g[l].distanceSoFar && (g[v].outcome == g[l].outcome && g[v].predecessors>g[l].predecessors)){
			moreCostlyThanLeaf =1;
		}

	}
	
	//ADD OPTIONS FOR CURRENT ACTIONS BASED ON THE OUTCOME OF THE Primitive/TASK/MOTORPLAN ETC i haven't decided a name yet
	if(!fl&& !moreCostlyThanLeaf){//} && ((v==srcVertex) || (g[srcVertex].endPose !=g[v].endPose))){
		if (result.resultCode != Primitive::simResult::successful){ //accounts for simulation also being safe for now
			if (s.getType()==Primitive::Type::BASELINE){
				Primitive::Direction dir = Primitive::Direction::NONE;
				if (boost::in_degree(srcVertex, g)>0){
					dir = g[boost::in_edges(srcVertex, g).first.dereference()].direction;
				}
				if (result.resultCode == Primitive::simResult::crashed && dir != Primitive::NONE && g[v].nodesInSameSpot<maxNodesOnSpot){
						g[v].options.push_back(dir);
					}
				else if (result.resultCode == Primitive::simResult::safeForNow || boost::in_degree(srcVertex, g)==0){
					Primitive::Action reflex;
					reflex.__init__(result.collision, Primitive::Direction::NONE);
					Primitive::Direction reflexDirection = reflex.getDirection();
					g[v].options.push_back(reflexDirection);// the first branch is the actions generating from a reflex to the collision
					g[v].options.push_back(getOppositeDirection(reflexDirection));
				}
				}
			}
		else { //will only enter if successful
			if (s.getType()==Primitive::Type::AVOID){
				g[v].options.push_back(Primitive::Direction::NONE);
			}
	}	
			//}
	}


	isLeaf = (g[v].options.size() ==0);

	//IF THE Primitive COLLIDES CREATE A PLAN, DEPTH-FIRST
			//DEFINE POSSIBLE NEXT PrimitiveS DEPENDING ON OUTCOME, only if it's not a leaf
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


bool Configurator::build_tree(vertexDescriptor v, CollisionGraph& g, Primitive s, b2World & w, std::vector <vertexDescriptor> &_leaves){
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
		Primitive::Direction d = g[v1InEdge].direction;
		s = Primitive(g[v1Src].obstacle, d);
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