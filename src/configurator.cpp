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
	//printf("data = %i\n", current.size());
	//printf("current length: %i, previous length%i\n", current.size(), previous.size());

	//BENCHMARK + FIND TRUE SAMPLING RATE
	auto now =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>diff= now - previousTimeScan; //in seconds
	timeElapsed=float(diff.count())/1000; //express in seconds
	//printf("time elapsed between newscans = %4f ms\n", timeElapsed);
	totalTime += timeElapsed; //for debugging
	previousTimeScan=now; //update the time of sampling

	//timeElapsed = samplingRate;
	//DISCARD BAD SCANS
	if ( timeElapsed< .15){
		//timeElapsed= samplingRate;
		//return;
	}
	else if (timeElapsed >.25){
		printf("took too long! %f\n", timeElapsed);
		//return;
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
	//printf("estimatedVelocity = %f, %f\n", estimatedVelocity.x, estimatedVelocity.y);

	//MAKE NOTE OF WHAT STATE WE'RE IN BEFORE RECHECKING FOR COLLISIONS
	bool wasAvoiding = 0;
	bool isSameDMP = 1;
	//UPDATE ABSOLUTE POSITION (SLAM-ISH for checking accuracy of velocity measurements)

	absPosition.x += estimatedVelocity.x*timeElapsed;
	absPosition.y += estimatedVelocity.y*timeElapsed;
	//dumpPath = fopen("/tmp/dumpPath.txt", "a+");
	//fprintf(dumpPath, "%f\t%f\n", absPosition.x, absPosition.y);
	//fclose(dumpPath);

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
		if (p != *(&p-1)&& p.x >=0 && p.r < currentDMP.box2dRange){
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
	//printf("bodies = %i\n", world.GetBodyCount());
	//printf("is obstacle still there = %i\n", isObstacleStillThere);
	if (!isObstacleStillThere){ 
		currentDMP = desiredDMP;
	}

	//CREATE ANOTHER PARALLEL PLAN TO COMPARE IT AGAINST THE ONE BEING EXECUTED: currently working on greedy algorithm so local minimum wins
	// Tree decisionTree;
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
	//tree[v0]= currentDMP.willCollide(world, iteration, )

	/////////////REACTIVE AVOIDANCE: substitute the currentDMP
	switch (planning){
		case 0:
			printf("reacting\n");
			reactiveAvoidance(world, result, currentDMP, start, theta);
		case 1:
			//printf("planning on\n");
			//currentDMP.change =build_tree(v, g, world); //for now should produce the same behaviour because the tree is not being pruned
			currentDMP.change = build_tree(v0, g, currentDMP, world, leaves); //for now should produce the same behaviour because the tree is not being pruned. original build_tree returned bool, now currentDMP.change is changed directly
			if (currentDMP.change){
				//see search algorithms for bidirectional graphs (is this like incorrect bonkerballs are mathematicians going to roast me)
				//FIND BEST OPTION FOR CHANGING
				//auto e = boost::out_edges(v0, tree).first.dereference();
				printf("task tree size = %i now finding best branch\n", g.m_vertices.size());
				edgeDescriptor e = findBestBranch(g, leaves);
				Primitive::Direction dir = g[e].direction;
				//printf("new currentDMP from v %i\n", v0);
				currentDMP = Primitive(g[v0].obstacle, dir); //new currentDMP has the obstacle of the previous and the direction of the edge remaining 
				//no pruning yet, no choosing optimal action
			}
			break;
		default: break;

	}
	

	//CHOOSE BEXT NEXT currentDMP BASED ON LOOKING AHEAD OF THE PRESENT OBSTACLE

	printf("new currentDMP wheel speeds: %f, %f\n", currentDMP.getAction().LeftWheelSpeed, currentDMP.getAction().RightWheelSpeed);

	//IF THE currentDMP DIDN'T CHANGE, CORRECT PATH 
	//printf("was avoiding? %i\tis same currentDMP? %i\n", wasAvoiding, isSameDMP);
	applyController(isSameDMP, currentDMP);
	//printf("plan size (end of newscan) = %i, iteration %i\n", plan.size(), iteration);


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
			float tmpPi = tmpAngle/M_PI;
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


// void Configurator::eliminateDisturbance(b2World& world, Primitive & s, b2Vec2 &start, float& angle, Primitive::Direction d = Primitive::Direction::NONE){ //STRATEGY AVOIDING JUST ONE OBSTACLE IN FRONT, REACTIVE, NO PLANNING
// 	//Primitive::simResult result;
// 	Primitive::simResult result =s.willCollide(world, iteration, start, angle);
// 	// start = currentDMP.endPose.p;
// 	// theta = currentDMP.endPose.q.GetAngle();
// 	if (result.resultCode == Primitive::simResult::crashed){
// 		printf("crashed\n");
// 		//IF THERE IS NO PLAN OR THE OBJECT WE CRASHED INTO IS NOT ALREADY BEING AVOIDED ADD NEW currentDMP TO THE PLAN
// 		Point p(result.collision.getPosition());
// 		if (!s.obstacle.isValid() || !(p.isInRadius(s.obstacle.getPosition()))){
// 			//BUILD TREE HERE?
// 			CollisionGraph tree;
// 			s = Primitive(result.collision, d);
// 			printf("new currentDMP wheel speeds: %f, %f\n", currentDMP.getAction().LeftWheelSpeed, currentDMP.getAction().RightWheelSpeed);
// 		}			
// 	}

// }

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

// vertexDescriptor Configurator::eliminateDisturbance(b2World & world, vertexDescriptor v, Graph &g){
// 	//vertexDescriptor ogV = v;
// 	//PREPARE TO LOOK AT BACK EDGES
// 	edgeDescriptor inEdge;
// 	vertexDescriptor srcVertex=v; //default
// 	bool notRoot = boost::in_degree(v, g)>0;
// 	//printf("degree = %i, is it >0, %i\n", boost::in_degree(v, g), boost::in_degree(v, g) >0);
// 	bool isLeaf=0;
// 	vertexDescriptor v1 = v; //by default if no vertices need to be added the function returns the startingVertex

// 		//FIND IF THE PRESENT Primitive WILL COLLIDE
// 	//printf("add to vertex %i, ", v);
// 	Primitive::simResult result; 

// 	//IDENTIFY SOURCE NODE, IF ANY
// 	if (notRoot){
// 		//printf("is not root\n");
// 		inEdge = boost::in_edges(v, g).first.dereference();
// 		srcVertex = boost::source(inEdge, g);
// 		result =g[v].willCollide(world, iteration, g[srcVertex].endPose.p, g[srcVertex].endPose.q.GetAngle()); //start from where the last Primitive ended (if previous Primitive crashes)
// 		g[inEdge]= result; //assign data to edge
// 	}

// 	else{
// 		result =g[v].willCollide(world, iteration, {0.0, 0.0}, 0); //default start from 0
// 	}

// 	//IS THIS NODE LEAF? to be a leaf 1) either the maximum distance has been covered or 2) avoiding an obstacle causes the robot to crash
// 	bool fl = isFullLength(v,g);
// 	if(!fl){
// 		switch (result.resultCode){
// 				case Primitive::simResult::resultType::crashed:
// 					if (g[v].getType()==Primitive::Type::BASELINE){
// 						Primitive::Action reflex;
// 						reflex.__init__(result.collision, Primitive::Direction::NONE);
// 						Primitive::Direction reflexDirection = reflex.getDirection();
// 						g[v].options.push_back(reflexDirection);// the first branch is the actions generating from a reflex to the collision
// 						g[v].options.push_back(getOppositeDirection(reflexDirection));
// 						//g[v].change=1;
// 					}
// 					break;
// 				case Primitive::simResult::resultType::successful:
// 					if (g[v].getType()==Primitive::Type::AVOID){
// 						g[v].options.push_back(Primitive::Direction::NONE);
// 					}
// 					break;
// 				default: break;
// 			}
// 	}

// 	// for (Primitive::Direction d:g[v].options){
// 	// 	printf("action %i\n", static_cast<int>(d));
// 	// }


// 	isLeaf = fl ||(g[v].options.size() <=0);
// //	printf("is leaf? %i full length = %i, options = %i\n", isLeaf, fl, g[v].options.size());

// 	//IF THE Primitive COLLIDES CREATE A PLAN, DEPTH-FIRST
// 			//DEFINE POSSIBLE NEXT PrimitiveS DEPENDING ON OUTCOME, only if it's not a leaf
// 	if (g[v].options.size()>0){
// 			addVertex(v, v1, g, result.collision);
// 	}
// 	//IF NO VERTICES CAN BE ADDED TO THE CURRENT BRANCH, CHECK THE CLOSEST BRANCH
// 	else {
// 		//printf("seeing if can go back\n");
// 		Primitive::Object previousCollision;
//                 while (g[v].options.size()<=0){ //keep going back until it finds an incomplete node
// 					// if (boost::in_degree(v,g)>0){
// 					// }
// 					//printf("is in edge surce = target? %i\n", inEdge.m_source == inEdge.m_target);
//                     if(boost::in_degree(v, g)>0){
// 	                    inEdge = boost::in_edges(v, g).first.dereference();
// 						previousCollision = g[v].obstacle;
// 						//printf("collision from vertex %i, pos %f, %f, valid = %i\n", v, previousCollision.getPosition().x, previousCollision.getPosition().y, previousCollision.isValid());
// 						v = source(inEdge, g); //go back a node
// 					    //printf("new src = %i, in degree = %i\n", v, boost::in_degree(v, g));
//                     }
//                     else{
//                         //printf("source has no back edge\n");
//                         break;
//                     }
//                 }
// 				if (g[v].options.size()>0){ //if if the vertex exiting the while loop is incomplete add a new node
// 					addVertex(v,v1,g, previousCollision);
//                     //printf("added edge %i, %i\n", v, v1);
//                 }
// 		}
// 		return v1;
// 	}

// vertexDescriptor Configurator::eliminateDisturbance(vertexDescriptor v, CollisionGraph&g, Primitive  s, b2World & w){
// 	//vertexDescriptor ogV = v;
// 	//PREPARE TO LOOK AT BACK EDGES
// 	edgeDescriptor inEdge;
// 	vertexDescriptor srcVertex=v; //default
// 	bool notRoot = boost::in_degree(v, g)>0;
// 	//printf("degree = %i, is it >0, %i\n", boost::in_degree(v, g), boost::in_degree(v, g) >0);
// 	bool isLeaf=0;
// 	vertexDescriptor v1 = v; //by default if no vertices need to be added the function returns the startingVertex

// 		//FIND IF THE PRESENT Primitive WILL COLLIDE
// 	//printf("add to vertex %i, ", v);
// 	Primitive::simResult result; 

// 	//IDENTIFY SOURCE NODE, IF ANY
// 	if (notRoot){
// 		//printf("is not root\n");
// 		inEdge = boost::in_edges(v, g).first.dereference();
// 		srcVertex = boost::source(inEdge, g);
// 		result =s.willCollide(world, iteration, g[srcVertex].endPose.p, g[srcVertex].endPose.q.GetAngle()); //start from where the last Primitive ended (if previous Primitive crashes)
// 		g[inEdge]= result; //assign data to edge
// 	}

// 	else{
// 		result =s.willCollide(world, iteration, {0.0, 0.0}, 0); //default start from 0
// 	}

// 	//IS THIS NODE LEAF? to be a leaf 1) either the maximum distance has been covered or 2) avoiding an obstacle causes the robot to crash
// 	bool fl = isFullLength(v,g);
// 	if(!fl){
// 		switch (result.resultCode){
// 				case Primitive::simResult::resultType::crashed:
// 					if (g[v].getType()==Primitive::currentDMPType::BASELINE){
// 						Primitive::Action reflex;
// 						reflex.__init__(result.collision, Primitive::Direction::NONE);
// 						Primitive::Direction reflexDirection = reflex.getDirection();
// 						g[v].options.push_back(reflexDirection);// the first branch is the actions generating from a reflex to the collision
// 						g[v].options.push_back(getOppositeDirection(reflexDirection));
// 						//g[v].change=1;
// 					}
// 					break;
// 				case Primitive::simResult::resultType::successful:
// 					if (g[v].getType()==Primitive::PrimitiveType::AVOID){
// 						g[v].options.push_back(Primitive::Direction::NONE);
// 					}
// 					break;
// 				default: break;
// 			}
// 	}

// 	// for (Primitive::Direction d:g[v].options){
// 	// 	printf("action %i\n", static_cast<int>(d));
// 	// }


// 	isLeaf = fl ||(g[v].options.size() <=0);
// //	printf("is leaf? %i full length = %i, options = %i\n", isLeaf, fl, g[v].options.size());

// 	//IF THE Primitive COLLIDES CREATE A PLAN, DEPTH-FIRST
// 			//DEFINE POSSIBLE NEXT PrimitiveS DEPENDING ON OUTCOME, only if it's not a leaf
// 	if (g[v].options.size()>0){
// 			addVertex(v, v1, g, result.collision);
// 	}
// 	//IF NO VERTICES CAN BE ADDED TO THE CURRENT BRANCH, CHECK THE CLOSEST BRANCH
// 	else {
// 		//printf("seeing if can go back\n");
// 		Primitive::Object previousCollision;
//                 while (g[v].options.size()<=0){ //keep going back until it finds an incomplete node
// 					// if (boost::in_degree(v,g)>0){
// 					// }
// 					//printf("is in edge surce = target? %i\n", inEdge.m_source == inEdge.m_target);
//                     if(boost::in_degree(v, g)>0){
// 	                    inEdge = boost::in_edges(v, g).first.dereference();
// 						previousCollision = g[v].obstacle;
// 						//printf("collision from vertex %i, pos %f, %f, valid = %i\n", v, previousCollision.getPosition().x, previousCollision.getPosition().y, previousCollision.isValid());
// 						v = source(inEdge, g); //go back a node
// 					    //printf("new src = %i, in degree = %i\n", v, boost::in_degree(v, g));
//                     }
//                     else{
//                         //printf("source has no back edge\n");
//                         break;
//                     }
//                 }
// 				if (g[v].options.size()>0){ //if if the vertex exiting the while loop is incomplete add a new node
// 					addVertex(v,v1,g, previousCollision);
//                     //printf("added edge %i, %i\n", v, v1);
//                 }
// 		}
// 		return v1;
// 	}


vertexDescriptor Configurator::eliminateDisturbance(vertexDescriptor v, CollisionGraph&g, Primitive  s, b2World & w, std::vector <vertexDescriptor> &_leaves){
	//vertexDescriptor ogV = v;
	//PREPARE TO LOOK AT BACK EDGES
	edgeDescriptor inEdge;
	vertexDescriptor srcVertex=v; //default
	bool notRoot = boost::in_degree(v, g)>0;
	//printf("degree = %i, is it >0, %i\n", boost::in_degree(v, g), boost::in_degree(v, g) >0);
	bool isLeaf=0;
	vertexDescriptor v1 = v; //by default if no vertices need to be added the function returns the startingVertex

		//FIND IF THE PRESENT STATE WILL COLLIDE
	printf("add to vertex %i, ", v);
	Primitive::simResult result; 
	//b2Transform startTransform;(g[srcVertex].endPose.p, g[srcVertex].endPose.q);

	//IDENTIFY SOURCE NODE, IF ANY
	if (notRoot){
		//printf("is not root\n");
		inEdge = boost::in_edges(v, g).first.dereference();
		srcVertex = boost::source(inEdge, g);
		result =s.willCollide(w, iteration, debugOn, g[srcVertex].endPose.p, g[srcVertex].endPose.q.GetAngle()); //start from where the last Primitive ended (if previous Primitive crashes)
		g[inEdge].distanceCovered= result.distanceCovered; //assign data to edge
		g[inEdge].outcome = result.resultCode;
		//g[v].totalDistance = g[srcVertex].totalDistance + result.distanceCovered; // attach total distance to each vertex for easy score calculation
		g[v].updateTotalDistance(g[srcVertex].distanceSoFar, g[inEdge]);
		g[v].predecessors = g[srcVertex].predecessors +1;
	}

	else{
		result =s.willCollide(w, iteration, debugOn, {0.0, 0.0}, 0); //default start from 0
	}
	//FILL IN CURRENT NODE WITH ANY COLLISION AND END POSE
	g[v].obstacle = result.collision;
	g[v].endPose = result.endPose;
	//IS THIS NODE LEAF? to be a leaf 1) either the maximum distance has been covered or 2) avoiding an obstacle causes the robot to crash
	//bool fl = isFullLength(v,g);
	bool fl = g[v].distanceSoFar >= BOX2DRANGE;
	bool moreCostlyThanLeaf =0; 

	//ABANDON EARLY IF CURRENT PATH IS MORE COSTLY THAN THE LAST LEAF: if this vertex is the result of more branching while traversing a smaller distance than other leaves, it is more costly
	for (auto l: _leaves){
		if (g[v].distanceSoFar <= g[l].distanceSoFar && g[v].predecessors>= g[l].predecessors){
			moreCostlyThanLeaf =1;
		}

	}

	//ADD OPTIONS FOR CURRENT ACTIONS BASED ON THE OUTCOME OF THE Primitive/TASK/MOTORPLAN ETC i haven't decided a name yet
	if(!fl&& !moreCostlyThanLeaf && ((v==srcVertex) || (g[srcVertex].endPose !=g[v].endPose))){
		//switch (){
				if (result.resultCode != Primitive::simResult::resultType::successful){ //accounts for simulation also being safe for now
					if (s.getType()==Primitive::Type::BASELINE){
						//printf("adding L/R");
						Primitive::Action reflex;
						reflex.__init__(result.collision, Primitive::Direction::NONE);
						Primitive::Direction reflexDirection = reflex.getDirection();
						g[v].options.push_back(reflexDirection);// the first branch is the actions generating from a reflex to the collision
						g[v].options.push_back(getOppositeDirection(reflexDirection));
					}
				}
				else if (result.resultCode != Primitive::simResult::resultType::crashed){
					if (s.getType()==Primitive::Type::AVOID){
						//printf("adding str");
						g[v].options.push_back(Primitive::Direction::NONE);
					}
			}	
			//}
	}


	isLeaf = fl ||(g[v].options.size() <=0);

	if (isLeaf){ //if this is a leaf save it
		_leaves.push_back(v);
	}

	//IF THE Primitive COLLIDES CREATE A PLAN, DEPTH-FIRST
			//DEFINE POSSIBLE NEXT PrimitiveS DEPENDING ON OUTCOME, only if it's not a leaf
	if (g[v].options.size()>0){
		addVertex(v, v1, g); //ADD AN EMPTY VERTEX. only info entered for the next vertex is the direction 
		return v1; //added now	
	}
	//IF NO VERTICES CAN BE ADDED TO THE CURRENT BRANCH, CHECK THE CLOSEST BRANCH
	else {
		//printf("seeing if can go back\n");
                while (g[v].options.size()==0){ //keep going back until it finds an incomplete node
					//printf("is in edge surce = target? %i\n", inEdge.m_source == inEdge.m_target);
                    if(boost::in_degree(v, g)>0){
						//printf("in degree >0\n");
						bool isVStraight = s.getAction().getDirection()==Primitive::Direction::NONE; //check if g[v] represents goign straight
						//collision stored in result
	                    inEdge = boost::in_edges(v, g).first.dereference();
						//printf("collision from vertex %i, pos %f, %f, valid = %i\n", v, g[v].obstacle.getPosition().x, g[v].obstacle.getPosition().y, g[v].obstacle.isValid());
						v = source(inEdge, g); //go back a node
					    //printf("new src = %i, options = %i\n", v, boost::in_degree(v, g));
						if (g[v].options.size()>0){ //if if the vertex exiting the while loop is incomplete add a new node
							addVertex(v,v1,g);
						//	printf("added vertex to %i, new = %i\n", v, v1);
							return v1;
						}
                    }
                    else{
                      //  printf("source has no back edge\n");
                        break;
                    }
                }

		}
		return v1;
	}



// bool Configurator::build_tree(vertexDescriptor v, Graph&g, b2World & w){
//     vertexDescriptor v1 = eliminateDisturbance(w, v, g);
// 	//printf("set v1\n");
//     if (v1!=v){
//         build_tree(v1, g, w);
//     }
// 	return g[0].change; //

//}

// bool Configurator::build_tree(vertexDescriptor v, Graph&g, b2World & w){
// 	vertexDescriptor v1 = eliminateDisturbance(w, v, g);
//     while (v1!= v){
// 		printf("v1 = %i\n", v1);
// 		v= v1;
// 		printf("v = %i\n", v);
// 		v1 = eliminateDisturbance(w, v, g);
// 	}
// 	return g[0].change; //

// }


bool Configurator::build_tree(vertexDescriptor v, CollisionGraph& g, Primitive s, b2World & w, std::vector <vertexDescriptor> &_leaves){
	vertexDescriptor v1 = eliminateDisturbance(v, g,s,w, _leaves); 
	//destroying world causes segfault even if it's no longer required so skipping for now
    while (v1!= v){
		//printf("reset world\n");
		b2World newWorld({0.0f, 0.0f});
		//printf("v1 = %i, v = %i\n", v1, v);
		edgeDescriptor v1InEdge = boost::in_edges(v1, g).first.dereference();

		vertexDescriptor v1Src = v1InEdge.m_source;
		Primitive::Direction d = g[v1InEdge].direction;
		//Primitive::Direction d = g[boost::in_edges(v1, g).first.dereference()].direction;
		//printf("in build tree, direction = %i\n", static_cast<int>(d));

//		v = v1;

		s = Primitive(g[v1Src].obstacle, d);
		//printf("in build tree v1Src= %i, v1= %i, Primitive code = %i, obstacle valid = %i, wheel speeds = L %f, R= %f, action type = %i\n",v1Src, v1, s.getType(), s.obstacle.isValid(), s.getAction().LeftWheelSpeed, s.getAction().RightWheelSpeed, s.getAction().getDirection());
		constructWorldRepresentation(newWorld, d, g[v1Src].endPose); //was g[v].endPose
		//DEBUG
		// char filename[256];
		// sprintf(filename, "/tmp/dumpbodies%04i_%i.txt", iteration, v1);
		// FILE * dump = fopen(filename, "w");
		// for (b2Body * b = newWorld.GetBodyList(); b!=NULL; b=b->GetNext()){
		// 	fprintf(dump, "%.2f\t%.2f\n", b->GetPosition().x, b->GetPosition().y);
		// }
		// fclose(dump);
		//END DEBUG
		v= v1;
		//printf("v = %i\n", v);
		v1 = eliminateDisturbance(v,g,s, newWorld, _leaves);
	}
	//printf("end buildtree\n");
	return !g[0].obstacle.safeForNow;

}