void Configurator::NewScan(){ 
		iteration++;
		//if this is the first run, set state
		if (!currentState.type){
			currentState = desiredState; //this way the state lives between scans
		}
		//make the plan for current
		crashed=0;
		realVelocity = GetRealVelocity();
		// float speed = realVelocity.Length(); //to move at constant speed
		// float angle = -atan2(realVelocity.x, realVelocity.y);

		//WRITE TO FILE (debug) + WORLD BUILDING
		b2World world = b2World({0.0f,0.0f});
		char name[256];
		sprintf(name, "/tmp/bodies%04d.txt", iteration);
		FILE *file= fopen(name, "w+");
		

		for (int i=0; i<current.size(); i++){ //makes obstacles and checks for duplicates
			b2Vec2 pointCurrent(roundf((current[i].x)*100.f)/100.f, roundf((current[i].y)*100.f)/100.f);
			b2Vec2 pointPrevious(roundf((current[i-1].x)*100.f)/100.f, roundf((current[i-1].y)*100.f)/100.f);
			if (pointCurrent.x != pointPrevious.x && pointCurrent.y!=pointPrevious.y){ // robot only sees obstacles ahead of it
				b2Body * body;
				b2BodyDef bodyDef;
				b2FixtureDef fixtureDef;
				bodyDef.type = b2_dynamicBody;
				b2PolygonShape fixture; //giving the ground the shape of a box. useful to know if collision can be detected without fixture
				fixtureDef.shape = &fixture;
				fixture.SetAsBox(.001f, .001f); 
				if (pointCurrent.x==0.0f && pointCurrent.y==0.0f){
					printf("weird!");
				}
					bodyDef.position.Set(pointCurrent.x, pointCurrent.y); //set origin to ground body (is this in world coor?)
					body = world.CreateBody(&bodyDef);
					body->CreateFixture(&fixtureDef);
					fprintf(file, "%f\t%f\n", body->GetPosition().x, body->GetPosition().y);

			}
			currentState.setVelocity(GetRealVelocity(currentState.RecordedVelocity)); //logs in previous recorded velocity in case affine transformation fails
			currentState.WillCollide(world);
            //check if collided
			if (currentState.hasCollision()){
                printf("bonk");
            }
		}
		fclose(file);

		


		// switch (openLoop){ //OLD THING, OPEN LOOP, MOVES IN A STRAIGHT LINE
		// 	case 0: { //OPEN LOOP, NOT TESTED ON ROBOT
		// 		float steer = 0; //in this model robot moved at k velocity and k angle
		// 		float acceleration =0;
		// 		std::vector <float> angles={0,M_PI/2, -M_PI/2};
		// 		for (int n=0; n<angles.size(); n++){ //can put in inlined functions
		// 			std::vector <b2Vec2> plan; //make plan
		// 			Action _action(speed, angle+angles[n], steer);
		// 			_action.type=static_cast<ActionType>(n);
		// 			for (int i=0; i<hz*simDuration;i++){
		// 				action= Action(speed, angle +angles[n],steer); //time step not important because turning angle is constant
		// 				b2Vec2 instantVel(action.x, action.y);
		// 				plan.push_back(instantVel);
		// 			}
		// 			printf("action =%i\n", _action.type);
		// 			WillCollide(_action, world);
		// 			if ((!optimalAction.isValid)||optimalAction.toi<_action.toi){ //see if current action is optimal
		// 				optimalAction=_action;
		// 			}
		// 			else if (optimalAction.toi == _action.toi){
		// 				int random = rand()%2;
		// 				switch(random){
		// 					case 0: break; //optimal action stays the same
		// 					case 1: optimalAction=_action;break;
		// 					default: printf("invalid action selected\n");break;
		// 				}
		// 			}
		// 			if (n ==0 && crashed==false){ //if robot hasn't crashed with the "go" action path is safe, don't need to do anything
		// 				can have the actiontype be for the robot. Take turning angle from action and make plan for motor
		// 				look at the way alphabot is thinghied because maybe if there's no plan it just keeps going
		// 				return; 
		// 			}
		// 		printf("rechecking action\n");
		// 			else save location and time of collision and compare it to left or right, execute the plan that collides the latest
		// 		}
		// 	break;
		// 	}
		// 	case 1:{

		// 	}
		// 	default: {
		// 		printf("can't make smart plan yet\n"); 
		// 		break;				
		// 	}
		// }
		// printf("optimal action=%i\n", optimalAction.type);

	}

b2Vec2 Configurator::GetRealVelocity(b2Vec2 previousVelocity={0,0}){		
        b2Vec2 returnVec = previousVelocity;
		std::vector <cv::Point2f> currentTmp =current;
		std::vector <cv::Point2f> previousTmp=previous;
        //adjust for discrepancies in vector size
		if (previousTmp.size()>0){
			int diff = currentTmp.size()-previousTmp.size(); //if +ve,current is bigger, if -ve, previous is bigger
			if(diff>0){ //(current.size()>previous.size()){ 
				for (int i=0; i<abs(diff); i++){
				previousTmp.push_back(previousTmp[-1]);
				}
			}
		
			else if (diff<0){//(current.size()<previous.size()){
				for (int i=0; i<abs(diff); i++){
				currentTmp.push_back(currentTmp[-1]);
			}
			}
		cv::Mat transformMatrix = cv::estimateAffinePartial2D(previousTmp, currentTmp, cv::noArray(), cv::LMEDS);
			if (!transformMatrix.empty()){
				b2Vec2 tmp;
				tmp.x= -(transformMatrix.at<double>(0,2))/samplingRate;
				tmp.y = -(transformMatrix.at<double>(1,2))/samplingRate;
				// if (abs(tmp.Length())>maxAbsSpeed){
				// 	tmp=defaultSpeed;
				// }
				returnVec = tmp;

			}
		}
		return returnVec;
	}