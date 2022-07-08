bool State::willCollide(b2World & _world){ //CLOSED LOOP CONTROL
		crashed =false;
		Listener listener;
		_world.SetContactListener(&listener);		
		Robot robot(&world);
		char name_r[50];
		sprintf(name_r, "/tmp/robot%04d_%i.txt", iteration, type);
		FILE * robotPath = fopen(name_r, "a+");
        float dOmega=omega/hz;
        float omegaT=0; //instantaneous angular velocity at time T
		for (int step = 0; step <= (hz*simDuration); step++) {//3 second
            float dx= RecordedVelocity.Length()*(-sin(omegaT)); //recorded velocity because we assume that the linear velocity is constant in a state
            float dy = RecordedVelocity.Length()*cos(omegaT);
			robot.setVelocity({dx,dy}); //instantaneous linear veloctiy
			_world.Step(1.0f/hz, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
			fprintf(robotPath, "%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y); //save predictions
            omegaT+=dOmega;
			if (listener.collision.isValid){ //
                listener.collision.setStep(step);
                collisions.push_back(listener.collision);
				fclose(robotPath);
				return;
			}


		}
		//setPlan(_action); 		//this needs refinement as the thingy for the alphabot is hard-coded. coudl be, if one callback is 10d R or 13 to the L, get the angle from each speed in the plan and divide it
		printf("path is safe\n");
		fclose(robotPath);
	}