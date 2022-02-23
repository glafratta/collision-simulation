#include "/usr/include/Box2D/Box2D.h"
#include <iostream>
#include <chrono>
#include "robot.h"
#include "listener.h"
#include "environment.h"
#include "cppTimer/CppTimerCallback.h"
#include <thread>


//the environment is sampled by the LIDAR every .2 seconds

void plotData(b2Body* body, b2World& world, std::string filename) { ///prints path on screen	
	std::ofstream file(filename);
	float timeStep = 1.0f / 80.0f;
	int32 velocityIterations = 8;
	int32 positionIterations = 3;
	for (int32 i = 0; i < 500; i++)
	{
		world.Step(timeStep, velocityIterations, positionIterations);
		b2Vec2 position = body->GetPosition();
		file << position.x << "\t" << position.y << std::endl;
	}
}


class updateCallback :public CppTimerCallback::Runnable{
public:
	Environment * e;
	void setEnv(Environment * env){
		e=env;
	}
private:
	void run(){
		e->robot->bodyDef.position.SetZero();
		e->update();
		int i= e->getIteration();
		//std::cout<<e->getMap(i)->getFilename()<<std::endl;
	}

};

int main() {
	//world setup with environment class

	Environment* env = new Environment;
	Listener* listener = new Listener;
	b2World* world = env->world; //using these variables for ease
	world->SetContactListener(listener);
	Robot* robot = env->robot;
	env->setFileList();
	robot->setVelocity({1.0f, 0.0f});

	// robot->setVelocity({ 0.0, 3.0 });
	// CppTimerCallback timer;
	// updateCallback callback;
	// callback.setEnv(env);
	// timer.registerEventRunnable(callback);
	// timer.startms(200);
	// int duration= 200 * env->getFileList().size();
	// //while (env->getMap(env->getIteration())){
	// std::this_thread::sleep_for(std::chrono::milliseconds(duration));
	// //}
	// std::cout<<env->getIteration();
	int total=0;
	for (int i=0; i<env->getFileList().size(); i++){
		//env->simulate();
		env->update();
		std::cout<<env->getMap(i)->getFilename()<<std::endl;
		// std::cout<<"duplicates: "<<env->getMap(i)->countDup()<<" out of "<<env->getMap(i)->countLines()<<". This is "<<(float)env->getMap(i)->countDup()/(float)env->getMap(i)->countLines()<<"of total points."<<std::endl;
		// total += env->getMap(i)->countDup();


		for (int j=0; j<1000;j++){
			for (int z=0; z<2; z++){
				std::cout<<env->getMap(j)->coordinates[j][z]<<"\t";
			}
			std::cout<<"\n";
		}
	}
	// std::cout<<"avg number of duplicates is "<<total /env->getFileList().size()<<std::endl;













	////////CODE LAYOUT////////////
	///For event handling install Qt
	//Events that need handled: robot-> is crashed, a map.dat file being created


	//timeStart;

	//while(stopKeyPressed = false){
		//if (time==timeStart+.2s){
		//env->update(); //uodates environment data
		// env->step(); //simulates data for the next x seconds and if the robot crashes in that time it makes a maneuvre
		//}
//}
	
	
	
	//plotData(robot->body, *world, "robot.dat");

	// for (int i = 0; i < 300; i++) {
	// 	world->Step(TIME_STEP, VEL_IT, POS_IT);
	// 	std::cout << robot->body->GetPosition().x << "\t" << robot->body->GetPosition().y << std::endl;
	// };
	// std::cout<<robot->getVelocity().Length()<<std::endl;

	delete listener, env, world, robot;
}
	
	