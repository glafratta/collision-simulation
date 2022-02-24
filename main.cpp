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
	Box2DEnv * b;
	void setEnv(Box2DEnv * box2d){
		e=env;
	}
private:
	void run(){
		box2d->robot->bodyDef.position.SetZero();
		box2d->update();
		int i= box2d->getIteration();
		//std::cout<<e->getMap(i)->getFilename()<<std::endl;
	}

};



int main() {
	//world setup with environment class

	Box2DEnv* box2d = new Box2DEnv;
	Listener* listener = new Listener;
	world->SetContactListener(listener);
	A1Lidar lidar;
	DataInterface dataInterface;
	lidar.registerInterface(&dataInterface);
	lidar.start();
	do {
	} while (!getchar());
	lidar.stop();
	

	// CppTimerCallback timer;
	// updateCallback callback;
	// callback.setEnv(env);
	// timer.registerEventRunnable(callback);
	// timer.startms(200);
	// int duration= 200 * env->getFileList().size();
	// //while (env->getMap(env->getIteration())){
	// std::this_thread::sleep_for(std::chrono::milliseconds(duration));
	// //}
	

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
	
	