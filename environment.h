#ifndef ENV_H
#define ENV_H

#include "obstacle.h"
#include "Box2D/Box2D.h"
#include "robot.h"
#include <vector>
#include "opencv2/opencv.hpp"
#include <thread>
#include <cmath>
//#include "/usr/local/include/alphabot.h"
//#include <string>
//#include <dirent.h>



class Box2DEnv{
public:
	float samplingRate = 1.0f / 5.0f; //for now it is fixed at .2ms but will need to write a function timing each revolution 
	int iteration=-	1; //represents that hasn't started yet, robot isn't moving and there are no map data
	int lidarIteration=-1;
	bool running = false;
	b2Vec2 gravity = { 0.0f, 0.0f };
	b2World * world = new b2World(gravity);
	Robot robot = Robot(world);
	std::vector <cv::Point2f> frames[2];
	std::thread * worker = nullptr; //for scan listener
	



	void setIteration(int i) {
		iteration = i;
	}

	int getIteration() {
		return iteration;
	}


	void findRealVelocity(){
		iteration++;
		if (iteration>0){
		std::vector <cv::Point2f> current= frames[1];
		std::vector <cv::Point2f> previous = frames[0];
		current.resize(8192);
		previous.resize(8192);
		cv::Mat transformMatrix = cv::estimateAffinePartial2D(previous, current, cv::noArray(), cv::LMEDS);
		if (!transformMatrix.empty()){
			float velX = -(transformMatrix.at<float>(0,2))/samplingRate;
			float velY = -(transformMatrix.at<float>(1,2))/samplingRate;
			robot.setVelocity({velX, velY});
		}
		//std::cout<<transformMatrix <<std::endl<<std::endl;
		//std::cout<<"velocity: "<<robot->velocity.x<<" , "<<robot->velocity.y<<"\tangle: "<<robot->angle<<"\n";
		}
	}



	void simulate() { 
		//printf("robot velocity %f, %f", robot.velocity.x, robot.velocity.y);
		for (int i = 0; i <= 30; i++) {//1 second
			world->Step(1.0f/10.0f, 2, 6);
		}
		//std::cout<<"simulation "<<iteration<<" complete\n";
		
	}



	bool isRunning(){
		return running;
	}

	bool isNewScan(){
		return lidarIteration>iteration;
	}

	float approximate(float a){ //more efficient with pointers?? (float a, float * ptr){assign value of a to *ptr}
		if (a>=.10 or a<=-.10){ //only approximating points which will not be put in the 0 bin (prevent overlap with the robot)
			a = round(a*10.0)/10.0;
			if (a<=-0.10){ //centering
				a-=0.05;
			}
			else if (a>=0.10){
				a+= 0.05;
			}
		}
		return a;
	}

	cv::Point2f approximatePoint(cv::Point2f p){
		p = cv::Point2f(approximate(p.x), approximate(p.y));
		return p;
	}


	void makeObstacles(){
		std::vector <cv::Point2f> points = frames[1];
		for (int i=0; i<points.size(); i++){ //makes obstacles and checks for duplicates
			b2Body * body;
			b2BodyDef bodyDef;
			cv::Point2f pointCurrent = approximatePoint((points[i]));
			cv::Point2f pointPrevious = approximatePoint((points[i-1]));
		 if (pointCurrent !=pointPrevious){
			b2FixtureDef fixtureDef;
			b2PolygonShape fixture;
			fixtureDef.shape = &fixture;
			bodyDef.type = b2_dynamicBody;
			fixture.SetAsBox(.05f, .05f); 
			bodyDef.position.Set(pointCurrent.x, pointCurrent.y); //set origin to ground body (is this in world coor?)
			body = world->CreateBody(&bodyDef);
			body->CreateFixture(&fixtureDef);
			//printf("%f\t%f\n", body->GetPosition().x, body->GetPosition().y);
			}
		}

	}

	void clearObstacles(){
		for (b2Body* b=world->GetBodyList(); b;b= b->GetNext()){
			if (!b->GetUserData()){
				world->DestroyBody(b);

			}
	}
	}


	static void scanListener(Box2DEnv * box2d){ //to be able to call it in start()
		while (box2d->isRunning()){
			if (box2d->isNewScan()){
				// auto begin = std::chrono::high_resolution_clock::now();
				box2d->makeObstacles();
				box2d->findRealVelocity();
				std::vector <cv::Point2f> points = box2d->frames[1];
				int32 count = box2d->world->GetBodyCount();
				box2d->simulate();
				// auto end = std::chrono::high_resolution_clock::now();
				// auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
				//std::cout<<"simulated in "<<elapsed.count()* 1e-9<<"seconds\n";
				box2d->frames[0]=box2d->frames[1];
				box2d->frames[1].clear();
				box2d->clearObstacles();
			}
		}
	}



	void start(){
		running = true;

		worker= new std::thread(Box2DEnv::scanListener, this);
		
	}

	void stop(){
		running=false;
		worker->join();
	}
};
	///TO_DO: make the map vector of a max size reflecting working memory ca. 20s = 100 maps

#endif