#include "box2d/box2d.h"
#include "robot.h"
#include <iostream>


class Listener : public b2ContactListener {
public:
	//Listener();
	//~Listener();
	void BeginContact(b2Contact * contact) {
		void* bodyData = &(contact->GetFixtureA()->GetBody()->GetUserData());
		if (bodyData) {
			static_cast<Robot*>(bodyData)->startContact(); //this is where I've had problems in the past
			//std::cout<<"bonk\n";
		}
	}

	void EndContact(b2Contact* contact) {
		void* bodyData = &(contact->GetFixtureA()->GetBody()->GetUserData());
		if (bodyData) {
			static_cast<Robot*>(bodyData)->endContact(); //this is where I've had problems in the past
			//std::cout << "bye-ee!\n";
		}
		//std::cout << "unbonked\n";
	}

	//void PreSolve() {
	//	void* bodyData = &(contact->GetFixtureA()->GetBody()->GetUserData());
	//	if (bodyData) {
			//static_cast<Robot*>(bodyData)->body->a; //this is where I've had problems in the past
			//std::cout << "bye-ee!\n";
	//	}
	//}
};
