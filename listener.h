#include "Box2D/Box2D.h"
#include "robot.h"
#include <iostream>


class Listener : public b2ContactListener {
public:
	//Listener();
	//~Listener();
	void BeginContact(b2Contact * contact) {
		void* bodyData = contact->GetFixtureA()->GetBody()->GetUserData();
		if (bodyData) {
			static_cast<Robot*>(bodyData)->startContact(); 
			//std::cout<<"robot is fixture A\n";
		}

		bodyData = contact->GetFixtureB()->GetBody()->GetUserData();
		if (bodyData) {
			static_cast<Robot*>(bodyData)->startContact(); 
		}
		
	}

	void EndContact(b2Contact* contact) {
		void* bodyData = contact->GetFixtureA()->GetBody()->GetUserData();
		if (bodyData) {
			static_cast<Robot*>(bodyData)->endContact(); 
			//std::cout << "unbonk robot A!\n";
		}
		bodyData = contact->GetFixtureB()->GetBody()->GetUserData();
		if (bodyData) {
			static_cast<Robot*>(bodyData)->endContact(); 
			//std::cout << "unbonk robot B!\n";
		}
		
	}

	//void PreSolve() {
	//	void* bodyData = &(contact->GetFixtureA()->GetBody()->GetUserData());
	//	if (bodyData) {
			//static_cast<Robot*>(bodyData)->body->a; //this is where I've had problems in the past
			//std::cout << "bye-ee!\n";
	//	}
	//}
};
