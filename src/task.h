#ifndef TASK_H
#define TASK_H
#include "measurement.h"
const float SIM_DURATION = int(BOX2DRANGE*2 /MAX_SPEED);


class Task{
public:
    friend class Configurator;
    char planFile[250]; //for debug
    b2Transform start;
    bool change =0;
    EndCriteria endCriteria; //end criteria other than task encounters a disturbance
    Direction direction= DEFAULT;
    int motorStep=0;
    int stepError=0;
protected:
public:
struct Action{
private:
    float linearSpeed=MAX_SPEED/2; //used to calculate instantaneous velocity using omega
    float recordedSpeed=linearSpeed;
    float omega=0; //initial angular velocity is 0
    float recordedOmega = omega;
    bool valid=0;
public:
    float R=.5;
    float L=.5;

    Action()=default;

    void init(Direction& direction){
        switch (direction){
        case Direction::DEFAULT:
        L=0.5;
        R=.5;
        break;
        case Direction::LEFT:
        L = -0.5;
        R=0.5;
        break;
        case Direction::RIGHT:
        L=0.5;
        R = - 0.5;
        break;
        case Direction::BACK:
        L = -0.5;
        R = -0.5;
        break;
        case Direction::STOP:
        L=0;
        R=0;
        direction=Direction::DEFAULT;
        break;
        default:
        throw std::invalid_argument("not a valid direction for M");
        break;
    }
    //kinematic model internal to action so it can be versatile for use in real P and simulated P
    setVelocities(L, R);
    }

void setVelocities(float l, float r){
    omega = (MAX_SPEED*(r-l)/BETWEEN_WHEELS)*TURN_FRICTION; //instant velocity, determines angle increment in willcollide
    recordedOmega = omega;
    linearSpeed = MAX_SPEED*(l+r)/2;
    recordedSpeed=linearSpeed;
    valid=1;
}

    b2Vec2 getLinearVelocity(){
        b2Vec2 velocity;
        velocity.x = linearSpeed *cos(omega);
        velocity.y = linearSpeed *sin(omega);
        return velocity;
    }

    b2Transform getTransform(){
    return b2Transform(getLinearVelocity(), b2Rot(omega));
}

    float getRWheelSpeed(){
        return R;
    }

    float getLWheelSpeed(){
    return L;
    }


    bool isValid(){
        return valid;
    }

    float getLinearSpeed(){
        return linearSpeed;
    }

    float getOmega(){
    return omega;
    }

    void setOmega(float o){
        omega =o;
    }

    void setLinearSpeed(float s){
        linearSpeed =s;
    }

    void setRecSpeed(float s){
        recordedSpeed =s;
    }

    void setRecOmega(float w){
        recordedOmega=w;
    }

    float getRecSpeed(){
        return recordedSpeed;
    }

    float getRecOmega(){
        return recordedOmega;
    }
    //friend class Configurator;
    void setRec(float _speed, float _omega){
        recordedSpeed=_speed;
        recordedOmega=_omega;
    }

};



class Listener : public b2ContactListener {
  int iteration=1;
    public:
    Listener(){}
    std::vector <b2Vec2> collisions;
    
		void BeginContact(b2Contact * contact) {
			b2BodyUserData bodyData = contact->GetFixtureA()->GetBody()->GetUserData();
			if (bodyData.pointer) {
                b2Body * other = contact->GetFixtureB()->GetBody();
                collisions.push_back(other->GetPosition());
			}
			bodyData = contact->GetFixtureB()->GetBody()->GetUserData();
			if (bodyData.pointer) {
                b2Body * other = contact->GetFixtureA()->GetBody();
                collisions.push_back(other->GetPosition());
                }       
		}
        
	};
private:
Action action;
public:
Disturbance disturbance;

Task::Action getAction(){
    return action;
}

AffordanceIndex getAffIndex(){
    return disturbance.getAffIndex();
}


Direction H(Disturbance, Direction, bool topDown=0); //topDown enables Configurator topdown control on reactive behaviour

void setEndCriteria(Angle angle=SAFE_ANGLE, Distance distance=BOX2DRANGE);

void setErrorWeights();

EndedResult checkEnded(b2Transform robotTransform = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)));

EndedResult checkEnded(State);

Task(){
    start = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    direction = DEFAULT;
    action.init(direction);
    printf("default constructro\n");
}

Task(Direction d){
    direction=d;
    action.init(direction);
}

Task(Disturbance ob, Direction d, b2Transform _start=b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)), bool topDown=0){
    start = _start;
    disturbance = ob;
    direction = H(disturbance, d, topDown);  
    //action = Action(direction);
    action.init(direction);
    setEndCriteria();
}

simResult willCollide(b2World &, int, bool debug =0, float remaining = 8.0, float simulationStep=BOX2DRANGE);

struct Correct{
    
    Correct(){}

    void operator()( Action&);

    float errorCalc(Action, double);

    float getError(){
        return p();
    }

    float get_i(){
        return i;
    }

    float update(float);

    float kp=0.08;
    private:


    float p(){
        float sum=0;
        for (int j=0;j<p_buffer.size(); j++){
            sum+=p_buffer[j];
        }
        return sum;
    }
    std::vector <float>p_buffer=std::vector <float>(3,0);
    float kd=1, ki=1;
    float i=0, d=0;

}correct;

friend Task::Correct;    

};

#endif