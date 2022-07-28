#pragma once
#include "alphabot.h"
#include "environment.h"
#include <ncurses.h>
#include "pigpio.h"


class Callback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
public:
Action currentState = GO;
//std::vector <Action> plan = {};
char toPrint[10];
int iteration=-1;
Configurator & box2d;

Callback(Configurator & _box2d): box2d(_box2d){};

void setState(){                                    //if a long (say 30 steps) plan exists, it will be executed until it's finished
    if (!box2d.plan.empty()){                             ///
        auto it = box2d.plan.begin();                     //for testing input and angles, plan is a single state, so after being sent to the callback
        currentState = *(it);                       //  it gets erased and the robot stops, so you can enter another input
        box2d.plan.erase(it);
        }
    else{
        currentState = GO;
    }
    }
public :


virtual void step(AlphaBot & motors){ //                turns in place and then goes foward again
    //setState();
     if (!box2d.plan.empty()){                             ///
                      //for testing input and angles, plan is a single state, so after being sent to the callback
        currentState = *(box2d.plan.begin());
        box2d.plan.erase(box2d.plan.begin());
                       //  it gets erased and the robot stops, so you can enter another input
        }
    else{
        currentState = GO;
    }
    switch(currentState){
        case GO:
        //sprintf(toPrint, "fw\n");
        motors.setLeftWheelSpeed(1.0);
        motors.setRightWheelSpeed(1.0);        
        break;
        case RIGHT:
        //sprintf(toPrint, "right\n");
        motors.setLeftWheelSpeed(1.0);
        motors.setRightWheelSpeed(-1.0);        
        break;
        case LEFT:
       // sprintf(toPrint, "left\n");
        motors.setLeftWheelSpeed(-1.0);
        motors.setRightWheelSpeed(1.0);   
        break;
        // case R_SMOOTH:							//not yet
        // sprintf(toPrint, "right smooth\n");
        // motors.setLeftWheelSpeed(1.0);
        // motors.setRightWheelSpeed(0.25);         //trying smooth turning by just decreasing the ipsilateral wheel speed
        // break;
        // case L_SMOOTH:
        // sprintf(toPrint, "left smooth\n");
        // motors.setLeftWheelSpeed(0.25);
        // motors.setRightWheelSpeed(1.0);   
        // break;
        default:
        currentState=STOP;
       // sprintf(toPrint, "invalid plan\n");
        motors.setLeftWheelSpeed(0.0);
        motors.setRightWheelSpeed(0.0);   
        break;
    }
  //  mvaddstr(0,0, toPrint);
  //  refresh();
}

};
