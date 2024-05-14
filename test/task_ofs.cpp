#include "task.h"
#include "sensor.h"
#include "libcam2opencv.h"
#include "alphabot.h"

class MotorCallback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
    unsigned int m_step=0;
    char a='0';
    int n_l, n_r, n_s;
public:
Task t=Task(STOP);

MotorCallback(){}

void step( AlphaBot &motors){
    m_step--;
    if (m_step==0){
        a='0';
        t=Task(STOP);
    }
    motors.setRightWheelSpeed(t.getAction().getRWheelSpeed()); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(t.getAction().getLWheelSpeed());
}

void setA(char _a){
    a=_a;
    if (a== 'l'){
        t=Task(LEFT);
        m_step=14;
        n_l++;
    }
    else if (a=='r'){
        t=Task(RIGHT);
        m_step=14;
        n_r++;
    }
    else if (a=='s'){
        t=Task(DEFAULT);
        m_step=22;
        n_s++;
    }
    else{
        t=Task(STOP);
    }
}

int getCount(){
    if (a=='s'){
        return n_s;
    }
    else if (a=='l'){
        return n_l;
    }
    else if (a='r'){
        return n_r;
    }
    else if (a='0'){
        return 0;
    }
}
};

struct CameraCallback: Libcam2OpenCV::Callback {
    CameraCallback(MotorCallback * _cb):cb(_cb){}

	virtual void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) {
		b2Vec2 optic_flow=imgProc.opticFlow(frame, imgProc.corners(), imgProc.previous());
		cb->t.correct.update(optic_flow.x); //for now just going straight
        //if (cb->t.direction!=STOP){
            char dumpname[50];
            sprintf(dumpname, "%s_%i.txt", cb->a, cb->getCount());
            FILE * dump=fopen(dumpname, "a+");
            fprintf(dump, "%f\t%f\n", optic_flow.x, optic_flow.y);
            fclose(dump);
        //}
    }
private:
ImgProc imgProc;
MotorCallback *cb=NULL;
};

int main(int argc, char** argv) {
    if (argc==1){
        return 0;
    }
	AlphaBot motors;
	MotorCallback cb();
	motors.registerStepCallback(&cb);
	motors.start();
	do {
        if (getchar()){
            char a=getchar();
            cb.setA(a);
        } 
	} while(true);
	motors.stop();

}
	
	
