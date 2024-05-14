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
    printf("char =%c\n", a);
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

char getID(){
    return a;
}
};

struct CameraCallback: Libcam2OpenCV::Callback {
    CameraCallback(MotorCallback * _cb):cb(_cb){}

	virtual void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) {
		std::vector <cv::Point2f> corners= imgProc.corners();
        cv::Mat previousFrame =imgProc.previous();
        b2Vec2 optic_flow=imgProc.opticFlow(frame,corners, previousFrame);
		cb->t.correct.update(optic_flow.x); //for now just going straight
        //if (cb->t.direction!=STOP){
            char dumpname[50];
            sprintf(dumpname, "%s_%i.txt", cb->getID(), cb->getCount());
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
	AlphaBot motors;
    Libcam2OpenCV camera;
    Libcam2OpenCVSettings settings;
    settings.framerate = 30;
	MotorCallback cb;
    CameraCallback cameraCB(&cb);
    camera.registerCallback(&cameraCB);
	motors.registerStepCallback(&cb);
    camera.start(settings);
	motors.start();
	do {
       // if (getchar()){
            char a=getchar();
            cb.setA(a);
       // } 
	} while(true);
	motors.stop();
    camera.stop();

}
	
	
