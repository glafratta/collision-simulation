#include "iir_test_headers.h"


void MotorCallback::step( AlphaBot &motors){
    m_step--;
    if (m_step==0){
        setA();
        return;
    }
	//Task::Action action=t.getAction();
	t.correct(t.action, m_step);
    motors.setRightWheelSpeed(t.getAction().getRWheelSpeed()); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(t.getAction().getLWheelSpeed());
    printf("L=%f, R=%f\n", t.getAction().getLWheelSpeed() ,t.getAction().getRWheelSpeed());
    printf("char =%c, step=%i\n", a, m_step);
}

void MotorCallback::setA(char _a){
    a=_a;
    if (a== 'l'){
        t=Task(LEFT);
        m_step=15;
        n_l++;
    }
    else if (a=='r'){
        t=Task(RIGHT);
        m_step=15;
        n_r++;
    }
    else if (a=='s'){
        t=Task(DEFAULT);
        m_step=28;
        n_s++;
    }
    else{
        t=Task(STOP);
        
    }
}

void CameraCallback::hasFrame(const cv::Mat &frame, const libcamera::ControlList &){
		//printf("has frame\n");
        float error=0;
        cv::Vec2d  optic_flow=imgProc.avgOpticFlow(frame);
        cv::Vec2d  optic_flow_filtered=optic_flow;
        //printf("optic flow = %f, %f\n", optic_flow[0], optic_flow[1]);
        signal= signal+optic_flow[0];
        optic_flow_filtered[0]=low_pass.filter((optic_flow[0]));
        optic_flow_filtered[0]= band_stop.filter(optic_flow_filtered[0]);
        filtered_signal=filtered_signal+optic_flow_filtered[0];
		if (cb->t.motorStep!=cb->getStep() & cb->getStep()!=0){ //, in the future t.motorStepdiscard will be t.change
																//signal while the robot isn' moving
        	Task::Action action= cb->t.getAction();
			error= cb->t.correct.errorCalc(action, double(optic_flow_filtered[0]));
			printf("error=%f\n", error);
		}
        cb->t.correct.update(error); //for now just going straight
        if (cb->getStep()<=0){
            return;
        }
        FILE * dump=fopen(dumpname, "a+");
        fprintf(dump, "%f\t%f\t%f\t%f\n", 
            error, cb->t.correct.getError(), cb->t.correct.get_i(), cb->t.correct.get_d());
        fclose(dump);
    }

int main(int argc, char** argv) {
    char a=0;
    if (argc>1){
        a=*argv[1];
    }

	AlphaBot motors;	
    MotorCallback cb;
    cb.setA(a);
	if (argc>3){
        char k=*argv[3];
		cb.setK(atof(argv[2]), k);
	}
    CameraCallback cameraCB(&cb);
    sprintf(cameraCB.dumpname, "errors_p%.3f_i%.3f_d%.3f.txt", cb.t.correct.Kp(), cb.t.correct.Ki(), cb.t.correct.Kd());
    FILE * dump=fopen(cameraCB.dumpname, "w+");
    fclose(dump);
    Libcam2OpenCV camera;
    camera.registerCallback(&cameraCB);
    Libcam2OpenCVSettings settings;
    settings.framerate = 30;
	motors.registerStepCallback(&cb);
    camera.start(settings);
	motors.start();
	// do {
    //    // if (getchar()){
    //         //char a=getchar();
            
    //    // } 
	// } while(true);
    getchar();
	motors.stop();
    camera.stop();

}