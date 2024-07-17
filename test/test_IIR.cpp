#include "iir_test_headers.h"

void MotorCallback::step( AlphaBot &motors){
    m_step--;
    if (m_step==0){
        setA();
    }
    motors.setRightWheelSpeed(t.getAction().getRWheelSpeed()); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(t.getAction().getLWheelSpeed());
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
        m_step=100;
        n_s++;
    }
    else{
        t=Task(STOP);
        
    }
}

void CameraCallback::hasFrame(const cv::Mat &frame, const libcamera::ControlList &){
		printf("has frame\n");
        cv::Vec2d  optic_flow=imgProc.avgOpticFlow(frame);
        cv::Vec2d  optic_flow_filtered=optic_flow;
        printf("optic flow = %f, %f\n", optic_flow[0], optic_flow[1]);
        signal= signal+optic_flow[0];
        optic_flow_filtered[0]=low_pass.filter((optic_flow[0]));
        optic_flow_filtered[0]= band_stop.filter(optic_flow_filtered[0]);
        filtered_signal=filtered_signal+optic_flow_filtered[0];
        // cb->t.correct.update(optic_flow_filtered[0]); //for now just going straight
        FILE * dump=fopen(dumpname, "a+");
        fprintf(dump, "%f\t%f\t%f\t%f\t%f\t%f\n", 
            optic_flow[0], optic_flow[1], optic_flow_filtered[0], optic_flow_filtered[1], signal, filtered_signal);
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
    CameraCallback cameraCB(&cb);
    sprintf(cameraCB.dumpname, "avg%s_%i_iir.txt", cb.getID(), cb.getCount());
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
	
	
