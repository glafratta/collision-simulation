#include "task.h"
#include "sensor.h"
#include "libcam2opencv.h"
#include "alphabot.h"
#include "Iir.h"

const int order=3;
const int DC=0; //HZ
const int cutoff_frequency=4; //HZ
const int band_width=0.5;

class MotorCallback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
    unsigned int m_step=0;
    char a='0';
    int n_l=0, n_r=0, n_s=0;
public:
Task t=Task(STOP);


MotorCallback(){}

void step( AlphaBot &motors){
    m_step--;
    if (m_step==0){
        setA();
    }
    motors.setRightWheelSpeed(t.getAction().getRWheelSpeed()); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(t.getAction().getLWheelSpeed());
    printf("char =%c, step=%i\n", a, m_step);
}

unsigned int getStep(){
    return m_step;
}

void setA(char _a='0'){
    a=_a;
  //  printf("size of dn=%i\n", sizeof(dumpname));
    //memset(dumpname, 0, sizeof(dumpname));
    //printf("cleared dumpname, size =%i\n", sizeof(dumpname));
    //dumpname =char[50];
    //dumpname.clear();
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
  // 
//     snprintf(dumpname, "%c_%i.txt", getID(), getCount());
//    dumpname =std::string(tmp);
//     FILE * dump=fopen(dumpname, "w+");
//     fclose(dump);
}

int getCount(){
    if (a=='s'){
        return n_s;
    }
    else if (a=='l'){
        return n_l;
    }
    else if (a=='r'){
        return n_r;
    }
    else if (a=='0'){
        return 0;
    }

}

char* getID(){
    return &a;
}
};

struct CameraCallback: Libcam2OpenCV::Callback {
    char dumpname[50];
    //struct FilterParameters{
    //};
    double signal=0;
    double filtered_signal=0;
   // FilterParameters filter_parameters;
    Iir::Butterworth::LowPass<order>low_pass;
    Iir::Butterworth::BandStop<order>band_stop;

    CameraCallback(MotorCallback * _cb):cb(_cb){
        low_pass.setup(FPS, cutoff_frequency);
        band_stop.setup(FPS, DC, band_width);
    }


	void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) {
		printf("has frame\n");
        if (cb->t.motorStep==cb.getStep()|| cb.getStep()==0){
            return;
        }
        cv::Vec2d  optic_flow=imgProc.avgOpticFlow(frame);
        cv::Vec2d  optic_flow_filtered=optic_flow;
        printf("optic flow = %f, %f\n", optic_flow[0], optic_flow[1]);
        signal= signal+optic_flow[0];
        optic_flow_filtered[0]=low_pass.filter((optic_flow[0]));
        optic_flow_filtered[0]= band_stop.filter(optic_flow_filtered[0]);
		//cb->t.correct.update(optic_flow.x); //for now just going straight
        filtered_signal=filtered_signal+optic_flow_filtered[0];
        FILE * dump=fopen(dumpname, "a+");
        fprintf(dump, "%f\t%f\t%f\t%f\t%f\t%f\n", 
            optic_flow[0], optic_flow[1], optic_flow_filtered[0], optic_flow_filtered[1], signal, filtered_signal);
        fclose(dump);
    }
private:
ImgProc imgProc;
MotorCallback *cb=NULL;
};

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
	
	
