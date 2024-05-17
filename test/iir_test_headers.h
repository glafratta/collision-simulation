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

void step( AlphaBot &motors);

unsigned int getStep(){
    return m_step;
}

void setA(char _a='0'){
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


	void hasFrame(const cv::Mat &frame, const libcamera::ControlList &);
private:
ImgProc imgProc;
MotorCallback *cb=NULL;
};