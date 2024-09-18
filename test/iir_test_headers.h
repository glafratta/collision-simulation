#include "task.h"
#include "sensor.h"
#include "libcam2opencv.h"
#include "alphabot.h"
#include "Iir.h"

// const int order=3;
// const int DC=0; //HZ
// const int cutoff_frequency=4; //HZ
// const int band_width=0.5;

struct Correct{
    
    Correct(){}

    void operator()( Task::Action&, int);

    float errorCalc(Task::Action, double);

    float getError(){
        return p();
    }

    float Ki(){
        return ki;
    }

    float Kp(){
        return kp;
    }
    float Kd(){
        return kd;
    }

    float get_i(){
        return i;
    }

    float get_d(){
        return d;
    }

    float update(float);

    void reset(){
        p_buffer=std::vector <float>(bufferSize,0);
        i=0;
        d=0;
        mf.buffer=std::vector<float>(mf.kernelSize,0);
    }

    float kp=0.075;    
    float kd=0, ki=0;
    private:


    float p(){
        float sum=0;
        for (int j=0;j<p_buffer.size(); j++){
            sum+=p_buffer[j];
        }
        return sum;
    }
    int correction_rate=2; //Hz
    int bufferSize= correction_rate*(FPS/MOTOR_CALLBACK);
    std::vector <float>p_buffer=std::vector <float>(bufferSize,0);
    float i=0, d=0;
    float tolerance_upper=0.01, tolerance_lower=-0.01;

    struct MedianFilter{
        int kernelSize=3;
        std::vector<float>buffer=std::vector<float>(kernelSize,0);

        float get_median(){
            std::vector <float> tmp=buffer;
            std::sort(tmp.begin(), tmp.end());
            return tmp[int(kernelSize/2)];
        }
    }mf;
    

};


class MotorCallback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
    unsigned int m_step=0;
    char a='0';
    int n_l=0, n_r=0, n_s=0;
public:
Task t=Task(STOP);


MotorCallback(){}

void step( AlphaBot &);

unsigned int getStep(){
    return m_step;
}

void setK(float f, char k){
    if (k=='p'){
    t.correct.kp=f;  
    }
    else if (k=='i'){
        t.correct.ki=f;
    }
    else if (k=='d'){
        t.correct.kd=f;
    }

   
}

void setA(char a='0');

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