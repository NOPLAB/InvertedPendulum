#ifndef APP_INC_LPF_HPP_
#define APP_INC_LPF_HPP_

#include <cmath>

class LowPassFilter {
public:
    LowPassFilter(float gain, float time_constant, float dt);
    
    void setGain(float gain);
    void setTimeConstant(float time_constant);
    void setDt(float dt);
    void reset();
    
    float update(float input);
    
    float getGain() const { return gain_; }
    float getTimeConstant() const { return time_constant_; }
    float getDt() const { return dt_; }
    float getOutput() const { return output_; }
    
private:
    float gain_;
    float time_constant_;
    float dt_;
    float alpha_;
    float output_;
    bool first_call_;
    
    void updateAlpha();
};

#endif /* APP_INC_LPF_HPP_ */