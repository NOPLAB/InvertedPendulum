#ifndef MIT_ADAPTIVE_CONTROLLER_HPP_
#define MIT_ADAPTIVE_CONTROLLER_HPP_

class MitAdaptiveController {
private:
    float dt;
    float gamma[4];
    float adaptive_gains[4];
    float initial_gains[4];
    
    // Reference model parameters
    float reference_model_wn;
    float reference_model_zeta;
    float reference_model_a1;
    float reference_model_a2;
    
    // State variables
    float prev_reference_output;
    float prev_reference_velocity;
    float prev_plant_output;
    float prev_plant_velocity;
    float adaptation_error;
    float reference_output;
    float plant_output;
    float error_integral;
    float prev_error;
    
public:
    MitAdaptiveController(float dt, float gamma1, float gamma2, float gamma3, float gamma4);
    
    void setInitialGains(float k1, float k2, float k3, float k4);
    void setReferenceModel(float wn, float zeta);
    void setAdaptationRates(float gamma1, float gamma2, float gamma3, float gamma4);
    
    float update(float x, float dx, float theta, float dtheta, float reference);
    void reset();
    
    void getAdaptiveGains(float* gains);
    float getAdaptationError() const { return adaptation_error; }
    
private:
    void updateReferenceModel(float reference);
    void updateAdaptiveGains(float x, float dx, float theta, float dtheta);
    float calculatePlantOutput(float x, float dx, float theta, float dtheta);
};

#endif /* MIT_ADAPTIVE_CONTROLLER_HPP_ */