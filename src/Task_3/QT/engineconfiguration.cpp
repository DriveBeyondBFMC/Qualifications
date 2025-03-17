#include "engineconfiguration.h"

EngineConfiguration::EngineConfiguration()
{

}
void EngineConfiguration::setEngineRPM(int i_engineRPM) {
    if(i_engineRPM > 0){
        m_engineRPM = i_engineRPM;
    }
}
int EngineConfiguration::getEngineRPM() const {
    return m_engineRPM;
};

void EngineConfiguration::setSpeed(int i_speed) {
    if(i_speed > 0){
        m_speed = i_speed;
    }
};
int EngineConfiguration::getSpeed() const {
    return m_speed;
};


void EngineConfiguration::accelerate(bool i_acceler) {
    if(i_acceler == true) {
        isBraking = false;
        isAccelerating = true;
        emit speedChanged();
        emit engineRPMChanged();
    }
    else {
        isAccelerating = false;
        emit speedChanged();
        emit engineRPMChanged();
    }
}
void EngineConfiguration::applyBrake(bool i_breaks) {
    if(i_breaks == true){
        isBraking = true;
        isAccelerating = false;
        emit speedChanged();
        emit engineRPMChanged();
    }else{
        isBraking = false;
        emit speedChanged();
        emit engineRPMChanged();

    }
}
