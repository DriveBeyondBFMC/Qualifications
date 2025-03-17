#include "enginconfiguratio.h"

EnginConfiguratio::EnginConfiguratio()
{

}
void EnginConfiguratio::setEngineRPM(int i_engineRPM) {
    if(i_engineRPM > 0){
        m_engineRPM = i_engineRPM;
    }
}
int EnginConfiguratio::getEngineRPM() const {
    return m_engineRPM;
};

void EnginConfiguratio::setSpeed(int i_speed) {
    if(i_speed > 0){
        m_speed = i_speed;
    }
};
int EnginConfiguratio::getSpeed() const {
    return m_speed;
};


void EnginConfiguratio::accelerate(bool acc);
void EnginConfiguratio::applyBrake(bool breaks);
