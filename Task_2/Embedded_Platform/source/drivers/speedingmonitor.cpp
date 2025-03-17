#include "drivers/speedingmonitor.hpp"
#include <stdio.h>
#include <cmath> 
#define PULSES_PER_REV   192     // Số xung trên 1 vòng quay động cơ
#define GEAR_RATIO       0.812   // Tỉ số truyền từ động cơ ra bánh xe
#define WHEEL_RADIUS     34.0    // Bán kính bánh xe (mm)
#define PI               3.1416  // Giá trị số pi
#define SAMPLE_TIME_MS   10      // Chu kỳ lấy mẫu (ms)



// TODO: Add your code here


namespace drivers
{
   /**
    * @brief Class constructor speedingmonitor
    *
    */
    CSpeedingmonitor::CSpeedingmonitor(
            PinName f_chanel_A_pin,
            PinName f_chanel_B_pin
        )
        : m_channel_A(f_chanel_A_pin)
        , m_channel_B(f_chanel_B_pin)
    {
        /* constructor behaviour */
        m_channel_A.rise(mbed::callback(this, &CSpeedingmonitor::channelA_rise_fall));
        m_channel_A.fall(mbed::callback(this, &CSpeedingmonitor::channelA_rise_fall));
        m_channel_B.rise(mbed::callback(this, &CSpeedingmonitor::channelB_rise_fall));
        m_channel_B.fall(mbed::callback(this, &CSpeedingmonitor::channelB_rise_fall));
        m_velocity_ticker.attach(mbed::callback(this, &CSpeedingmonitor::velocity_ticker_callback), 10ms);
    }

    CSpeedingmonitor::~CSpeedingmonitor()
    {

    }

    void CSpeedingmonitor::velocity_ticker_callback(){
        float tmp = count_velocity_value;
        double pulse_per_sec = tmp * (1000.0 / SAMPLE_TIME_MS);  // Quy đổi ra xung/giây
        double wheel_revolutions_per_sec = (pulse_per_sec / PULSES_PER_REV) * GEAR_RATIO;
        double wheel_circumference = (2 * PI * WHEEL_RADIUS) / 10.0;
        double speed_mm_per_sec = wheel_revolutions_per_sec * wheel_circumference;
        velocity = speed_mm_per_sec;
        count_velocity_value = 0;
    }
    


    void CSpeedingmonitor::channelB_rise_fall(){
        unsigned char state0 = 0;
        state0 = (state0 << 1) | m_channel_A.read();
        state0 = (state0 << 1) | m_channel_B.read();
        state0 &= 0x03;  // Giữ lại 2 bit cuối
        switch (state0) {
            case 0:
                if (previous_state == 1) count_value++;
                else count_value--;
                break;
            case 1:
                if (previous_state == 3) count_value++;
                else count_value--;
                break;
            case 2:
                if (previous_state == 0) count_value++;
                else count_value--;
                break;
            case 3:
                if (previous_state == 2) count_value++;
                else count_value--;
                break;
        }
        previous_state = state0;
        count_velocity_value++;
       
    }

    void CSpeedingmonitor::channelA_rise_fall(){
        unsigned char state1;
        state1 = (state1 << 1) | m_channel_A.read();
        state1 = (state1 << 1) | m_channel_B.read();
        state1 &= 0x03;
        switch (state1) {
            case 0:
                if (previous_state == 1) count_value++;
                else count_value--;
                break;
            case 1:
                if (previous_state == 3) count_value++;
                else count_value--;
                break;
            case 2:
                if (previous_state == 0) count_value++;
                else count_value--;
                break;
            case 3:
                if (previous_state == 2) count_value++;
                else count_value--;
                break;
        }
        previous_state = state1;
        count_velocity_value++;
    }


    int32_t CSpeedingmonitor::getEncoderCount() {
        double position = (count_value * 0.0856) * 1000;
        int32_t tmp = (int32_t) round(position);  
        return tmp / 1000;
    }

    int32_t CSpeedingmonitor::getEncoderVel() {
        return int32_t(velocity);
    }


}; // namespace drivers