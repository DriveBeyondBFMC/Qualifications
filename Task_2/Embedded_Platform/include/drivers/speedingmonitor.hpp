#ifndef SPEEDINGMONITOR_HPP
#define SPEEDINGMONITOR_HPP
#define Kb 200 // 1/sample_time
#define SAMPLE_TIME 0.005
// TODO: Add your code here
#include <mbed.h>
#include <sstream>
namespace drivers
{
   /**
    * @brief Class speedingmonitor
    *
    */
    class ISpeedingMonitor
    {
        public:
            virtual int32_t getEncoderCount();
            virtual int32_t getEncoderVel();

    };

    class CSpeedingmonitor : public ISpeedingMonitor
    {
        public:
            /* Constructor */
            CSpeedingmonitor(
                PinName f_chanel_A_pin,
                PinName f_chanel_B_pin
            );
            /* Destructor */
            ~CSpeedingmonitor();
            void channelA_rise_fall();
            void channelB_rise_fall();
            int32_t getEncoderCount();
            int32_t getEncoderVel();
            void velocity_ticker_callback();
            // uint16_t PID_velocity(double desired_val, double current_val);
        private:
            /* private variables & method member */
            InterruptIn m_channel_A;  // Kênh A
            InterruptIn m_channel_B;
            // Ticker m_velocity_ticker;
            uint16_t wheel_diameter = 100;
            volatile int encoder_count = 0;
            float velocity = 0;
            Ticker m_velocity_ticker;
            int32_t vel_value;
            int count_value = 0;
            char buffer[50];
            volatile uint32_t count_velocity_value = 0;
            int32_t count_position_value = 0;
            uint8_t previous_state;
            int32_t position = 0; 

    }; // class CSpeedingmonitor
}; // namespace drivers

#endif // SPEEDINGMONITOR_HPP

