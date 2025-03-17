#ifndef VEHICLEDYNAMIC_HPP
#define VEHICLEDYNAMIC_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>
#include <periodics/imu.hpp>
#include <drivers/speedingmonitor.hpp>

namespace periodics
{
   /**
    * @brief Class vehicledynamic
    *
    */



    class CVehicledynamic: public utils::CTask
    {
        public:
            /* Constructor */
            CVehicledynamic(
                std::chrono::milliseconds   f_period,
                UnbufferedSerial&           f_serial,
                // UnbufferedSerial&           f_uwb_serial,
                periodics::ICImu&           f_imu,
                drivers::ISpeedingMonitor&  f_position
            );
            void init_coordinates();
            void car_coordinates(s32 imu, s32 position);
            void serialCallbackVEHICLEDYNAMICcommand(char const * a, char * b);
            /* Destructor */
            ~CVehicledynamic();
        private:
            /* private variables & method member */
            int buffer_index = 0;
            char buffer[100];
            char uwb_buffer[100];
            char received_data[100];
            bool data_ready = false;
            int16_t imu_reset_value = 0;
            int16_t new_value = 0;
            /* Run method */
            virtual void        _run();

            /** @brief Active flag  */
            bool                        m_isActive;
            periodics::ICImu&           m_imu;
            drivers::ISpeedingMonitor&  m_position;
            UnbufferedSerial&           m_serial;
            // UnbufferedSerial&           m_uwb_serial;
            int x = 0, y = 0, position_prev = 0, theta = 0;

            // struct bno055_t bno055;

    }; // class CVehicledynamic
}; // namespace periodics

#endif // VEHICLEDYNAMIC_HPP
