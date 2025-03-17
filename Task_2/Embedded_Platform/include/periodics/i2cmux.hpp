#ifndef I2CMUX_HPP
#define I2CMUX_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>
#include "distance.hpp"
#include "headingimu.hpp"
#include <periodics/imu.hpp>

#include <chrono>

namespace periodics
{
   /**
    * @brief Class i2cmux
    *
    */

    class ICI2cmux {
        public:
            virtual s32 get_imu_value();
            virtual s32 get_position(); 
    };

    class CI2cmux: public utils::CTask, periodics::ICI2cmux
    {
        public:
            /* Constructor */
            CI2cmux(
                std::chrono::milliseconds   f_period,
                UnbufferedSerial            &f_serial,
                periodics::ICImu&           f_imu,
                PinName                     SDA,
                PinName                     SCL
            );
            /* Destructor */
            static int8_t pca9548_i2c_write(uint8_t addr8bit_pca, uint8_t channel);
            void serialCallbackI2CMUXcommand(char const * a, char * b);
            s32 get_imu_value();
            s32 get_position(); 
            ~CI2cmux();

            
        private:
            /* private variables & method member */
            static I2C* i2c_instance;
            /* Run method */
            virtual void _run();
            /** @brief Active flag  */
            bool m_isActive;
            uint64_t m_delta_time;
            uint16_t m_distance_value;
            uint16_t m_distance_value_2;
            UnbufferedSerial&       m_serial;
            periodics::CDistance    m_distance;
            periodics::ICImu&       m_imu;
            char buffer[100];
            uint32_t m_prev_ms;

    }; // class CI2cmux
}; // namespace periodics

#endif // I2CMUX_HPP
