#ifndef BATTERYMONITOR_HPP
#define BATTERYMONITOR_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>

#define I2C_BUFFER_LEN 8
#define I2C0           5

/* The mbed library */
#include <mbed.h>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>
#include <drivers/ads1115.hpp>

namespace periodics
{
   /**
    * @brief Class batterymonitor
    *
    */
    class CBatterymonitor: public utils::CTask
    {
        public:
            /* Constructor */
            CBatterymonitor(
                std::chrono::milliseconds f_period,
                UnbufferedSerial& f_serial,
                PinName SDA,
                PinName SCL
            );

            static s8 ADS1115_I2C_bus_write(u8 dev_addr, u8 reg_addr, u16 reg_data);

            static s8 ADS1115_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data);
            
            static void ADS1115_delay_msek(u32 msek);

            void serialCallbackBATTERYMONITORcommand(char const * a, char * b);
            /* Destructor */
            ~CBatterymonitor();
        private:
            /* private variables & method member */
            static I2C* i2c_instance;
            /* private variables & method member */
            virtual void I2C_routine(void);
            /* Run method */
            virtual void        _run();
            struct ads1115_t ads1115;
            u8 m_velocityStationaryCounter;
            u64 m_delta_time;
            u8 m_period;
            UnbufferedSerial& m_serial;
            /** @brief Active flag  */
            bool m_isActive;

    }; // class CBatterymonitor
}; // namespace periodics

#endif // BATTERYMONITOR_HPP
