#ifndef ORIGINIMU_HPP
#define ORIGINIMU_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>
#define I2C_BUFFER_LEN 8
#define I2C0           5

/* The mbed library */
#include <mbed.h>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>
#include <drivers/mpu9255.hpp>


namespace periodics
{
   /**
    * @brief Class originimu
    *
    */
    class CHeadingimu: public utils::CTask
    {
        public:
            /* Constructor */
            CHeadingimu(
                std::chrono::milliseconds f_period,
                UnbufferedSerial& f_serial,
                PinName SDA,
                PinName SCL
            );
            /* Destructor */
            ~CHeadingimu();
            /* The API is used as SPI bus write */
            static void MPU9255_I2C_bus_write_byte(u8 address, u8 subAddress, u8 data);

            static u8 MPU9255_I2C_bus_read_byte(u8 address, u8 subAddress);
            /* The API is used as I2C bus read */
            static void MPU9255_I2C_bus_read_bytes(u8 address, u8 subAddress, u8 count, u8* dest);

            /* TThe delay routine */
            static void MPU9255_delay_msek(u32 msek);
            /* Serial callback implementation */
            void serialCallbackHEADINGIMUcommand(char const * a, char * b);
            
            void headingimu_init();
            void get_raw();


            // void get_imu(struct acceleration_raw_data_t *acceleration_raw_data);

            private:
            /* private variables & method member */
            static I2C* i2c_instance;
            /*I2C init routine */
            virtual void I2C_routine(void);

            /* Run method */
            virtual void _run();

            struct mpu9255_t       mpu9255;
            u8 m_velocityStationaryCounter;
            u64 m_delta_time;
            u8 m_period;
            UnbufferedSerial&       m_serial;
            char buffer[100];
            uint32_t m_prev_ms;

            /** @brief Active flag  */
            bool m_isActive;

    }; // class CHeadingimu
}; // namespace periodics

#endif // ORIGINIMU_HPP
