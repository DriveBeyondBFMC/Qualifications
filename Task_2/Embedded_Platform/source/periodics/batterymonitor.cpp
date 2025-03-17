#include "periodics/batterymonitor.hpp"

// TODO: Add your code here
namespace periodics
{
   /**
    * @brief Class constructor batterymonitor
    *
    */
    I2C *periodics::CBatterymonitor::i2c_instance = nullptr;

    CBatterymonitor::CBatterymonitor(
        std::chrono::milliseconds f_period,
        UnbufferedSerial &f_serial,
        PinName SDA,
        PinName SCL
    )
    : utils::CTask(f_period)
    , m_delta_time(f_period.count())
    , m_isActive(false)
    , m_serial(f_serial)
    {
        /* constructor behaviour */
        if (m_delta_time < 150)
        {
            setNewPeriod(150);
            m_delta_time = 150;
        }

        s32 comres = ADS1115_ERROR;
        i2c_instance = new I2C(SDA, SCL);
        i2c_instance->frequency(400000);
        ThisThread::sleep_for(chrono::milliseconds(300));
        I2C_routine();
    }

    /** @brief  CBatterymonitor class destructor
     */
    CBatterymonitor::~CBatterymonitor()
    {
        if(i2c_instance != nullptr)
        {
            delete i2c_instance;
            i2c_instance = nullptr;
        }
    }


    s8 CBatterymonitor::ADS1115_I2C_bus_write(u8 dev_addr, u8 reg_addr, u16 reg_data){
        s32 ADS1115_iERROR = ADS1115_INIT_VALUE;
        u8 l_buffer[3];
        l_buffer[0] = reg_addr;
        l_buffer[1] = reg_data >> 8;
        l_buffer[2] = reg_data & 0xFF;
        if (i2c_instance->write(dev_addr, (const char*)l_buffer, 3, false) != 0)
            ADS1115_iERROR = ADS1115_ERROR;
        return ADS1115_iERROR;
    }

    s8 CBatterymonitor::ADS1115_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data) {
        s32 ADS1115_iERROR = ADS1115_INIT_VALUE;
        char buffer[32]; 
        if (i2c_instance->write(dev_addr, (const char*)&reg_addr, 1) != 0)
            ADS1115_iERROR = ADS1115_ERROR;
        if (i2c_instance->read(dev_addr, (char*)buffer, 2) != 0)
            ADS1115_iERROR = ADS1115_ERROR;
        memcpy(reg_data, buffer, 2);
        return 0; 
    }
    


    void CBatterymonitor::serialCallbackBATTERYMONITORcommand(char const * a, char * b) {
        uint8_t l_isActivate=0;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);
        if(1 == l_res){
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive=(l_isActivate>=1);
                bool_globalsV_distance_isActive = (l_isActivate>=1);
                sprintf(b,"1");
            }
            else
                sprintf(b,"kl 15/30 is required!!");
        }else
            sprintf(b,"syntax error");
    }

    void CBatterymonitor::ADS1115_delay_msek(u32 msek)
    {
        /*Here you can write your own delay routine*/
        ThisThread::sleep_for(chrono::milliseconds(msek));
    }

    void CBatterymonitor::I2C_routine(void)
    {
        //Asigns function pointer for I2C write operation
        ads1115.bus_write = ADS1115_I2C_bus_write;
        ads1115.bus_read = ADS1115_I2C_bus_read;
        //Asign the delay function for timming operations
        ads1115.delay_msec = ADS1115_delay_msek;
        //Set the I2C device address 
        ads1115.dev_addr = 0x48 << 1;
        ThisThread::sleep_for(chrono::milliseconds(300));
    }

    /* Run method */
    void CBatterymonitor::_run()
    {
        /* Run method behaviour */
        if(!m_isActive) return;
        printf("battery\n");
        ADS1115_delay_msek(500);
    }

}; // namespace periodics