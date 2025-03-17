#include "periodics/i2cmux.hpp"
const int addr8bit_pca = 0x70 << 1; 
// TODO: Add your code here
namespace periodics
{
   /**
    * @brief Class constructor i2cmux
    *
    */
    I2C *periodics::CI2cmux::i2c_instance = nullptr;
    
    CI2cmux::CI2cmux(
        std::chrono::milliseconds   f_period,
        UnbufferedSerial&           f_serial,
        periodics::ICImu&           f_imu,
        PinName SDA,
        PinName SCL
    )
    : utils::CTask(f_period)
    , m_delta_time(f_period.count())
    , m_serial(f_serial)
    , m_isActive(false)
    , m_distance_value(0)
    , m_distance(f_period * 200, f_serial, SDA, SCL)
    , m_prev_ms(0)
    , m_imu(f_imu)
    {
        /* constructor behaviour */

        if (m_delta_time < 150)
        {
            setNewPeriod(150);
            m_delta_time = 150;
        }
        i2c_instance = new I2C(SDA, SCL);
        i2c_instance->frequency(400000);
        ThisThread::sleep_for(chrono::milliseconds(300));
        pca9548_i2c_write(addr8bit_pca, 0);
        m_distance.distance_init();
        pca9548_i2c_write(addr8bit_pca, 1);
        m_distance.distance_init();

    }

    int8_t CI2cmux::pca9548_i2c_write(uint8_t addr8bit_pca, uint8_t channel){
        // s32 VL53L1X_iERROR = VL53L1X_INIT_VALUE;
        uint8_t buffer = 1 << channel;
        if (i2c_instance->write(addr8bit_pca, (const char*)&buffer, 1, false)) {
            // VL53L1X_iERROR = VL53L1X_ERROR; 
            return -1;
        }
        return 0;

    }
    void CI2cmux::serialCallbackI2CMUXcommand(char const * a, char * b) {
        uint8_t l_isActivate=0;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);

        if(1 == l_res){
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive=(l_isActivate>=1);
                bool_globalsV_i2c_mux_isActive = (l_isActivate>=1);
                sprintf(b,"1");
            }
            else{
                sprintf(b,"kl 15/30 is required!!");
            }
            
        }else{
            sprintf(b,"syntax error");
        }
    }

    /** @brief  CI2cmux class destructor
     */
    CI2cmux::~CI2cmux()
    {
        if(i2c_instance != nullptr)
        {
            delete i2c_instance;
            i2c_instance = nullptr;
        }
    }


    /* Run method */
    void CI2cmux::_run()
    {
        // if(!m_isActive) return;
        pca9548_i2c_write(addr8bit_pca, 0);
        m_distance.get_distance(&m_distance_value);

        pca9548_i2c_write(addr8bit_pca, 1);
        m_distance.get_distance(&m_distance_value_2);
        // m_originimu.test();
        snprintf(buffer, sizeof(buffer), "distance:%d %d\r\n",m_distance_value, m_distance_value_2);
        m_serial.write(buffer,strlen(buffer));

    }

    s32 CI2cmux::get_imu_value() {
        return 0;
    }

    s32 CI2cmux::get_position() {
        return 0;
    }

}; // namespace periodics