#include "periodics/headingimu.hpp"

// TODO: Add your code here
namespace periodics
{
   /**
    * @brief Class constructor originimu
    *
    */
   I2C *periodics::CHeadingimu::i2c_instance = nullptr;

    CHeadingimu::CHeadingimu(
        std::chrono::milliseconds f_period,
        UnbufferedSerial &f_serial,
        PinName SDA,
        PinName SCL
    )
    : utils::CTask(f_period)
    , m_delta_time(f_period.count())
    , m_isActive(false)
    , m_serial(f_serial)
    , m_prev_ms(0)
    {
        /* constructor behaviour */
        if (m_delta_time < 150)
        {
            setNewPeriod(150);
            m_delta_time = 150;
        }

        s32 comres = MPU9255_ERROR;
        i2c_instance = new I2C(SDA, SCL);
        i2c_instance->frequency(400000);
        ThisThread::sleep_for(chrono::milliseconds(300));
        I2C_routine();
        // headingimu_init();
    }

    /** @brief  CHeadingimu class destructor
     */
    CHeadingimu::~CHeadingimu()
    {
        if(i2c_instance != nullptr)
        {
            delete i2c_instance;
            i2c_instance = nullptr;
        }
    }

    void CHeadingimu::get_raw() {
        // if(mpu9255_update()){
        //     if(current_ms > m_prev_ms + 50) {
        //         printf("pitch: %d yaw: %d\n",int16_t (mpu9255_get_pitch()), int16_t (mpu9255_get_yaw()));
        //         m_prev_ms = current_ms;
        //     }
        // }

        if(mpu9255_update(m_delta_time)){
                printf("pitch: %d yaw: %d\n",int16_t (mpu9255_get_pitch()), int16_t (mpu9255_get_yaw()));
            }
    }
 

    void CHeadingimu::serialCallbackHEADINGIMUcommand(char const * a, char * b) {
        uint8_t l_isActivate=0;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);

        if(1 == l_res){
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive=(l_isActivate>=1);
                bool_globalsV_distance_isActive = (l_isActivate>=1);
                sprintf(b,"1");
            }
            else{
                sprintf(b,"kl 15/30 is required!!");
            }
            
        }else{
            sprintf(b,"syntax error");
        }
    }


    void CHeadingimu::MPU9255_I2C_bus_write_byte(u8 address, u8 subAddress, u8 data){
        char cmd[2];
        cmd[0] = subAddress;  
        cmd[1] = data;        
      
        int result = i2c_instance->write(address << 1, cmd, 2);  
        if (result != 0) {
            printf("Writing error!\n");
        }
    }

    void CHeadingimu::headingimu_init (){
        if(!mpu9255_setup(0x68, &mpu9255)){
            printf("Error connection\n");
        }
        printf("Accel Gyro calibration will start in 5sec\n");
        mpu9255_verbose(true);
        mpu9255_calibrate_accel_gyro();
        printf("Mag calibration will start in 5sec\n");
        mpu9255_calibrate_mag();
    }



    u8 CHeadingimu::MPU9255_I2C_bus_read_byte(u8 address, u8 subAddress){
        uint8_t data = 0;  
        char cmd[1];
        cmd[0] = subAddress; 
      
        int result = i2c_instance->write(address << 1, cmd, 1); 
        if (result != 0) {
            printf("Writing error!\n");
            return 0;          
        }
      
        result = i2c_instance->read(address << 1, cmd, 1); 
        if (result == 0) {
            data = cmd[0];  
        } else {
            printf("Reading error!\n");
        }
        return data; 
    }

    void CHeadingimu::MPU9255_I2C_bus_read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest){
        char cmd[1] = {subAddress};
        int i2c_err_ = i2c_instance->write(address << 1, cmd, 1, true);  
        if (i2c_err_ != 0) {
            printf("Writing error!\n"); 
            return;
        }
        i2c_err_ = i2c_instance->read(address << 1, (char*)dest, count);  
        if (i2c_err_ != 0) {
           printf("Reading error!\n");
        }
    }

    void CHeadingimu::I2C_routine(void) {
        mpu9255.bus_write_byte  = MPU9255_I2C_bus_write_byte;
        mpu9255.bus_read_bytes  = MPU9255_I2C_bus_read_bytes;
        mpu9255.bus_read_byte   = MPU9255_I2C_bus_read_byte;


        mpu9255.delay_msec      = MPU9255_delay_msek;
        mpu9255.dev_addr        = 0x68 << 1;
        mpu9255.mag_addr        = 0x48 << 1;
        ThisThread::sleep_for(chrono::milliseconds(300));
    }


    void CHeadingimu::MPU9255_delay_msek(u32 msek)
    {
        /*Here you can write your own delay routine*/
        ThisThread::sleep_for(chrono::milliseconds(msek));
    }

    // void CHeadingimu::get_imu(struct acceleration_raw_data_t *acceleration_raw_data) {
    //     mpu9255_read_acc(acceleration_raw_data);
    // }
    /* Run method */
    void CHeadingimu::_run()
    {
        if(!m_isActive) return;
        printf("Hello I'm heading imu\n");
        // get_raw();

    }

}; // namespace periodics