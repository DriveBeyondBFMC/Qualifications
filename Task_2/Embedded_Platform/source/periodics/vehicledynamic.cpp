#include "periodics/vehicledynamic.hpp"

// TODO: Add your code here
namespace periodics
{
   /**
    * @brief Class constructor vehicledynamic
    *
    */
    CVehicledynamic::CVehicledynamic(
        std::chrono::milliseconds   f_period,
        UnbufferedSerial&           f_serial,
        periodics::ICImu&           f_imu,
        drivers::ISpeedingMonitor&  f_position 
    )
    : utils::CTask(f_period)
    , m_imu(f_imu)
    , m_position(f_position)
    , m_serial(f_serial)
    // , m_uwb_serial(f_uwb_serial)
    {
        /* constructor behaviour */
    }

    /** @brief  CVehicledynamic class destructor
     */
    CVehicledynamic::~CVehicledynamic()
    {
    }

    void CVehicledynamic::serialCallbackVEHICLEDYNAMICcommand(char const * a, char * b){
        uint8_t l_isActivate=0;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);
        char *endPtr;
        int16_t number = strtol(a, &endPtr, 10);

        if (endPtr != a) { // Kiểm tra xem có số được đọc không
            if (number != 1 && number != 0){
                imu_reset_value = number;
                m_imu.reset_imu_value();
                new_value = imu_reset_value;
                return;
            }
        }


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


    

    void CVehicledynamic::car_coordinates(s32 imu, s32 position) {
        double imu_rad = imu * (M_PI / 180.0);  // Chuyển imu từ độ sang radian
        double delta_s = position - position_prev; // Quãng đường đã đi

        // Cập nhật vị trí theo heading (imu_rad chính là theta)
        x += (delta_s * cos(imu_rad)) * 1000;
        y += (delta_s * sin(imu_rad)) * 1000;

        // Cập nhật heading của xe trực tiếp từ imu
        theta = imu_rad;

        printf("x: %d, y: %d, theta: %d\n", x/1000, y/1000, imu);
        position_prev = position;
    }
    /* Run method */
    void CVehicledynamic::_run()
    {
        /* Run method behaviour */
        
        // if(!m_isActive) return;
        s32 imu_data = m_imu.get_imu_value(new_value);
        s32 position_data = m_position.getEncoderCount();
        snprintf(buffer, sizeof(buffer), "vel: %d \r\n",m_position.getEncoderVel());
        m_serial.write(buffer, strlen(buffer));
        car_coordinates(imu_data, position_data);
    }

}; // namespace periodics