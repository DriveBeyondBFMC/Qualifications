#ifndef MPU9255_HPP
#define MPU9255_HPP

#include "quaternionfilter.h"
/*signed integer types*/
typedef signed char s8;             /**< used for signed 8bit */
typedef signed short int s16;       /**< used for signed 16bit */
typedef signed int s32;             /**< used for signed 32bit */
typedef signed long long int s64;   /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8;           /**< used for unsigned 8bit */
typedef unsigned short int u16;     /**< used for unsigned 16bit */
typedef unsigned int u32;           /**< used for unsigned 32bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */



#define MPU9255_WR_BYTE_FUNC_PTR      void \
        (*bus_write_byte)(u8, u8, u8)
#define MPU9255_RD_BYTE_FUNC_PTR       	  u8 \
        (*bus_read_byte)(u8, u8)
#define MPU9255_RD_BYTES_FUNC_PTR      void \
        (*bus_read_bytes)(u8, u8, u8, u8*)


#define MPU9255_BUS_WRITE_BYTE_FUNC(address, subAddress, data) \
  bus_write_byte(address, subAddress, data) 

#define MPU9255_BUS_READ_BYTE_FUNC(address, subAddress) \
  bus_read(address, subAddress) 

#define MPU9255_BUS_READ_BYTES_FUNC(address, subAddress, count, dest) \
  bus_read_bytes(address, subAddress, count, dest) 



  #define MPU9255_RETURN_FUNCTION_TYPE  s8
  #define AK8963_WHO_AM_I         0x00 
  #define AK8963_INFO             0x01
  #define AK8963_ST1              0x02  
  #define AK8963_XOUT_L	        0x03  
  #define AK8963_XOUT_H	        0x04
  #define AK8963_YOUT_L	        0x05
  #define AK8963_YOUT_H	        0x06
  #define AK8963_ZOUT_L	        0x07
  #define AK8963_ZOUT_H	        0x08
  #define AK8963_ST2              0x09  
  #define AK8963_CNTL             0x0A  
  #define AK8963_ASTC             0x0C  
  #define AK8963_I2CDIS           0x0F  
  #define AK8963_ASAX             0x10  
  #define AK8963_ASAY             0x11  
  #define AK8963_ASAZ             0x12  
  #define SELF_TEST_X_GYRO        0x00
  #define SELF_TEST_Y_GYRO        0x01
  #define SELF_TEST_Z_GYRO        0x02
  #define SELF_TEST_X_ACCEL       0x0D
  #define SELF_TEST_Y_ACCEL       0x0E
  #define SELF_TEST_Z_ACCEL       0x0F
  #define SELF_TEST_A             0x10
  #define XG_OFFSET_H             0x13  
  #define XG_OFFSET_L             0x14
  #define YG_OFFSET_H             0x15
  #define YG_OFFSET_L             0x16
  #define ZG_OFFSET_H             0x17
  #define ZG_OFFSET_L             0x18
  #define SMPLRT_DIV              0x19
  #define MPU_CONFIG              0x1A
  #define GYRO_CONFIG             0x1B
  #define ACCEL_CONFIG            0x1C
  #define ACCEL_CONFIG2           0x1D
  #define LP_ACCEL_ODR            0x1E
  #define WOM_THR                 0x1F
  #define MOT_DUR                 0x20  
  #define ZMOT_THR                0x21  
  #define ZRMOT_DUR               0x22  
  #define FIFO_EN                 0x23
  #define I2C_MST_CTRL            0x24
  #define I2C_SLV0_ADDR           0x25
  #define I2C_SLV0_REG            0x26
  #define I2C_SLV0_CTRL           0x27
  #define I2C_SLV1_ADDR           0x28
  #define I2C_SLV1_REG            0x29
  #define I2C_SLV1_CTRL           0x2A
  #define I2C_SLV2_ADDR           0x2B
  #define I2C_SLV2_REG            0x2C
  #define I2C_SLV2_CTRL           0x2D
  #define I2C_SLV3_ADDR           0x2E
  #define I2C_SLV3_REG            0x2F
  #define I2C_SLV3_CTRL           0x30
  #define I2C_SLV4_ADDR           0x31
  #define I2C_SLV4_REG            0x32
  #define I2C_SLV4_DO             0x33
  #define I2C_SLV4_CTRL           0x34
  #define I2C_SLV4_DI             0x35
  #define I2C_MST_STATUS          0x36
  #define INT_PIN_CFG             0x37
  #define INT_ENABLE              0x38
  #define DMP_INT_STATUS          0x39  // Check DMP interrupt
  #define INT_STATUS              0x3A
  #define ACCEL_XOUT_H            0x3B
  #define ACCEL_XOUT_L            0x3C
  #define ACCEL_YOUT_H            0x3D
  #define ACCEL_YOUT_L            0x3E
  #define ACCEL_ZOUT_H            0x3F
  #define ACCEL_ZOUT_L            0x40
  #define TEMP_OUT_H              0x41
  #define TEMP_OUT_L              0x42
  #define GYRO_XOUT_H             0x43
  #define GYRO_XOUT_L             0x44
  #define GYRO_YOUT_H             0x45
  #define GYRO_YOUT_L             0x46
  #define GYRO_ZOUT_H             0x47
  #define GYRO_ZOUT_L             0x48
  #define EXT_SENS_DATA_00        0x49
  #define EXT_SENS_DATA_01        0x4A
  #define EXT_SENS_DATA_02        0x4B
  #define EXT_SENS_DATA_03        0x4C
  #define EXT_SENS_DATA_04        0x4D
  #define EXT_SENS_DATA_05        0x4E
  #define EXT_SENS_DATA_06        0x4F
  #define EXT_SENS_DATA_07        0x50
  #define EXT_SENS_DATA_08        0x51
  #define EXT_SENS_DATA_09        0x52
  #define EXT_SENS_DATA_10        0x53
  #define EXT_SENS_DATA_11        0x54
  #define EXT_SENS_DATA_12        0x55
  #define EXT_SENS_DATA_13        0x56
  #define EXT_SENS_DATA_14        0x57
  #define EXT_SENS_DATA_15        0x58
  #define EXT_SENS_DATA_16        0x59
  #define EXT_SENS_DATA_17        0x5A
  #define EXT_SENS_DATA_18        0x5B
  #define EXT_SENS_DATA_19        0x5C
  #define EXT_SENS_DATA_20        0x5D
  #define EXT_SENS_DATA_21        0x5E
  #define EXT_SENS_DATA_22        0x5F
  #define EXT_SENS_DATA_23        0x60
  #define MOT_DETECT_STATUS       0x61
  #define I2C_SLV0_DO             0x63
  #define I2C_SLV1_DO             0x64
  #define I2C_SLV2_DO             0x65
  #define I2C_SLV3_DO             0x66
  #define I2C_MST_DELAY_CTRL      0x67
  #define SIGNAL_PATH_RESET       0x68
  #define MOT_DETECT_CTRL         0x69
  #define USER_CTRL               0x6A 
  #define PWR_MGMT_1              0x6B 
  #define PWR_MGMT_2              0x6C
  #define DMP_BANK                0x6D  
  #define DMP_RW_PNT              0x6E  
  #define DMP_REG                 0x6F  
  #define DMP_REG_1               0x70
  #define DMP_REG_2               0x71
  #define FIFO_COUNTH             0x72
  #define FIFO_COUNTL             0x73
  #define FIFO_R_W                0x74
  #define WHO_AM_I_MPU9250        0x75 
  #define XA_OFFSET_H             0x77
  #define XA_OFFSET_L             0x78
  #define YA_OFFSET_H             0x7A
  #define YA_OFFSET_L             0x7B
  #define ZA_OFFSET_H             0x7D
  #define ZA_OFFSET_L             0x7E
  
  
  #define MPU9255_MDELAY_DATA_TYPE      u32
  #define MPU9255_ERROR                ((s8) - 1)
  #define MPU9255_SUCCESS              ((u8)0)
  #define MPU9255_INIT_VALUE           ((u8)0)
  
  
  #define MPU9255_DELAY_PARAM_TYPES       u32
  
  #define MPU9255_DELAY_FUNC  (delay_in_msec) \
      delay_func(delay_in_msec)
  
  #define MPU9255_DELAY_RETURN_TYPE       void
  
  
enum class accel_fs_sel_e {
  A2G,
  A4G,
  A8G,
  A16G
};
enum class gyro_fs_sel_e {
  G250DPS,
  G500DPS,
  G1000DPS,
  G2000DPS
};
enum class mag_output_bits_e {
  M14BITS,
  M16BITS
};

enum class fifo_sample_rate_e : uint8_t {
  SMPL_1000HZ,
  SMPL_500HZ,
  SMPL_333HZ,
  SMPL_250HZ,
  SMPL_200HZ,
  SMPL_167HZ,
  SMPL_143HZ,
  SMPL_125HZ,
};

enum class gyro_dlpf_cfg_e : uint8_t {
  DLPF_250HZ,
  DLPF_184HZ,
  DLPF_92HZ,
  DLPF_41HZ,
  DLPF_20HZ,
  DLPF_10HZ,
  DLPF_5HZ,
  DLPF_3600HZ,
};

enum class accel_dlpf_cfg_e : uint8_t {
  DLPF_218HZ_0,
  DLPF_218HZ_1,
  DLPF_99HZ,
  DLPF_45HZ,
  DLPF_21HZ,
  DLPF_10HZ,
  DLPF_5HZ,
  DLPF_420HZ,
};


struct mpu9255_setting_t {
  accel_fs_sel_e      accel_fs_sel                {accel_fs_sel_e::A16G};
  gyro_fs_sel_e       gyro_fs_sel                 {gyro_fs_sel_e::G2000DPS};
  mag_output_bits_e   mag_output_bits             {mag_output_bits_e::M16BITS};
  fifo_sample_rate_e  fifo_sample_rate            {fifo_sample_rate_e::SMPL_200HZ};
  gyro_dlpf_cfg_e     gyro_dlpf_cfg               {gyro_dlpf_cfg_e::DLPF_41HZ};
  accel_dlpf_cfg_e    accel_dlpf_cfg              {accel_dlpf_cfg_e::DLPF_45HZ};
  uint8_t             accel_fchoice               {0x01};
  uint8_t             gyro_fchoice                {0x03};
};


struct sensor_resolution_t {
  float acc {0.f};  
  float gyro {0.f};  
  float mag {0.f};  
};

struct sensor_calibration_t {
  float acc_bias[3]           {0.f, 0.f, 0.f};
  float gyro_bias[3]          {0.f, 0.f, 0.f};
  float mag_bias_factory[3]   {0.f, 0.f, 0.f};
  float mag_bias[3]           {0.f, 0.f, 0.f};
  float mag_scale[3]          {1.f, 1.f, 1.f};
  float magnetic_declination  {-7.51f};
};

struct sensor_temperature_t {
  int16_t raw_count {0};
  float value {0.f};
};

struct sensor_self_test_t {
  float result[6] {0.f};
};

struct sensor_data_t {
  float acc[3] {0.f, 0.f, 0.f};
  float gyro[3] {0.f, 0.f, 0.f};
  float mag[3] {0.f, 0.f, 0.f};
  float quaternion[4] {1.0f, 0.0f, 0.0f, 0.0f};
  float rpy[3] {0.f, 0.f, 0.f};
  float lin_acc[3] {0.f, 0.f, 0.f};
  QuaternionFilter quat_filter;
  size_t n_filter_iter {1};
};

// Trạng thái hệ thống
struct system_status_t {
  bool has_connected {false};
  bool b_ahrs {true};
  bool b_verbose {false};
  uint8_t i2c_err_ {0};
};

// Cấu hình cảm biến
struct sensor_config_t {
  static constexpr uint8_t mag_mode {0x06}; // 0x02 for 8 Hz, 0x06 for 100 Hz
};










struct mpu9255_t
{
    u8 chip_id; 
    u8 dev_addr; 
    u8 mag_addr;

    MPU9255_WR_BYTE_FUNC_PTR;
    MPU9255_RD_BYTE_FUNC_PTR; 
    MPU9255_RD_BYTES_FUNC_PTR; 

    void (*delay_msec)(MPU9255_MDELAY_DATA_TYPE);
};


bool  mpu9255_setup(const uint8_t addr, mpu9255_t* mpu_9255);
void  mpu9255_set_mag_bias(const float x, const float y, const float z);
void  mpu9255_set_mag_scale(const float x, const float y, const float z);
void  mpu9255_sleep(bool b);
void  mpu9255_verbose(const bool b);
void  mpu9255_ahrs(const bool b);
void  mpu9255_collect_mag_data_to(float* m_bias, float* m_scale);
void  mpu9255_calibrate_mag_impl();
void  mpu9255_calibrate_mag();
void  mpu9255_delay(int ms);
void  mpu9255_calibrate_accel_gyro();
void  mpu9255_calibrate_acc_gyro_impl();
void  mpu9255_set_acc_gyro_to_calibration();
void  mpu9255_collect_acc_gyro_data_to(float* a_bias, float* g_bias);
bool  mpu9255_is_connected();
bool  mpu9255_is_connected_MPU9255();
bool  mpu9255_is_connected_AK8963();
bool  mpu9255_is_sleeping();
bool  mpu9255_available();
bool  mpu9255_update(u64 delta_time);
float mpu9255_get_roll();
float mpu9255_get_pitch();
float mpu9255_get_yaw();
void  mpu9255_set_acc_bias(const float x, const float y, const float z);
void  mpu9255_select_filter(QuatFilterSel sel);
void  mpu9255_set_filter_iterations(const size_t n);
void  mpu9255_init_MPU9255();
void  mpu9255_init_AK8963();
void  mpu9255_update_rpy(float qw, float qx, float qy, float qz);
void  mpu9255_update_accel_gyro();
void  mpu9255_read_accel_gyro(int16_t* destination);
void  mpu9255_update_mag();
bool  mpu9255_read_mag(int16_t* destination);
int16_t mpu9255_read_temperature_data();
float   mpu9255_get_acc_resolution(const accel_fs_sel_e accel_af_sel);
float   mpu9255_get_gyro_resolution(const gyro_fs_sel_e gyro_fs_sel);
void    mpu9255_write_accel_offset();
void    mpu9255_write_gyro_offset();
float   mpu9255_get_mag_resolution(const mag_output_bits_e mag_output_bits);
void    mpu9255_write_byte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t mpu9255_read_byte(uint8_t address, uint8_t subAddress);
void    mpu9255_read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest);


static constexpr uint8_t MPU9250_WHOAMI_DEFAULT_VALUE {0x71};
static constexpr uint8_t MPU9255_WHOAMI_DEFAULT_VALUE {0x73};
static constexpr uint8_t MPU6500_WHOAMI_DEFAULT_VALUE {0x70};


static constexpr uint16_t CALIB_GYRO_SENSITIVITY {131};     
static constexpr uint16_t CALIB_ACCEL_SENSITIVITY {16384};  // LSB/g

#define DEG_TO_RAD 0.017453292519943295


#endif // MPU9255_HPP
