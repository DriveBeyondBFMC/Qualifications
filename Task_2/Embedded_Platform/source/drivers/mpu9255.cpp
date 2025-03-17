#include "drivers/mpu9255.hpp"

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *) 0)
#endif
#endif

mpu9255_t               *p_mpu9255;
sensor_calibration_t    sensor_calibration;
sensor_temperature_t    sensor_temperature;
sensor_self_test_t      sensor_self_test;
sensor_data_t           sensor_data;
system_status_t         system_status;
sensor_config_t         sensor_config;
sensor_resolution_t     sensor_resolution;
mpu9255_setting_t       setting;

static constexpr uint8_t    MPU9250_DEFAULT_ADDRESS {0x68};  // Device address when ADO = 0
static constexpr uint8_t    AK8963_ADDRESS {0x0C};           //  Address of magnetometer
static constexpr uint8_t    AK8963_WHOAMI_DEFAULT_VALUE {0x48};
uint8_t mpu_i2c_addr        {MPU9250_DEFAULT_ADDRESS};

bool mpu9255_setup(const uint8_t addr, mpu9255_t *mpu9255) {
  
  if ((addr < MPU9250_DEFAULT_ADDRESS) || (addr > MPU9250_DEFAULT_ADDRESS + 7)) {
      printf("I2C address 0x");
      printf("is not valid for MPU. Please check your I2C address.");
      return false;
  }
  mpu_i2c_addr = addr;
  p_mpu9255 = mpu9255;

  if (mpu9255_is_connected_MPU9255()) {
      mpu9255_init_MPU9255();
      if (mpu9255_is_connected_AK8963())
          mpu9255_init_AK8963();
      else {
          if (system_status.b_verbose) {printf("Could not connect to AK8963");}
          system_status.has_connected = false;
          return false;
      }
  } else {
      if (system_status.b_verbose) {printf("Could not connect to MPU9250");}
      system_status.has_connected = false;
      return false;
  }
  system_status.has_connected = true;
  return true;
}

bool mpu9255_read_mag(int16_t* destination) {
  const uint8_t st1 = p_mpu9255->bus_read_byte(AK8963_ADDRESS, AK8963_ST1);
  if (st1 & 0x01) {                                                    // wait for magnetometer data ready bit to be set
      uint8_t raw_data[7];                                             // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
      p_mpu9255->bus_read_bytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &raw_data[0]);      // Read the six raw data and ST2 registers sequentially into data array
      if (sensor_config.mag_mode == 0x02 || sensor_config.mag_mode == 0x04 || sensor_config.mag_mode == 0x06) {  // continuous or external trigger read mode
          if ((st1 & 0x02) != 0)                                       // check if data is not skipped
              return false;                                            // this should be after data reading to clear DRDY register
      }
      uint8_t c = raw_data[6];                                         // End data read by reading ST2 register
      if (!(c & 0x08)) {                                               // Check if magnetic sensor overflow set, if not then report data
          destination[0] = ((int16_t)raw_data[1] << 8) | raw_data[0];  // Turn the MSB and LSB into a signed 16-bit value
          destination[1] = ((int16_t)raw_data[3] << 8) | raw_data[2];  // Data stored as little Endian
          destination[2] = ((int16_t)raw_data[5] << 8) | raw_data[4];
          return true;
      }
  }
  return false;
}



void mpu9255_set_mag_bias(const float x, const float y, const float z) {
  sensor_calibration.mag_bias[0] = x;
  sensor_calibration.mag_bias[1] = y;
  sensor_calibration.mag_bias[2] = z;
}

void mpu9255_set_mag_scale(const float x, const float y, const float z) {
  sensor_calibration.mag_scale[0] = x;
  sensor_calibration.mag_scale[1] = y;
  sensor_calibration.mag_scale[2] = z;
}


void mpu9255_sleep(bool b) {
  uint8_t c = p_mpu9255->bus_read_byte(mpu_i2c_addr, PWR_MGMT_1);  // read the value, change sleep bit to match b, write byte back to register
  if (b)
      c = c | 0x40;  // sets the sleep bit
  else
      c = c & 0xBF;  // mask 1011111 keeps all the previous bits
  p_mpu9255->bus_write_byte(mpu_i2c_addr, PWR_MGMT_1, c);
}

void mpu9255_verbose(const bool b) { system_status.b_verbose = b;}


void mpu9255_ahrs(const bool b) { system_status.b_ahrs = b;}



 void mpu9255_collect_mag_data_to(float* m_bias, float* m_scale) {
    if (system_status.b_verbose)
    mpu9255_delay(4000);

    uint16_t sample_count = 0;
    if (sensor_config.mag_mode == 0x02)
        sample_count = 128;    
    else if (sensor_config.mag_mode == 0x06) 
        sample_count = 1500;   

    int32_t bias[3] = {0, 0, 0}, scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767};
    int16_t mag_min[3] = {32767, 32767, 32767};
    int16_t mag_temp[3] = {0, 0, 0};
    for (uint16_t ii = 0; ii < sample_count; ii++) {
      mpu9255_read_mag(mag_temp);  // Read the mag data
      for (int jj = 0; jj < 3; jj++) {
          if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
          if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
      }
      if (sensor_config.mag_mode == 0x02) mpu9255_delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
      if (sensor_config.mag_mode == 0x06) mpu9255_delay(12);   // at 100 Hz ODR, new mag data is available every 10 ms
    }


    bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
    bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
    bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

    float bias_resolution = mpu9255_get_mag_resolution(mag_output_bits_e::M16BITS);
    m_bias[0] = (float)bias[0] * bias_resolution * sensor_calibration.mag_bias_factory[0];  // save mag biases in G for main program
    m_bias[1] = (float)bias[1] * bias_resolution * sensor_calibration.mag_bias_factory[1];
    m_bias[2] = (float)bias[2] * bias_resolution * sensor_calibration.mag_bias_factory[2];

    scale[0] = (float)(mag_max[0] - mag_min[0]) * sensor_calibration.mag_bias_factory[0] / 2;  // get average x axis max chord length in counts
    scale[1] = (float)(mag_max[1] - mag_min[1]) * sensor_calibration.mag_bias_factory[1] / 2;  // get average y axis max chord length in counts
    scale[2] = (float)(mag_max[2] - mag_min[2]) * sensor_calibration.mag_bias_factory[2] / 2;  // get average z axis max chord length in counts

    float avg_rad = scale[0] + scale[1] + scale[2];
    avg_rad /= 3.0;

    m_scale[0] = avg_rad / ((float)scale[0]);
    m_scale[1] = avg_rad / ((float)scale[1]);
    m_scale[2] = avg_rad / ((float)scale[2]);
}

void mpu9255_calibrate_mag_impl() {
  mag_output_bits_e mag_output_bits_cache = setting.mag_output_bits;
  setting.mag_output_bits = mag_output_bits_e::M16BITS;
  mpu9255_init_AK8963();
  mpu9255_collect_mag_data_to(sensor_calibration.mag_bias, sensor_calibration.mag_scale);
  setting.mag_output_bits = mag_output_bits_cache;
  mpu9255_init_AK8963();
}

void mpu9255_calibrate_mag() {
  mpu9255_calibrate_mag_impl();
}


void mpu9255_delay(int ms) {
  ThisThread::sleep_for(chrono::milliseconds(ms));
}

void mpu9255_calibrate_accel_gyro() {
  mpu9255_calibrate_acc_gyro_impl();
}

void mpu9255_calibrate_acc_gyro_impl() {
  mpu9255_set_acc_gyro_to_calibration();
  mpu9255_collect_acc_gyro_data_to(sensor_calibration.acc_bias, sensor_calibration.gyro_bias);
  mpu9255_write_accel_offset();
  mpu9255_write_gyro_offset();
  mpu9255_delay(100);
  mpu9255_init_MPU9255();
  mpu9255_delay(1000);
}


void mpu9255_set_acc_gyro_to_calibration() {
  p_mpu9255->bus_write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
  mpu9255_delay(100);

  p_mpu9255->bus_write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x01);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, PWR_MGMT_2, 0x00);
  mpu9255_delay(200);

  // Configure device for bias calculation
  p_mpu9255->bus_write_byte(mpu_i2c_addr, INT_ENABLE, 0x00);    // Disable all interrupts
  p_mpu9255->bus_write_byte(mpu_i2c_addr, FIFO_EN, 0x00);       // Disable FIFO
  p_mpu9255->bus_write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x00);    // Turn on internal clock source
  p_mpu9255->bus_write_byte(mpu_i2c_addr, I2C_MST_CTRL, 0x00);  // Disable I2C master
  p_mpu9255->bus_write_byte(mpu_i2c_addr, USER_CTRL, 0x00);     // Disable FIFO and I2C master modes
  p_mpu9255->bus_write_byte(mpu_i2c_addr, USER_CTRL, 0x0C);     // Reset FIFO and DMP
  mpu9255_delay(15);

  p_mpu9255->bus_write_byte(mpu_i2c_addr, MPU_CONFIG, 0x01);    // Set low-pass filter to 188 Hz
  p_mpu9255->bus_write_byte(mpu_i2c_addr, SMPLRT_DIV, 0x00);    // Set sample rate to 1 kHz
  p_mpu9255->bus_write_byte(mpu_i2c_addr, GYRO_CONFIG, 0x00);   // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  p_mpu9255->bus_write_byte(mpu_i2c_addr, ACCEL_CONFIG, 0x00);  // Set accelerometer full-scale to 2 g, maximum sensitivity

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  p_mpu9255->bus_write_byte(mpu_i2c_addr, USER_CTRL, 0x40);  // Enable FIFO
  p_mpu9255->bus_write_byte(mpu_i2c_addr, FIFO_EN, 0x78);    // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  mpu9255_delay(40);                                  // accumulate 40 samples in 40 milliseconds = 480 bytes
}


void mpu9255_collect_acc_gyro_data_to(float* a_bias, float* g_bias) {
  // At end of sample accumulation, turn off FIFO sensor read
  uint8_t data[12];                                    // data array to hold accelerometer and gyro x, y, z, data
  p_mpu9255->bus_write_byte(mpu_i2c_addr, FIFO_EN, 0x00);             // Disable gyro and accelerometer sensors for FIFO
  p_mpu9255->bus_read_bytes(mpu_i2c_addr, FIFO_COUNTH, 2, &data[0]);  // read FIFO sample count
  uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
  uint16_t packet_count = fifo_count / 12;  // How many sets of full gyro and accelerometer data for averaging

  for (uint16_t ii = 0; ii < packet_count; ii++) {
      int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
      p_mpu9255->bus_read_bytes(mpu_i2c_addr, FIFO_R_W, 12, &data[0]);              // read data for averaging
      accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
      accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
      accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
      gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
      gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
      gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

      a_bias[0] += (float)accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
      a_bias[1] += (float)accel_temp[1];
      a_bias[2] += (float)accel_temp[2];
      g_bias[0] += (float)gyro_temp[0];
      g_bias[1] += (float)gyro_temp[1];
      g_bias[2] += (float)gyro_temp[2];
  }
  a_bias[0] /= (float)packet_count;  // Normalize sums to get average count biases
  a_bias[1] /= (float)packet_count;
  a_bias[2] /= (float)packet_count;
  g_bias[0] /= (float)packet_count;
  g_bias[1] /= (float)packet_count;
  g_bias[2] /= (float)packet_count;

  if (a_bias[2] > 0L) {
      a_bias[2] -= (float)CALIB_ACCEL_SENSITIVITY;
  }  // Remove gravity from the z-axis accelerometer bias calculation
  else {
      a_bias[2] += (float)CALIB_ACCEL_SENSITIVITY;
  }
}



bool mpu9255_is_connected() {
  system_status.has_connected = mpu9255_is_connected_MPU9255() && mpu9255_is_connected_AK8963();
  return system_status.has_connected;
}

bool mpu9255_is_connected_MPU9255() {
  // uint8_t c = mpu9255_read_byte(mpu_i2c_addr, WHO_AM_I_MPU9250);
   uint8_t c = p_mpu9255->bus_read_byte(mpu_i2c_addr, WHO_AM_I_MPU9250);
  if (system_status.b_verbose) {
      printf("MPU9250 WHO AM I = ");
  }
  bool b = (c == MPU9250_WHOAMI_DEFAULT_VALUE);
  b |= (c == MPU9255_WHOAMI_DEFAULT_VALUE);
  b |= (c == MPU6500_WHOAMI_DEFAULT_VALUE);
  return b;
}



bool mpu9255_is_connected_AK8963() {
  uint8_t c = p_mpu9255->bus_read_byte(AK8963_ADDRESS, AK8963_WHO_AM_I);
  if (system_status.b_verbose) {
      printf("AK8963 WHO AM I = ");
  }
  return (c == AK8963_WHOAMI_DEFAULT_VALUE);
}

bool mpu9255_is_sleeping() {
  uint8_t c = p_mpu9255->bus_read_byte(mpu_i2c_addr, PWR_MGMT_1);
  return (c & 0x40) == 0x40;
}

bool mpu9255_available() {
  return system_status.has_connected && (p_mpu9255->bus_read_byte(mpu_i2c_addr, INT_STATUS) & 0x01);
}

bool mpu9255_update(u64 delta_time) {
  if (!mpu9255_available()) return false;

  mpu9255_update_accel_gyro();
  mpu9255_update_mag();

  float an = -sensor_data.acc[0];
  float ae = +sensor_data.acc[1];
  float ad = +sensor_data.acc[2];
  float gn = +sensor_data.gyro[0] * DEG_TO_RAD;
  float ge = -sensor_data.gyro[1] * DEG_TO_RAD;
  float gd = -sensor_data.gyro[2] * DEG_TO_RAD;
  float mn = +sensor_data.mag[1];
  float me = -sensor_data.mag[0];
  float md = +sensor_data.mag[2];

  for (size_t i = 0; i < sensor_data.n_filter_iter; ++i)
      sensor_data.quat_filter.update(an, ae, ad, gn, ge, gd, mn, me, md, sensor_data.quaternion, delta_time);

  if (!system_status.b_ahrs) {
      sensor_temperature.raw_count = mpu9255_read_temperature_data();               // Read the adc values
      sensor_temperature.value = ((float)sensor_temperature.raw_count) / 333.87 + 21.0;  // Temperature in degrees Centigrade
  } else
      mpu9255_update_rpy(sensor_data.quaternion[0], sensor_data.quaternion[1], sensor_data.quaternion[2], sensor_data.quaternion[3]);
  return true;
}

int16_t mpu9255_read_temperature_data() {
  uint8_t raw_data[2];                                    // x/y/z gyro register data stored here
  p_mpu9255->bus_read_bytes(mpu_i2c_addr, TEMP_OUT_H, 2, &raw_data[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)raw_data[0] << 8) | raw_data[1];       // Turn the MSB and LSB into a 16-bit value
}


float mpu9255_get_roll()  { return sensor_data.rpy[0]; }
float mpu9255_get_pitch()  { return sensor_data.rpy[1]; }
float mpu9255_get_yaw()  { return sensor_data.rpy[2]; }


void mpu9255_set_acc_bias(const float x, const float y, const float z) {
  sensor_calibration.acc_bias[0] = x;
  sensor_calibration.acc_bias[1] = y;
  sensor_calibration.acc_bias[2] = z;
  mpu9255_write_accel_offset();
}

void mpu9255_set_gyro_bias(const float x, const float y, const float z) {
  sensor_calibration.gyro_bias[0] = x;
  sensor_calibration.gyro_bias[1] = y;
  sensor_calibration.gyro_bias[2] = z;
  mpu9255_write_gyro_offset();
}


void mpu9255_select_filter(QuatFilterSel sel) {
  sensor_data.quat_filter.select_filter(sel);
}

void mpu9255_set_filter_iterations(const size_t n) {
  if (n > 0) sensor_data.n_filter_iter = n;
}


void mpu9255_init_MPU9255() {
  sensor_resolution.acc   = mpu9255_get_acc_resolution(setting.accel_fs_sel);
  sensor_resolution.gyro  = mpu9255_get_gyro_resolution(setting.gyro_fs_sel);
  sensor_resolution.mag   = mpu9255_get_mag_resolution(setting.mag_output_bits);

  p_mpu9255->bus_write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
  mpu9255_delay(100);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x00);  // Clear sleep mode bit (6), enable all sensors
  mpu9255_delay(100);                                  // Wait for all registers to reset
  p_mpu9255->bus_write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  mpu9255_delay(200);
  uint8_t mpu_config = (uint8_t)setting.gyro_dlpf_cfg;

  p_mpu9255->bus_write_byte(mpu_i2c_addr, MPU_CONFIG, mpu_config);

  uint8_t sample_rate = (uint8_t)setting.fifo_sample_rate;
  p_mpu9255->bus_write_byte(mpu_i2c_addr, SMPLRT_DIV, sample_rate);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  uint8_t c = p_mpu9255->bus_read_byte(mpu_i2c_addr, GYRO_CONFIG);  // get current GYRO_CONFIG register value
  c = c & ~0xE0;                                     // Clear self-test bits [7:5]
  c = c & ~0x03;                                     // Clear Fchoice bits [1:0]
  c = c & ~0x18;                                     // Clear GYRO_FS_SEL bits [4:3]
  c = c | (uint8_t(setting.gyro_fs_sel) << 3);       // Set full scale range for the gyro
  c = c | (uint8_t(~setting.gyro_fchoice) & 0x03);   // Set Fchoice for the gyro
  p_mpu9255->bus_write_byte(mpu_i2c_addr, GYRO_CONFIG, c);          // Write new GYRO_CONFIG value to register

  c = p_mpu9255->bus_read_byte(mpu_i2c_addr, ACCEL_CONFIG);     // get current ACCEL_CONFIG register value
  c = c & ~0xE0;                                 // Clear self-test bits [7:5]
  c = c & ~0x18;                                 // Clear ACCEL_FS_SEL bits [4:3]
  c = c | (uint8_t(setting.accel_fs_sel) << 3);  // Set full scale range for the accelerometer
  p_mpu9255->bus_write_byte(mpu_i2c_addr, ACCEL_CONFIG, c);     // Write new ACCEL_CONFIG register value
  c = p_mpu9255->bus_read_byte(mpu_i2c_addr, ACCEL_CONFIG2);        // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F;                                     // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | (~(setting.accel_fchoice << 3) & 0x08);    // Set accel_fchoice_b to 1
  c = c | (uint8_t(setting.accel_dlpf_cfg) & 0x07);  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  p_mpu9255->bus_write_byte(mpu_i2c_addr, ACCEL_CONFIG2, c);        // Write new ACCEL_CONFIG2 register value
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
  p_mpu9255->bus_write_byte(mpu_i2c_addr, INT_PIN_CFG, 0x22);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  mpu9255_delay(100);
}



void mpu9255_init_AK8963() {
  // First extract the factory calibration for each magnetometer axis
  uint8_t raw_data[3];                            // x/y/z gyro calibration data stored here
  p_mpu9255->bus_write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00);  // Power down magnetometer
  mpu9255_delay(10);
  p_mpu9255->bus_write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x0F);  // Enter Fuse ROM access mode
  mpu9255_delay(10);
  p_mpu9255->bus_read_bytes(AK8963_ADDRESS, AK8963_ASAX, 3, &raw_data[0]);      // Read the x-, y-, and z-axis calibration values
  sensor_calibration.mag_bias_factory[0] = (float)(raw_data[0] - 128) / 256. + 1.;  // Return x-axis sensitivity adjustment values, etc.
  sensor_calibration.mag_bias_factory[1] = (float)(raw_data[1] - 128) / 256. + 1.;
  sensor_calibration.mag_bias_factory[2] = (float)(raw_data[2] - 128) / 256. + 1.;
  p_mpu9255->bus_write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00);  // Power down magnetometer
  mpu9255_delay(10);
  p_mpu9255->bus_write_byte(AK8963_ADDRESS, AK8963_CNTL, (uint8_t)setting.mag_output_bits << 4 | sensor_config.mag_mode);  // Set magnetometer data resolution and sample ODR
  mpu9255_delay(10);


}



void mpu9255_update_rpy(float qw, float qx, float qy, float qz) {

  float a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
  a12 = 2.0f * (qx * qy + qw * qz);
  a22 = qw * qw + qx * qx - qy * qy - qz * qz;
  a31 = 2.0f * (qw * qx + qy * qz);
  a32 = 2.0f * (qx * qz - qw * qy);
  a33 = qw * qw - qx * qx - qy * qy + qz * qz;
  sensor_data.rpy[0] = atan2f(a31, a33);
  sensor_data.rpy[1] = -asinf(a32);
  sensor_data.rpy[2] = atan2f(a12, a22);
  sensor_data.rpy[0] *= 180.0f / PI;
  sensor_data.rpy[1] *= 180.0f / PI;
  sensor_data.rpy[2] *= 180.0f / PI;
  sensor_data.rpy[2] += sensor_calibration.magnetic_declination;
  if (sensor_data.rpy[2] >= +180.f)
      sensor_data.rpy[2] -= 360.f;
  else if (sensor_data.rpy[2] < -180.f)
      sensor_data.rpy[2] += 360.f;

  sensor_data.lin_acc[0] = sensor_data.acc[0] + a31;
  sensor_data.lin_acc[1] = sensor_data.acc[1] + a32;
  sensor_data.lin_acc[2] = sensor_data.acc[2] - a33;
}


void mpu9255_update_accel_gyro() {
  int16_t raw_acc_gyro_data[7];        // used to read all 14 bytes at once from the MPU9250 accel/gyro
  mpu9255_read_accel_gyro(raw_acc_gyro_data);  // INT cleared on any read

  // Now we'll calculate the accleration value into actual g's
  sensor_data.acc[0] = (float)raw_acc_gyro_data[0] * sensor_resolution.acc;  // get actual g value, this depends on scale being set
  sensor_data.acc[1] = (float)raw_acc_gyro_data[1] * sensor_resolution.acc;
  sensor_data.acc[2] = (float)raw_acc_gyro_data[2] * sensor_resolution.acc;

  sensor_temperature.raw_count = raw_acc_gyro_data[3];                  // Read the adc values
  sensor_temperature.value = ((float)sensor_temperature.raw_count) / 333.87 + 21.0;  // Temperature in degrees Centigrade

  // Calculate the gyro value into actual degrees per second
  sensor_data.gyro[0] = (float)raw_acc_gyro_data[4] * sensor_resolution.gyro;  // get actual gyro value, this depends on scale being set
  sensor_data.gyro[1] = (float)raw_acc_gyro_data[5] * sensor_resolution.gyro;
  sensor_data.gyro[2] = (float)raw_acc_gyro_data[6] * sensor_resolution.gyro;

}




void mpu9255_read_accel_gyro(int16_t* destination) {
  uint8_t raw_data[14];                                                 // x/y/z accel register data stored here
  p_mpu9255->bus_read_bytes(mpu_i2c_addr, ACCEL_XOUT_H, 14, &raw_data[0]);             // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)raw_data[0] << 8) | (int16_t)raw_data[1];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)raw_data[2] << 8) | (int16_t)raw_data[3];
  destination[2] = ((int16_t)raw_data[4] << 8) | (int16_t)raw_data[5];
  destination[3] = ((int16_t)raw_data[6] << 8) | (int16_t)raw_data[7];
  destination[4] = ((int16_t)raw_data[8] << 8) | (int16_t)raw_data[9];
  destination[5] = ((int16_t)raw_data[10] << 8) | (int16_t)raw_data[11];
  destination[6] = ((int16_t)raw_data[12] << 8) | (int16_t)raw_data[13];
}



void mpu9255_update_mag() {
  int16_t mag_count[3] = {0, 0, 0};  // Stores the 16-bit signed magnetometer sensor output

  if (mpu9255_read_mag(mag_count)) {
      float bias_to_current_bits = sensor_resolution.mag / mpu9255_get_mag_resolution(mag_output_bits_e::M16BITS);
      sensor_data.mag[0] = (float)(mag_count[0] * sensor_resolution.mag * sensor_calibration.mag_bias_factory[0] - sensor_calibration.mag_bias[0] * bias_to_current_bits) * sensor_calibration.mag_scale[0];  // get actual magnetometer value, this depends on scale being set
      sensor_data.mag[1] = (float)(mag_count[1] * sensor_resolution.mag * sensor_calibration.mag_bias_factory[1] - sensor_calibration.mag_bias[1] * bias_to_current_bits) * sensor_calibration.mag_scale[1];
      sensor_data.mag[2] = (float)(mag_count[2] * sensor_resolution.mag * sensor_calibration.mag_bias_factory[2] - sensor_calibration.mag_bias[2] * bias_to_current_bits) * sensor_calibration.mag_scale[2];
  }
}



bool read_mag(int16_t* destination) {
  const uint8_t st1 = p_mpu9255->bus_read_byte(AK8963_ADDRESS, AK8963_ST1);
  if (st1 & 0x01) {                                                    // wait for magnetometer data ready bit to be set
      uint8_t raw_data[7];                                             // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
      p_mpu9255->bus_read_bytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &raw_data[0]);      // Read the six raw data and ST2 registers sequentially into data array
      if (sensor_config.mag_mode == 0x02 || sensor_config.mag_mode == 0x04 || sensor_config.mag_mode == 0x06) {  // continuous or external trigger read mode
          if ((st1 & 0x02) != 0)                                       // check if data is not skipped
              return false;                                            // this should be after data reading to clear DRDY register
      }

      uint8_t c = raw_data[6];                                         // End data read by reading ST2 register
      if (!(c & 0x08)) {                                               // Check if magnetic sensor overflow set, if not then report data
          destination[0] = ((int16_t)raw_data[1] << 8) | raw_data[0];  // Turn the MSB and LSB into a signed 16-bit value
          destination[1] = ((int16_t)raw_data[3] << 8) | raw_data[2];  // Data stored as little Endian
          destination[2] = ((int16_t)raw_data[5] << 8) | raw_data[4];
          return true;
      }
  }
  return false;
}


float mpu9255_get_acc_resolution(const accel_fs_sel_e accel_af_sel)  {
  switch (accel_af_sel) {
      // Possible accelerometer scales (and their register bit settings) are:
      // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
      // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
      case accel_fs_sel_e::A2G:
          return 2.0 / 32768.0;
      case accel_fs_sel_e::A4G:
          return 4.0 / 32768.0;
      case accel_fs_sel_e::A8G:
          return 8.0 / 32768.0;
      case accel_fs_sel_e::A16G:
          return 16.0 / 32768.0;
      default:
          return 0.;
  }
}

float mpu9255_get_gyro_resolution(const gyro_fs_sel_e gyro_fs_sel)  {
  switch (gyro_fs_sel) {
      case gyro_fs_sel_e::G250DPS:
          return 250.0 / 32768.0;
      case gyro_fs_sel_e::G500DPS:
          return 500.0 / 32768.0;
      case gyro_fs_sel_e::G1000DPS:
          return 1000.0 / 32768.0;
      case gyro_fs_sel_e::G2000DPS:
          return 2000.0 / 32768.0;
      default:
          return 0.;
  }
}

void mpu9255_write_accel_offset() {

  uint8_t read_data[2] = {0};
  int16_t acc_bias_reg[3] = {0, 0, 0};                      // A place to hold the factory accelerometer trim biases
  p_mpu9255->bus_read_bytes(mpu_i2c_addr, XA_OFFSET_H, 2, &read_data[0]);  // Read factory accelerometer trim values
  acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];
  p_mpu9255->bus_read_bytes(mpu_i2c_addr, YA_OFFSET_H, 2, &read_data[0]);
  acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];
  p_mpu9255->bus_read_bytes(mpu_i2c_addr, ZA_OFFSET_H, 2, &read_data[0]);
  acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];

  int16_t mask_bit[3] = {1, 1, 1};  // Define array to hold mask bit for each accelerometer bias axis
  for (int i = 0; i < 3; i++) {
      if (acc_bias_reg[i] % 2) {
          mask_bit[i] = 0;
      }
      acc_bias_reg[i] -= (int16_t)sensor_calibration.acc_bias[i] >> 3;  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
      if (mask_bit[i]) {
          acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i];  // Preserve temperature compensation bit
      } else {
          acc_bias_reg[i] = acc_bias_reg[i] | 0x0001;  // Preserve temperature compensation bit
      }
  }

  uint8_t write_data[6] = {0};
  write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
  write_data[1] = (acc_bias_reg[0]) & 0xFF;
  write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
  write_data[3] = (acc_bias_reg[1]) & 0xFF;
  write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
  write_data[5] = (acc_bias_reg[2]) & 0xFF;

  p_mpu9255->bus_write_byte(mpu_i2c_addr, XA_OFFSET_H, write_data[0]);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, XA_OFFSET_L, write_data[1]);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, YA_OFFSET_H, write_data[2]);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, YA_OFFSET_L, write_data[3]);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, ZA_OFFSET_H, write_data[4]);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, ZA_OFFSET_L, write_data[5]);
}



void mpu9255_write_gyro_offset() {
  uint8_t gyro_offset_data[6] {0};
  gyro_offset_data[0] = (-(int16_t)sensor_calibration.gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  gyro_offset_data[1] = (-(int16_t)sensor_calibration.gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
  gyro_offset_data[2] = (-(int16_t)sensor_calibration.gyro_bias[1] / 4 >> 8) & 0xFF;
  gyro_offset_data[3] = (-(int16_t)sensor_calibration.gyro_bias[1] / 4) & 0xFF;
  gyro_offset_data[4] = (-(int16_t)sensor_calibration.gyro_bias[2] / 4 >> 8) & 0xFF;
  gyro_offset_data[5] = (-(int16_t)sensor_calibration.gyro_bias[2] / 4) & 0xFF;

  p_mpu9255->bus_write_byte(mpu_i2c_addr, XG_OFFSET_H, gyro_offset_data[0]);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, XG_OFFSET_L, gyro_offset_data[1]);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, YG_OFFSET_H, gyro_offset_data[2]);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, YG_OFFSET_L, gyro_offset_data[3]);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, ZG_OFFSET_H, gyro_offset_data[4]);
  p_mpu9255->bus_write_byte(mpu_i2c_addr, ZG_OFFSET_L, gyro_offset_data[5]);
}


float mpu9255_get_mag_resolution(const mag_output_bits_e mag_output_bits)  {
  switch (mag_output_bits) {
      case mag_output_bits_e::M14BITS:
          return 10. * 4912. / 8190.0;
      case mag_output_bits_e::M16BITS:
          return 10. * 4912. / 32760.0;
      default:
          return 0.;
  }
}
// void p_mpu9255->bus_write_byte(uint8_t address, uint8_t subAddress, uint8_t data) {
//   char cmd[2];
//   cmd[0] = subAddress;  
//   cmd[1] = data;        

//   int result = i2c.write(address << 1, cmd, 2);  

//   if (result != 0) {
//       printf("Writing error!\n");
//   }
// }
// uint8_t mpu9255_read_byte(uint8_t address, uint8_t subAddress) {
//   uint8_t data = 0;  
//   char cmd[1];
//   cmd[0] = subAddress; 

//   int result = i2c.write(address << 1, cmd, 1); 
//   if (result != 0) {
//       // printf("Writing error!\n");
//       return 0;          
//   }

//   result = i2c.read(address << 1, cmd, 1); 
//   if (result == 0) {
//       data = cmd[0];  
//   } else {
//       // printf("Reading error!\n");
//   }
//   return data; 
// }


// void p_mpu9255->bus_read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
//   char cmd[1] = {subAddress};
//   int i2c_err_ = i2c.write(address << 1, cmd, 1, true);  
//   if (i2c_err_ != 0) {
//       // printf("Writing error!\n"); 
//       return;
//   }

//   i2c_err_ = i2c.read(address << 1, (char*)dest, count);  
//   if (i2c_err_ != 0) {
//   //    printf("Reading error!\n");
//   }
// }