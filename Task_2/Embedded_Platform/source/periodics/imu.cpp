/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/


#include <periodics/imu.hpp>
#include "imu.hpp"

#define _100_chars                      100
#define BNO055_EULER_DIV_DEG_int        16
#define BNO055_LINEAR_ACCEL_DIV_MSQ_int 100
#define precision_scaling_factor        1000

namespace periodics{
    /** \brief  Class constructor
     *
     *  It initializes the task and the state of the led. 
     *
     *  \param f_period       Toggling period of LED
     *  \param f_led          Digital output line to LED
     */

    /*--------------------------------------------------------------------------------------------------*
    *  Before initializiting with another value, the i2c_instance static pointer variable should be
    *  initialized with a nullptr.
    *---------------------------------------------------------------------------------------------------*/
    I2C* periodics::CImu::i2c_instance = nullptr;

    CImu::CImu(
            drivers::ISpeedingMonitor&   f_speedingMonitor,
            std::chrono::milliseconds    f_period, 
            UnbufferedSerial& f_serial,
            PinName SDA,
            PinName SCL)
        : utils::CTask(f_period)
        , m_isActive(false)
        , m_serial(f_serial)
        , m_delta_time(f_period.count())
        , m_speedingMonitor(f_speedingMonitor)
    {
        if(m_delta_time < 150){
            setNewPeriod(150);
            m_delta_time = 150;
        }
        
        s32 comres = BNO055_ERROR;
        /* variable used to set the power mode of the sensor*/
        u8 power_mode = BNO055_INIT_VALUE;

        /*---------------------------------------------------------------------------*
        *********************** START INITIALIZATION ************************
        *--------------------------------------------------------------------------*/

        /*--------------------------------------------------------------------------------------------------*
        *  i2c_instance variable member will be initialized with the actual I2C of the target board.
        *---------------------------------------------------------------------------------------------------*/      
        i2c_instance = new I2C(SDA, SCL);
        i2c_instance->frequency(400000);

        ThisThread::sleep_for(chrono::milliseconds(300));

        I2C_routine();
        printf("init bno \n");
        pca9548_i2c_write(0x70 << 1, 2);
        comres = bno055_init(&bno055);
        power_mode = BNO055_POWER_MODE_NORMAL;
        comres = bno055_set_powermode(power_mode);
        comres = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
        for(int i = 0; i < 10; i++) {
            s16_euler_h_raw = bno055_read_euler_h();
            s16_euler_h_init = ((s16_euler_h_raw) * 1000) / 16.00;
            s16_euler_h_init /= 1000;
            BNO055_delay_msek(300);
        }
        
    }

    /** @brief  CImu class destructor
     */
    CImu::~CImu()
    {
        /*-----------------------------------------------------------------------*
        ************************* START DE-INITIALIZATION ***********************
        *-------------------------------------------------------------------------*/
        s32 comres = BNO055_ERROR;
        /* variable used to set the power mode of the sensor*/
        u8 power_mode = BNO055_INIT_VALUE;
        /*  For de - initializing the BNO sensor it is required
        * to the operation mode of the sensor as SUSPEND
        * Suspend mode can set from the register
        * Page - page0
        * register - 0x3E
        * bit positions - 0 and 1*/
        power_mode = BNO055_POWER_MODE_SUSPEND;

        /* set the power mode as SUSPEND*/
        comres += bno055_set_powermode(power_mode);

        if(i2c_instance != nullptr)
        {
            delete i2c_instance;
            i2c_instance = nullptr;
        }
        /*---------------------------------------------------------------------*
        ************************* END DE-INITIALIZATION **********************
        *---------------------------------------------------------------------*/
    };

    /** \brief  Serial callback method to activate or deactivate the publisher. 
     * When the received integer value is bigger or equal to 1, then the publisher become 
     * active and send messages, otherwise is deactivated. 
     *
     * @param a                   input received string
     * @param b                   output reponse message
     * 
     */
    void CImu::serialCallbackIMUcommand(char const * a, char * b) {
        uint8_t l_isActivate=0;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);

        if(1 == l_res){
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive=(l_isActivate>=1);
                bool_globalsV_imu_isActive = (l_isActivate>=1);
                sprintf(b,"1");
            }
            else{
                sprintf(b,"kl 15/30 is required!!");
            }
            
        }else{
            sprintf(b,"syntax error");
        }
    }

    int8_t CImu::pca9548_i2c_write(uint8_t addr8bit_pca, uint8_t channel){
        // s32 VL53L1X_iERROR = VL53L1X_INIT_VALUE;
        uint8_t buffer = 1 << channel;
        if (i2c_instance->write(addr8bit_pca, (const char*)&buffer, 1, false)) {
            // VL53L1X_iERROR = VL53L1X_ERROR; 
            return -1;
        }
        return 0;

    }



    /**
    * \brief Writes data to the device over the I2C bus.
    * 
    * This function serves as a low-level I2C write routine tailored for the BNO055 sensor. 
    * It packages the register address and data to be written into a single buffer and then 
    * dispatches the write operation.
    * 
    * \param dev_addr   : I2C address of the BNO055 sensor.
    * \param reg_addr   : Address of the target register where the data needs to be written.
    * \param reg_data   : Pointer to an array containing the data bytes to be written to the sensor.
    * \param cnt        : Number of data bytes to write from the \a reg_data array.
    * 
    * \return BNO055_SUCCESS (0) on successful write operation.
    * \return BNO055_ERROR (-1) if the write operation encounters an error.
    * 
    * \note The actual data writing starts from the second position in the array, as the 
    *       first position is reserved for the register address.
    */
 

    int CImu::BNO055_I2C_bus_write(unsigned char dev_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt) {
    BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
    char *data = new char[cnt + 1]; // Tạo buffer để lưu địa chỉ thanh ghi và dữ liệu
    data[0] = reg_addr; // Byte đầu tiên là địa chỉ thanh ghi

    // Sao chép dữ liệu vào buffer
    for (unsigned char index = 0; index < cnt; index++) {
        data[index + 1] = reg_data[index];
    }

    // Gửi dữ liệu qua I2C
    if(i2c_instance->write(dev_addr, data, cnt + 1))
        printf("Error Writing!");

    delete[] data; // Giải phóng bộ nhớ
    wait_us(150); // Đợi 150 micro giây

    return comres;
}


    /**
    * \brief Reads data from the device over the I2C bus.
    * 
    * This function facilitates reading data from a specific register of the BNO055 sensor 
    * over the I2C communication protocol. It sends the desired register address to the sensor, 
    * then reads back the requested amount of data bytes into a provided buffer.
    * 
    * \param dev_addr   : I2C address of the BNO055 sensor.
    * \param reg_addr   : Address of the target register from which the data needs to be read.
    * \param reg_data   : Pointer to an array where the read data bytes will be stored.
    * \param cnt        : Number of data bytes to read into the \a reg_data array.
    * 
    * \return BNO055_SUCCESS (0) on successful read operation.
    * \return BNO055_ERROR (-1) if the read operation encounters an error.
    * 
    * \note The function first writes the register address to the sensor to set the pointer 
    *       to the desired location and then initiates the I2C read operation.
    */

int CImu::BNO055_I2C_bus_read(unsigned char dev_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt) {
    BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
    char cmd[1];
    cmd[0] = reg_addr;

    // Gửi địa chỉ thanh ghi cần đọc
    if(i2c_instance->write(dev_addr, cmd, 1, true) !=  0){
        printf("Error Writing!");
    } // true để gửi lại start condition

    // Đọc dữ liệu từ thanh ghi
    if(i2c_instance->read(dev_addr, (char *)reg_data, cnt))
     printf("Error read!");

    return comres;
}


    
    /*-------------------------------------------------------------------------*
     *  By using bno055 the following structure parameter can be accessed
     *  Bus write function pointer: BNO055_WR_FUNC_PTR
     *  Bus read function pointer: BNO055_RD_FUNC_PTR
     *  Delay function pointer: delay_msec
     *  I2C address: dev_addr
     *--------------------------------------------------------------------------*/
    void CImu::I2C_routine(void)
    {
        bno055.bus_write = BNO055_I2C_bus_write;
        bno055.bus_read = BNO055_I2C_bus_read;
        bno055.delay_msec = BNO055_delay_msek;
        bno055.dev_addr = 0x29 << 1;
        // bno055.dev_addr = BNO055_I2C_ADDR1 << 1;

        ThisThread::sleep_for(chrono::milliseconds(300));
    }

    /**
    * \brief Introduces a delay for the specified duration in milliseconds.
    * 
    * This function relies on the `ThisThread::sleep_for` method to create 
    * a delay, making the current thread sleep for the specified duration.
    * 
    * \param msek The delay duration in milliseconds.
    */
    void CImu::BNO055_delay_msek(u32 msek)
    {
        /*Here you can write your own delay routine*/
        ThisThread::sleep_for(chrono::milliseconds(msek));
    }

    /** 
    * \brief  Periodically retrieves and processes IMU sensor values.
    * 
    * This method is invoked periodically and handles:
    * 1. Reading Euler angles (roll, pitch, yaw) from the BNO055 sensor.
    * 2. Reading linear acceleration values in the x, y, and z axes from the sensor.
    * 3. Based on the linear acceleration, it updates the current velocity of the device.
    * 4. If the device appears to be stationary (based on x and y acceleration thresholds), 
    *    a counter is incremented. If the device remains stationary for a certain duration 
    *    (15 cycles in this case), the velocity is reset.
    * 5. Formats and sends the acquired data over a serial connection.
    * 
    * \note If there are any issues reading from the BNO055 sensor, the method will exit early without sending data.
    */

    s32 CImu::convert_imu_angle(s32 imu_angle, s32 offset) {
        return ((offset) - imu_angle + 360) % 360;
    }

    void CImu::reset_imu_value(void) {
        s32 tmp = bno055_read_euler_h();
        s16_euler_h_init = ((((tmp)) / 16.00));

    }

    void CImu::set_imu_value(s16 value) {
        s32 tmp = abs(bno055_read_euler_h()) + value;
        s16_euler_h_init = ((((tmp)) / 16.00));

    }

    s32 CImu::get_imu_value(s16 new_value) {
        s8 comres = BNO055_SUCCESS;
        s32 s16_euler_h = BNO055_INIT_VALUE;
        pca9548_i2c_write(0x70 << 1, 2);
        s16_euler_h_raw = bno055_read_euler_h();
        
        s16_euler_h = ((((s16_euler_h_raw)) / 16.00));
        data = convert_imu_angle(s16_euler_h + (-new_value),s16_euler_h_init);
        return data;
        // return convert_imu_angle(s16_euler_h / 1000,s16_euler_h_init/1000);
    }

    s32 CImu::get_imu_value_global() {
        s8 comres = BNO055_SUCCESS;
        s32 s16_euler_h_global_raw = BNO055_INIT_VALUE;
        s16_euler_h_global_raw = bno055_read_euler_h();
        s16_euler_h_global = ((((s16_euler_h_global_raw) * 1000) / 16.00));

        
        return s16_euler_h_global;
    }

    void CImu::_run()
    {
        if(!m_isActive) return;
        

    }

}; // namespace periodics