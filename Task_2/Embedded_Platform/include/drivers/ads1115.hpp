#ifndef ADS1115_HPP
#define ADS1115_HPP

// TODO: Add your code here
/*signed integer types*/

typedef signed char                 s8;             /**< used for signed 8bit */
typedef signed short int            s16;            /**< used for signed 16bit */
typedef signed int                  s32;            /**< used for signed 32bit */
typedef signed long long int        s64;            /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char               u8;             /**< used for unsigned 8bit */
typedef unsigned short int          u16;            /**< used for unsigned 16bit */
typedef unsigned int                u32;            /**< used for unsigned 32bit */
typedef unsigned long long int      u64;            /**< used for unsigned 64bit */

#define ADS1115_WR_FUNC_PTR       	    s8 \
    (*bus_write)(u8, u8, u16)

#define ADS1115_RD_FUNC_PTR       	    s8 \
    (*bus_read)(u8, u8, u8*)

#define ADS1115_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data) \
    bus_write(dev_addr, reg_addr, reg_data) 

#define ADS1115_BUS_READ_FUNC(dev_addr, reg_addr, reg_data) \
    bus_read(dev_addr, reg_addr, reg_data) 

#define ADS1115_MDELAY_DATA_TYPE            u32
#define ADS1115_ERROR                       ((s8) - 1)
#define ADS1115_SUCCESS                     ((u8)0)
#define ADS1115_INIT_VALUE                  ((u8)0)
#define ADS1115_RETURN_FUNCTION_TYPE        s8
#define ADS1115_DELAY_RETURN_TYPE           void
#define ADS1115_DELAY_FUNC (delay_in_msec) \
                            delay_func(delay_in_msec)


#define ADS1115_ADDRESS                     0x48
#define ADS1115_REG_POINTER_MASK            0x03      
#define ADS1115_REG_POINTER_CONVERT         0x00  
#define ADS1115_REG_POINTER_CONFIG          0x01      
#define ADS1115_REG_POINTER_LOWTHRESH       0x02  
#define ADS1115_REG_POINTER_HITHRESH        0x03      

#define ADS1115_REG_CONFIG_OS_MASK          0x8000    
#define ADS1115_REG_CONFIG_OS_SINGLE        0x8000    
#define ADS1115_REG_CONFIG_OS_BUSY          0x0000    
#define ADS1115_REG_CONFIG_OS_NOTBUSY       0x8000    

#define ADS1115_REG_CONFIG_MUX_MASK         0x7000    
#define ADS1115_REG_CONFIG_MUX_DIFF_0_1     0x0000
#define ADS1115_REG_CONFIG_MUX_DIFF_0_3     0x1000 
#define ADS1115_REG_CONFIG_MUX_DIFF_1_3     0x2000 
#define ADS1115_REG_CONFIG_MUX_DIFF_2_3     0x3000
#define ADS1115_REG_CONFIG_MUX_SINGLE_0     0x4000 
#define ADS1115_REG_CONFIG_MUX_SINGLE_1     0x5000 
#define ADS1115_REG_CONFIG_MUX_SINGLE_2     0x6000 
#define ADS1115_REG_CONFIG_MUX_SINGLE_3     0x7000 

constexpr u16 MUX_BY_CHANNEL[] = {
    ADS1115_REG_CONFIG_MUX_SINGLE_0, ///< Single-ended AIN0
    ADS1115_REG_CONFIG_MUX_SINGLE_1, ///< Single-ended AIN1
    ADS1115_REG_CONFIG_MUX_SINGLE_2, ///< Single-ended AIN2
    ADS1115_REG_CONFIG_MUX_SINGLE_3  ///< Single-ended AIN3
};   

#define ADS1115_REG_CONFIG_PGA_MASK         0x0E00   ///< PGA Mask
#define ADS1115_REG_CONFIG_PGA_6_144V       0x0000 ///< +/-6.144V range = Gain 2/3
#define ADS1115_REG_CONFIG_PGA_4_096V       0x0200 ///< +/-4.096V range = Gain 1
#define ADS1115_REG_CONFIG_PGA_2_048V       0x0400 ///< +/-2.048V range = Gain 2 (default)
#define ADS1115_REG_CONFIG_PGA_1_024V       0x0600 ///< +/-1.024V range = Gain 4
#define ADS1115_REG_CONFIG_PGA_0_512V       0x0800 ///< +/-0.512V range = Gain 8
#define ADS1115_REG_CONFIG_PGA_0_256V       0x0A00 ///< +/-0.256V range = Gain 16

#define ADS1115_REG_CONFIG_MODE_MASK        0x0100   ///< Mode Mask
#define ADS1115_REG_CONFIG_MODE_CONTIN      0x0000 ///< Continuous conversion mode
#define ADS1115_REG_CONFIG_MODE_SINGLE      0x0100 ///< Power-down single-shot mode (default)

#define ADS1115_REG_CONFIG_RATE_MASK        0x00E0 ///< Data Rate Mask

#define ADS1115_REG_CONFIG_CMODE_MASK       0x0010 ///< CMode Mask
#define ADS1115_REG_CONFIG_CMODE_TRAD       0x0000 ///< Traditional comparator with hysteresis (default)
#define ADS1115_REG_CONFIG_CMODE_WINDOW     0x0010 ///< Window comparator

#define ADS1115_REG_CONFIG_CPOL_MASK        0x0008 ///< CPol Mask
#define ADS1115_REG_CONFIG_CPOL_ACTVLOW     0x0000 ///< ALERT/RDY pin is low when active (default)
#define ADS1115_REG_CONFIG_CPOL_ACTVHI      0x0008 ///< ALERT/RDY pin is high when active

#define ADS1115_REG_CONFIG_CLAT_MASK        0x0004 ///< Determines if ALERT/RDY pin latches once asserted
#define ADS1115_REG_CONFIG_CLAT_NONLAT      0x0000 ///< Non-latching comparator (default)
#define ADS1115_REG_CONFIG_CLAT_LATCH       0x0004 ///< Latching comparator

#define ADS1115_REG_CONFIG_CQUE_MASK        0x0003 ///< CQue Mask
#define ADS1115_REG_CONFIG_CQUE_1CONV       0x0000 ///< Assert ALERT/RDY after one conversions
#define ADS1115_REG_CONFIG_CQUE_2CONV       0x0001 ///< Assert ALERT/RDY after two conversions
#define ADS1115_REG_CONFIG_CQUE_4CONV       0x0002 ///< Assert ALERT/RDY after four conversions
#define ADS1115_REG_CONFIG_CQUE_NONE        0x0003 ///< Disable the comparator and put ALERT/RDY in high state (default)

/** Gain settings */
typedef enum {
    GAIN_TWOTHIRDS = ADS1115_REG_CONFIG_PGA_6_144V,
    GAIN_ONE = ADS1115_REG_CONFIG_PGA_4_096V,
    GAIN_TWO = ADS1115_REG_CONFIG_PGA_2_048V,
    GAIN_FOUR = ADS1115_REG_CONFIG_PGA_1_024V,
    GAIN_EIGHT = ADS1115_REG_CONFIG_PGA_0_512V,
    GAIN_SIXTEEN = ADS1115_REG_CONFIG_PGA_0_256V
  } adsGain_t;

  /** Data rates */
#define RATE_ADS1015_128SPS             0x0000  ///< 128 samples per second
#define RATE_ADS1015_250SPS             0x0020  ///< 250 samples per second
#define RATE_ADS1015_490SPS             0x0040  ///< 490 samples per second
#define RATE_ADS1015_920SPS             0x0060  ///< 920 samples per second
#define RATE_ADS1015_1600SPS            0x0080 ///< 1600 samples per second (default)
#define RATE_ADS1015_2400SPS            0x00A0 ///< 2400 samples per second
#define RATE_ADS1015_3300SPS            0x00C0 ///< 3300 samples per second

#define RATE_ADS1115_8SPS               0x0000  ///< 8 samples per second
#define RATE_ADS1115_16SPS              0x0020  ///< 16 samples per second
#define RATE_ADS1115_32SPS              0x0040  ///< 32 samples per second
#define RATE_ADS1115_64SPS              0x0060  ///< 64 samples per second
#define RATE_ADS1115_128SPS             0x0080 ///< 128 samples per second (default)
#define RATE_ADS1115_250SPS             0x00A0 ///< 250 samples per second
#define RATE_ADS1115_475SPS             0x00C0 ///< 475 samples per second
#define RATE_ADS1115_860SPS             0x00E0 ///< 860 samples per second


struct ads1115_t
{
    u8 dev_addr; 
    u8 m_bitShift;
    adsGain_t m_gain;
	u16 m_dataRate;

    ADS1115_WR_FUNC_PTR; 
    ADS1115_RD_FUNC_PTR; 
    void (*delay_msec)(ADS1115_MDELAY_DATA_TYPE);
};


ADS1115_RETURN_FUNCTION_TYPE ads1115_init(struct ads1115_t *ads1115);
ADS1115_RETURN_FUNCTION_TYPE ads1115_read_ADC_single_ended(u8 channel);
ADS1115_RETURN_FUNCTION_TYPE ads1115_read_ADC_differential_0_1();
ADS1115_RETURN_FUNCTION_TYPE ads1115_read_ADC_differential_0_3();
ADS1115_RETURN_FUNCTION_TYPE ads1115_read_ADC_differential_1_3();
ADS1115_RETURN_FUNCTION_TYPE ads1115_read_ADC_differential_2_3();
ADS1115_RETURN_FUNCTION_TYPE ads1115_start_comparator_single_ended(u8 channel, s16 threshold);
ADS1115_RETURN_FUNCTION_TYPE ads1115_get_last_conversion_results();
ADS1115_RETURN_FUNCTION_TYPE ads1115_compute_volts(s16 counts);
ADS1115_RETURN_FUNCTION_TYPE ads1115_set_gain(adsGain_t gain);
ADS1115_RETURN_FUNCTION_TYPE ads1115_get_gain();
ADS1115_RETURN_FUNCTION_TYPE ads1115_set_data_rate(u16 rate);
ADS1115_RETURN_FUNCTION_TYPE ads1115_get_data_rate();
ADS1115_RETURN_FUNCTION_TYPE ads1115_start_ADC_reading(u16 mux, bool continuous);
ADS1115_RETURN_FUNCTION_TYPE ads1115_conversion_complete();





#endif // ADS1115_HPP
