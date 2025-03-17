#include "drivers/ads1115.hpp"
#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *) 0)
#endif
#endif
// TODO: Add your code here
struct ads1115_t *p_ads1115;

ADS1115_RETURN_FUNCTION_TYPE ads1115_init(struct ads1115_t *ads1115) {
    p_ads1115 = ads1115;
    return ADS1115_SUCCESS;
}

ADS1115_RETURN_FUNCTION_TYPE ads1115_set_gain(adsGain_t gain){
    p_ads1115->m_gain = gain;
    return ADS1115_SUCCESS;
}

ADS1115_RETURN_FUNCTION_TYPE ads1115_get_gain(adsGain_t *gain){
    *gain = p_ads1115->m_gain;
    return ADS1115_SUCCESS;
}

ADS1115_RETURN_FUNCTION_TYPE ads1115_set_data_rate(u16 rate) {
    p_ads1115->m_dataRate = rate;
    return ADS1115_SUCCESS;
}

ADS1115_RETURN_FUNCTION_TYPE ads1115_get_data_rate(u16 *m_dataRate) { 
    *m_dataRate = p_ads1115->m_dataRate;
    return ADS1115_SUCCESS;
}

ADS1115_RETURN_FUNCTION_TYPE ads1115_start_ADC_reading(u16 mux, bool continuous){
    ADS1115_RETURN_FUNCTION_TYPE com_rslt = ADS1115_INIT_VALUE; 
    u16 config =
      ADS1115_REG_CONFIG_CQUE_1CONV |   // Set CQUE to any value other than
                                        // None so we can use it in RDY mode
      ADS1115_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1115_REG_CONFIG_CMODE_TRAD;    // Traditional comparator (default val)

    if (continuous) {
        config |= ADS1115_REG_CONFIG_MODE_CONTIN;
    } else {
        config |= ADS1115_REG_CONFIG_MODE_SINGLE;
    }
    // Set PGA/voltage range
    config |= p_ads1115->m_gain;
    // Set data rate
    config |= p_ads1115->m_dataRate;
    // Set channels
    config |= mux;
    // Set 'start single-conversion' bit
    config |= ADS1115_REG_CONFIG_OS_SINGLE;
    // Write config register to the ADC
    com_rslt = p_ads1115->bus_write(p_ads1115->dev_addr, ADS1115_REG_POINTER_CONFIG, config);
    com_rslt = p_ads1115->bus_write(p_ads1115->dev_addr, ADS1115_REG_POINTER_HITHRESH, 0x8000);
    com_rslt = p_ads1115->bus_write(p_ads1115->dev_addr, ADS1115_REG_POINTER_LOWTHRESH, 0x0000);
    
    return com_rslt;
}


ADS1115_RETURN_FUNCTION_TYPE ads1115_conversion_complete(u8 *data) {
    ADS1115_RETURN_FUNCTION_TYPE com_rslt = ADS1115_INIT_VALUE; 
    u8 tmp = 0;
    com_rslt = p_ads1115->bus_read(p_ads1115->dev_addr, ADS1115_REG_POINTER_CONFIG, &tmp);
    *data = tmp & 0x8000;
    return com_rslt;
}

// ADS1115_RETURN_FUNCTION_TYPE ads1115_read_ADC_single_ended(u8 channel){
//     if (channel > 3) {
//         return 0;
//     }
//     ads1115_start_ADC_reading(MUX_BY_CHANNEL[channel], /*continuous=*/false);
//     while (!ads1115_conversion_complete());
//     getLastConversionResults();
//     return ADS1115_SUCCESS;
// }