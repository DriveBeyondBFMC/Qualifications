#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <math.h>
#include "deca_device_api.h"   // DW1000 API
#include "deca_types.h"
#include "deca_regs.h"
#include "deca_sleep.h"        // Sleep function
#include "deca_spi.h"       // SPI communication
#include "deca_mutex.h"        // Mutex for SPI access
#include "port.h"
#include "trilateration.h"

#define LOW_SPI_SPEED 500000       // Adjust SPI speed if needed
#define HIGH_SPI_SPEED 8000000

// Default communication configuration. 
static dwt_config_t config =
{
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. */
//#define TX_ANT_DLY 16436
//#define RX_ANT_DLY 16436
#define TX_ANT_DLY 0
#define RX_ANT_DLY 32950

/* Frames used in the ranging process */
static uint8 rx_poll_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 distance_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xAA, 0, 0,0, 0, 0};
static uint8 tx_poll_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 angle_msg[] =    {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xFE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 Semaphore_Release[] =    {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0, 0};
static uint8 Tag_Statistics[] =                      {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE1, 0, 0, 0};
static uint8 Master_Release_Semaphore[] =            {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE2, 0, 0, 0};
static uint8 Tag_Statistics_response[] =             {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE3, 0, 0, 0};
static uint8 Master_Release_Semaphore_comfirm[] =    {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE4, 0, 0, 0};

/* Length of the common part of the message (up to and including the function code */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define ALL_MSG_TAG_IDX 3
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
#define ANGLE_MSG_IDX 10
#define LOCATION_FLAG_IDX 11
#define LOCATION_INFO_LEN_IDX 12
#define LOCATION_INFO_START_IDX 13
#define ANGLE_MSG_MAX_LEN 30

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;
static uint8 frame_seq_nb_semaphore = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ? and 1 ? = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 3300

/* Delay between frames, in UWB microseconds */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 2800 //2700 will fail
/* Receive response timeout */
#define RESP_RX_TIMEOUT_UUS 2700

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance;
double final_distance =  0;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
static void compute_angle_send_to_anthor0(int distance1, int distance2, int distance3);
static void distance_mange(void);

//#define TAG
#define TAG_ID 0x0F
#define MASTER_TAG 0x0F
#define MAX_SLAVE_TAG 0x02
#define SLAVE_TAG_START_INDEX 0x01

#define ANTHOR            // select role: ANTHOR -> anchor, TAG -> tag
#define ANCHOR_MAX_NUM 3
#define ANCHOR_IND 0  // select anchor index: 0 1 2

uint8 Semaphore[MAX_SLAVE_TAG];

vec3d AnchorList[ANCHOR_MAX_NUM];
vec3d tag_best_solution;
int Anthordistance[ANCHOR_MAX_NUM];
int Anthordistance_count[ANCHOR_MAX_NUM];

#define ANCHOR_REFRESH_COUNT 5

// Interrupt Callback Function
void dwm1000_interrupt_callback(void)
{
    printf("DW1000 Interrupt Triggered!\n");

    // Example: Read system event status register
    uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
    printf("SYS_STATUS: 0x%08X\n", status);

    // Clear the interrupt (Important!)
    dwt_write32bitreg(SYS_STATUS_ID, status);
}

void Anchor_Array_Init(void)
{
    int anchor_index = 0;
    for(anchor_index = 0; anchor_index < ANCHOR_MAX_NUM; anchor_index++)
    {
        Anthordistance[anchor_index] = 0;
        Anthordistance_count[anchor_index] = 0;
    }
}
void Semaphore_Init(void)
{
    int tag_index = 0 ;
    for(tag_index = 0; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        Semaphore[tag_index]  = 0;
    }
}

int Sum_Tag_Semaphore_request(void)
{
    int tag_index = 0 ;
    int sum_request = 0;
    for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        sum_request+=Semaphore[tag_index];
    }
    return sum_request;
}

void Tag_Measure_Dis(void)
{
    uint8 dest_anthor = 0,frame_len = 0;
    float final_distance = 0;
    for(dest_anthor = 0 ;  dest_anthor<ANCHOR_MAX_NUM; dest_anthor++)
    {
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
        /* Write frame data to DW1000 and prepare transmission.
		 * Polling mode  for demostration
		 */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        tx_poll_msg[ALL_MSG_TAG_IDX] = TAG_ID;
		/*The anchor receives the tag information, which contains TAG_ID. When anchor replies to the tag, it also needs to specify the TAG_ID. Only the TAG_ID will be processed if it is consistent.   */

        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);

        dwt_rxenable(0);// manually enable RX

        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)
                continue;
            rx_buffer[ALL_MSG_TAG_IDX] = 0;

            /* As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 final_tx_time;

                /* Retrieve poll transmission and response reception timestamp. */
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                /* Compute final message transmission time. See NOTE 9 below. */
                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
                final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

                /* Write all timestamps in the final message. See NOTE 10 below. */
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                /* Write and send final message */
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                tx_final_msg[ALL_MSG_TAG_IDX] = TAG_ID;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
                dwt_writetxfctrl(sizeof(tx_final_msg), 0);

                //dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                //dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS*2);
                dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED );

                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { };

                /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                if (status_reg & SYS_STATUS_RXFCG)
                {
                    /* Clear good/fail RX frame event in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)
                        continue;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;

                    /*As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                    rx_buffer[ALL_MSG_SN_IDX] = 0;

                    if (memcmp(rx_buffer, distance_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                       // final_distance = rx_buffer[10] + (float)rx_buffer[11]/100;
                        Anthordistance[rx_buffer[12]] +=(rx_buffer[10]*1000 + rx_buffer[11]*10);
                        Anthordistance_count[rx_buffer[12]] ++;
                        {
                            int Anchor_Index = 0;
                            while(Anchor_Index < ANCHOR_MAX_NUM)
                            {
                                if(Anthordistance_count[Anchor_Index] >=ANCHOR_REFRESH_COUNT )
                                {
                                    distance_mange();
                                    Anchor_Index = 0;
									//clear all
                                    while(Anchor_Index < ANCHOR_MAX_NUM)
                                    {
                                        Anthordistance_count[Anchor_Index] = 0;
                                        Anthordistance[Anchor_Index] = 0;
										Anchor_Index++;
                                    }
                                    break;
                                }
								Anchor_Index++;
                            }
                        }
                    }
                }
                else
                {
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
        /* Execute a delay between ranging exchanges. */
        // deca_sleep(RNG_DELAY_MS);
        frame_seq_nb++;
    }

}


int main(void)
{
	uint8 anthor_index = 0;
    uint8 tag_index = 0;

    uint8 Semaphore_Enable = 0 ;
    uint8 Waiting_TAG_Release_Semaphore = 0;
    int8 frame_len = 0;
	
    // Setup Reset and IRQ pins
	port_init();
	
	printf("Initializing DWM1000...\n");
	reset_DW1000();
	
    // Initialize SPI
	// For initialisation, DW1000 clocks must be temporarily set to crystal speed. 
	init_spi_w_speed(LOW_SPI_SPEED);     

    // Initialize the DW1000 module
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        fprintf(stderr, "DWM1000 initialization failed!\n");
        return -1;
    }
	
	// Enable Interrupts on DW1000 IRQ pin (pin 16 on header)
    port_enable_ext_interrupt(dwm1000_interrupt_callback);

    printf("DWM1000 initialized successfully!\n");
	
	// After initialisation SPI rate can be increased for optimum
	init_spi_w_speed(HIGH_SPI_SPEED); 

    // Read Device ID
    uint32_t dev_id = dwt_readdevid();

    printf("DWM1000 Device ID: 0x%08X\n", dev_id);

    // Check if device ID is valid (should not be 0xFFFFFFFF)
    if (dev_id == 0xFFFFFFFF)
    {
        fprintf(stderr, "ERROR: Invalid Device ID (0xFFFFFFFF). Check wiring, power, and reset sequence!\n");
        return -1;
    }
	
	// Configure DW1000
    dwt_configure(&config);
	dwt_setleds(1);
    // Apply default antenna delay value
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
	
	// Declare Anchor location (modify after setup the anchors)
	AnchorList[0].x =0;
    AnchorList[0].y =0;
    AnchorList[0].z =0;

    AnchorList[1].x =0;
    AnchorList[1].y =2.4;
    AnchorList[1].z =0;

    AnchorList[2].x =2.4;
    AnchorList[2].y =0;
    AnchorList[2].z =0;
    
	int rx_ant_delay =32880;
    int index = 0 ;

#ifdef ANTHOR
    Anchor_Array_Init();
    /* Loop forever initiating ranging exchanges. */
    //KalMan_PramInit();

    while (1)
    {
        /* Clear reception timeout to start next ranging process. */
        dwt_setrxtimeout(0);
        /* Activate reception immediately. */
        dwt_rxenable(0);

        /* Poll for reception of a frame or error/timeout. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event in the DW1000 status register. */
             dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= RX_BUFFER_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
            /* Check that the frame is a poll sent by "DS TWR initiator" example.
             * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */

            if(rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM != ANCHOR_IND)
                continue;

            anthor_index = rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM;
            tag_index = rx_buffer[ALL_MSG_TAG_IDX];

            rx_buffer[ALL_MSG_SN_IDX] = 0;
            rx_buffer[ALL_MSG_TAG_IDX] = 0;

            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                /* Retrieve poll reception timestamp. */
                poll_rx_ts = get_rx_timestamp_u64();           

                /* Set expected delay and timeout for final message reception. */
                dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                /* Write and send the response message. See NOTE 9 below.*/
                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                tx_resp_msg[ALL_MSG_TAG_IDX] = tag_index;
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0);
                dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
			  
                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { };

                if (status_reg & SYS_STATUS_RXFCG)
                {
                    /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    if(tag_index != rx_buffer[ALL_MSG_TAG_IDX])
                        continue;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;
                    if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                        uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
                        uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                        double Ra, Rb, Da, Db;
                        int64 tof_dtu;

                        /* Retrieve response transmission and final reception timestamps. */
                        resp_tx_ts = get_tx_timestamp_u64();
                        final_rx_ts = get_rx_timestamp_u64();

                        /* Get timestamps embedded in the final message. */
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                        /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 10 below. */
                        poll_rx_ts_32 = (uint32)poll_rx_ts;
                        resp_tx_ts_32 = (uint32)resp_tx_ts;
                        final_rx_ts_32 = (uint32)final_rx_ts;
                        Ra = (double)(resp_rx_ts - poll_tx_ts);
                        Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                        Da = (double)(final_tx_ts - resp_rx_ts);
                        Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                        tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                        tof = tof_dtu * DWT_TIME_UNITS;
                        distance = tof * SPEED_OF_LIGHT;
                        distance = distance - dwt_getrangebias(config.chan,(float)distance, config.prf);//after correction
                        
						//printf("before kalman fliter Distance:%3.2f m\r\n",rx_buffer[12],final_distance);
                        //kalman filter
                        //distance =  KalMan_Update(&distance);
                        //  sprintf(dist_str, "dis: %3.2f m", distance);
                        //    printf("after kalman fliter Distance:%3.2f m\r\n",rx_buffer[12],final_distance);
                        
						// Send calculation results to TAG
                        int temp = (int)(distance*100);
                        distance_msg[10] = temp/100;
                        distance_msg[11] = temp%100;  // get 2 digits after the decimal point
                        distance_msg[12] = anthor_index;

                        distance_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                        distance_msg[ALL_MSG_TAG_IDX] = tag_index;
                        dwt_writetxdata(sizeof(distance_msg), distance_msg, 0);
                        dwt_writetxfctrl(sizeof(distance_msg), 0);

                        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
                         * set by dwt_setrxaftertxdelay() has elapsed. */
                        dwt_starttx(DWT_START_TX_IMMEDIATE );
                      //  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                      //  { };

                    }
                }
                else
                {
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }

            else if (memcmp(rx_buffer, angle_msg, ALL_MSG_COMMON_LEN) == 0 && ANCHOR_IND == 0)
            {
                if(rx_buffer[LOCATION_FLAG_IDX] == 1)//location infomartion
                {
                    rx_buffer[ALL_MSG_TAG_IDX] = tag_index;
                    //USART_puts(&rx_buffer[LOCATION_INFO_START_IDX],rx_buffer[LOCATION_INFO_LEN_IDX]);
                }
                else //follow car
                {
                    putchar(rx_buffer[10]);
                }
            }
        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
#endif

#ifdef TAG
    /* Set expected response's delay and timeout
     */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    if(TAG_ID ==  MASTER_TAG)
    {
        Semaphore_Enable = 1 ;
        Semaphore_Init();
        Waiting_TAG_Release_Semaphore = 0;
    }
    else
    {
        Semaphore_Enable = 0 ;
    }
    //Master TAG0
    while(1)
    {
        if(Semaphore_Enable == 1)
        {
            //send message to anthor,TAG<->ANTHOR
            Tag_Measure_Dis();//measure distance between tag and all anthor
            Semaphore_Enable = 0 ;

            if(TAG_ID != MASTER_TAG)
            {
                //send release semaphore to master tag
                Semaphore_Release[ALL_MSG_SN_IDX] = frame_seq_nb;
                Semaphore_Release[ALL_MSG_TAG_IDX] = TAG_ID;
                dwt_writetxdata(sizeof(Semaphore_Release), Semaphore_Release, 0);
                dwt_writetxfctrl(sizeof(Semaphore_Release), 0);

                dwt_starttx(DWT_START_TX_IMMEDIATE );
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                { };
            }
        }

        if(TAG_ID == MASTER_TAG)//master  tag
        {
            //statistics tag
            if(Sum_Tag_Semaphore_request() == 0)
            {
                for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
                {
                    Tag_Statistics[ALL_MSG_SN_IDX] = 0;
                    Tag_Statistics[ALL_MSG_TAG_IDX] = tag_index;
                    dwt_writetxdata(sizeof(Tag_Statistics), Tag_Statistics, 0);
                    dwt_writetxfctrl(sizeof(Tag_Statistics), 0);
                    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

                    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                    { };

                    if (status_reg & SYS_STATUS_RXFCG)
                    {
                        /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                        /* A frame has been received, read it into the local buffer. */
                        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                        if (frame_len <= RX_BUF_LEN)
                        {
                            dwt_readrxdata(rx_buffer, frame_len, 0);
                        }
                        rx_buffer[ALL_MSG_SN_IDX] = 0;

                        if(rx_buffer[ALL_MSG_TAG_IDX] == tag_index)
                        {
                            uint8 temp = rx_buffer[ALL_MSG_TAG_IDX] ;
                            rx_buffer[ALL_MSG_TAG_IDX] =0;
                            if (memcmp(rx_buffer, Tag_Statistics_response, ALL_MSG_COMMON_LEN) == 0)
                            {
                                Semaphore[temp] = 1;
                            }
                        }
                    }
                    else
                    {
                        /* Clear RX error events in the DW1000 status register. */
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                    }
                }
                //print all the tags in network
                for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
                {
                    if(Semaphore[tag_index] == 1)
                    {
                        // printf("Tag%d In NetWork!\r\n",tag_index);
                    }
                }
            }

            // pick one tag ,send Semaphore message
            // release to specific tag(TAG ID)
            // master tag send release signal,and the specific tag send comfirm message
            if(Waiting_TAG_Release_Semaphore == 0 && Sum_Tag_Semaphore_request() != 0)
            {
                Semaphore[0] = 0;//slave tag must not use tag_id = 0x00!!
                for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
                {
                    if(Semaphore[tag_index] == 1)
                    {
                        // printf("Release Semaphore to Tag%d!\r\n",tag_index);
                        // dwt_setrxtimeout(0);

                        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                        Master_Release_Semaphore[ALL_MSG_SN_IDX] = 0;
                        Master_Release_Semaphore[ALL_MSG_TAG_IDX] = tag_index;
                        dwt_writetxdata(sizeof(Master_Release_Semaphore), Master_Release_Semaphore, 0);
                        dwt_writetxfctrl(sizeof(Master_Release_Semaphore), 0);
                        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

                        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                        { };

                        if (status_reg & SYS_STATUS_RXFCG)
                        {
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                            if (frame_len <= RX_BUF_LEN)
                            {
                                dwt_readrxdata(rx_buffer, frame_len, 0);
                            }
                            rx_buffer[ALL_MSG_SN_IDX] = 0;

                            if(rx_buffer[ALL_MSG_TAG_IDX] == tag_index)
                            {
                                rx_buffer[ALL_MSG_TAG_IDX] = 0;
                                // USART_puts(rx_buffer,frame_len);
                                if (memcmp(rx_buffer, Master_Release_Semaphore_comfirm, ALL_MSG_COMMON_LEN) == 0)
                                {
                                    //if the tag recive a semaphore, wait release remaphore
                                    Waiting_TAG_Release_Semaphore ++;
                                    break;//only release one semphore once
                                }
                            }
                        }
                        else//the tag may leave net,clear semaphore
                        {
                            Semaphore[tag_index] = 0 ;
                            //printf("tag may leave net!\r\n");
                            /* Clear RX error events in the DW1000 status register. */
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                        }
                    }
                }
            }

            if(Waiting_TAG_Release_Semaphore == 0 )
            {
                // TODO
            }
            //Master tag waitting for specific tag Semaphore Release message
            if( Waiting_TAG_Release_Semaphore >0)
            {
                //  printf("Waiting for Release Semaphore!\r\n");
                dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS*5);//about 10ms,need adjust!!
                dwt_rxenable(0);
                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { };

                if (status_reg & SYS_STATUS_RXFCG)
                {
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                    if (frame_len <= RX_BUFFER_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    uint8 temp=rx_buffer[ALL_MSG_TAG_IDX] ;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;
                    if (memcmp(rx_buffer, Semaphore_Release, ALL_MSG_COMMON_LEN) == 0)
                    {
                        if(Semaphore[temp] == 1)
                        {
                            Semaphore[temp] = 0 ;
                            if(Waiting_TAG_Release_Semaphore > 0 )
                            {
                                Waiting_TAG_Release_Semaphore --;
                            }
                        }
                    }
                }
                else
                {
                    //maybe the tag leave network
                    if(Waiting_TAG_Release_Semaphore > 0)
                    {
                        Waiting_TAG_Release_Semaphore--;
                        Semaphore[tag_index] = 0 ;
                    }
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
            /* Possible problems are that the TAG leaves the network after receiving the Semaphore without releasing it, causing the Master TAG to be unable to recover the Semaphore. This requires a timer to be implemented. After a period of time, if the TAG still does not receive the Semaphore and releases the Semaphore, it needs to be forced to cancel. */
            //if all tag have serviced by  master tag -> master tag can measure the distance
            if(Sum_Tag_Semaphore_request() == 0)
            {
                Semaphore_Enable = 1 ;
                Waiting_TAG_Release_Semaphore= 0;
            }
        }
        else  //slave tags
        {
            //SLAVE TAG starts by default waiting for MASTER TAG to send statistical information and release the semaphore
            dwt_setrxtimeout(0);
            dwt_rxenable(0);

            /* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
            { };

            if (status_reg & SYS_STATUS_RXFCG)
            {
                /* Clear good RX frame event in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG|SYS_STATUS_TXFRS);//clear rx & tx flag at the same time

                /* A frame has been received, read it into the local buffer. */
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                if (frame_len <= RX_BUFFER_LEN)
                {
                    dwt_readrxdata(rx_buffer, frame_len, 0);
                }
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if(rx_buffer[ALL_MSG_TAG_IDX] == TAG_ID)
                {
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;
                    if (memcmp(rx_buffer, Tag_Statistics, ALL_MSG_COMMON_LEN) == 0)
                    {
                        //GPIO_SetBits(GPIOA,GPIO_Pin_3);
                        Tag_Statistics_response[ALL_MSG_SN_IDX] = frame_seq_nb;
                        Tag_Statistics_response[ALL_MSG_TAG_IDX] = TAG_ID;
                        dwt_writetxdata(sizeof(Tag_Statistics_response), Tag_Statistics_response, 0);
                        dwt_writetxfctrl(sizeof(Tag_Statistics_response), 0);
                        dwt_starttx(DWT_START_TX_IMMEDIATE );
                    }

                    if (memcmp(rx_buffer, Master_Release_Semaphore, ALL_MSG_COMMON_LEN) == 0)
                    {
                        Master_Release_Semaphore_comfirm[ALL_MSG_SN_IDX] = frame_seq_nb;
                        Master_Release_Semaphore_comfirm[ALL_MSG_TAG_IDX] = TAG_ID;
                        dwt_writetxdata(sizeof(Master_Release_Semaphore_comfirm), Master_Release_Semaphore_comfirm, 0);
                        dwt_writetxfctrl(sizeof(Master_Release_Semaphore_comfirm), 0);

                        dwt_starttx(DWT_START_TX_IMMEDIATE);
                        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                        { };

                        Semaphore_Enable = 1;
                    }
                }
            }
            else
            {
                /* Clear RX error events in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            }
        }
    }
#endif

    return 0;
}

#define Filter_N 5  //max filter use in this system
#define Filter_D 5  //each filter contain "Filter_D" data
int Value_Buf[Filter_N][Filter_D]= {0};
int filter_index[Filter_N] = {0};
int filter(int input, int fliter_idx )
{
    char count = 0;
    int sum = 0;
    if(input > 0)
    {
        Value_Buf[fliter_idx][filter_index[fliter_idx]++]=input;
        if(filter_index[fliter_idx] == Filter_D) filter_index[fliter_idx] = 0;

        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }
    else
    {
        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }

}

static void distance_mange(void)
{
    // if(GetLocation(&tag_best_solution, 0, &(AnchorList[0]), &(Anthordistance[0])) != -1)
    // {
    // printf("Tag Location:x=%3.2fm y=%3.2fm z=%3.2fm\r\n",tag_best_solution.x,tag_best_solution.y,tag_best_solution.z);
    // sprintf(dist_str, "x:%3.2f y:%3.2f",tag_best_solution.x,tag_best_solution.y);
    // OLED_ShowString(0,0,dist_str);
    //  }
    //GetLocation2(&tag_best_solution, 0, &(AnchorList[0]), &(Anthordistance[0]));
    //Th_Location( &(AnchorList[0]), &(Anthordistance[0]));
    //Th_Location2( &(AnchorList[0]), &(Anthordistance[0]));
    // OLED_ShowString(0, 2,"pass");

    //printf("Count:%d %d %d \r\n",Anthordistance[0]/Anthordistance_count[0],Anthordistance[1]/Anthordistance_count[1],Anthordistance[2]/Anthordistance_count[2]);
   // Anthordistance[0] =filter((int)(Anthordistance[0]/Anthordistance_count[0]),0);
   // Anthordistance[1] =filter((int)(Anthordistance[1]/Anthordistance_count[1]),1);
   // Anthordistance[2] = filter((int)(Anthordistance[2]/Anthordistance_count[2]),2);
    //printf("Count:%d %d %d \r\n",Anthordistance_count[0],Anthordistance_count[1],Anthordistance_count[2]);
    //printf("Count:%d %d %d \r\n",Anthordistance[0],Anthordistance[1],Anthordistance[2]);
    //printf(" \r\n");

    {
        int Anchor_Index = 0;
        while(Anchor_Index < ANCHOR_MAX_NUM)
        {
            if(Anthordistance_count[Anchor_Index] > 0 )
            {
                Anthordistance[Anchor_Index] =filter((int)(Anthordistance[Anchor_Index]/Anthordistance_count[Anchor_Index]),Anchor_Index);
            }
			Anchor_Index++;
        }
    }

    compute_angle_send_to_anthor0(Anthordistance[0], Anthordistance[1],Anthordistance[2]);


/*     if(Anthordistance_count[0]>0)
    {
        sprintf(dist_str, "an0:%3.2fm", (float)Anthordistance[0]/1000);
        OLED_ShowString(0, 2," 		   ");
        OLED_ShowString(0, 2,dist_str);
    }


    if(Anthordistance_count[1]>0)
    {
        sprintf(dist_str, "an1:%3.2fm", (float)Anthordistance[1]/1000);
        OLED_ShowString(0, 4,"		 ");
        OLED_ShowString(0, 4,dist_str);
    }


    if(Anthordistance_count[2]>0)
    {
        sprintf(dist_str, "an2:%3.2fm", (float)Anthordistance[2]/1000);
        OLED_ShowString(0, 6,"		 ");
        OLED_ShowString(0, 6,dist_str);
    } */
    // printf("Distance:%d,   %d,    %d mm\r\n",(int)((float)Anthordistance[0]/Anthordistance_count[0]),(int)((float)Anthordistance[1]/Anthordistance_count[1]),(int)((float)Anthordistance[2]/Anthordistance_count[2]));
}





#define DISTANCE3 0.27

//**************************************************************//
//distance1 anthor0 <--> TAG  mm
//distance2 anthor1 <--> TAG  mm
//distance3 anthor2 <--> TAG  mm
//**************************************************************//
static void compute_angle_send_to_anthor0(int distance1, int distance2,int distance3)
{
    static int framenum = 0 ;

#if 0 //compute angle for smartcar
    float dis3_constans = DISTANCE3;
    float cos = 0;
    float angle = 0 ;
    float dis1 = (float)distance1/1000; //m
    float dis2 = (float)distance2/1000;  //m

    if(dis1 + dis3_constans < dis2 || dis2+dis3_constans < dis1)
    {
        //printf("ERROR!\r\n");
        //return;
    }
    cos = (dis1*dis1 + dis3_constans* dis3_constans - dis2*dis2)/(2*dis1*dis3_constans);
    angle  = acos(cos)*180/3.1415926;
    printf("cos = %f, arccos = %f\r\n",cos,angle);
    sprintf(dist_str, "angle: %3.2f m", angle);
    OLED_ShowString(0, 6,"            ");
    OLED_ShowString(0, 6,dist_str);

    if(dis1 > 1)
    {
        if(angle > 110)
        {
            printf("turn right\r\n");
            angle_msg[10] = 'R';
        }
        else if(angle < 75)
        {
            printf("turn left\r\n");
            angle_msg[10] = 'L';
        }
        else
        {
            printf("forward\r\n");
            angle_msg[10] = 'F';
        }
    }
    else
    {
        printf("stay here\r\n");
        angle_msg[10] = 'S';
    }
    angle_msg[LOCATION_FLAG_IDX] = 0;

#else
    //location
    {
        uint8 len = 0;
        angle_msg[LOCATION_FLAG_IDX] = 1;

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 'm';
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 'r';

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 0x02;
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = TAG_ID;//TAG ID

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)(framenum&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((framenum>>8)&0xFF);

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10 >>8)&0xFF);

        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance2/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance2/10 >>8)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance3/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance3/10 >>8)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10 >>8)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = '\n';
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = '\r';


        angle_msg[LOCATION_INFO_LEN_IDX] = len;
        //MAX LEN
        if(LOCATION_INFO_START_IDX + len -2 >ANGLE_MSG_MAX_LEN)
        {
            while(1);//toggle LED
        }
        //USART_puts((char*)dist_str,16);

    }
#endif
    //only anthor0 recive angle message
    angle_msg[ALL_MSG_SN_IDX] = framenum;
    angle_msg[ALL_MSG_TAG_IDX] = TAG_ID;

    dwt_writetxdata(sizeof(angle_msg), angle_msg, 0);
    dwt_writetxfctrl(sizeof(angle_msg), 0);

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE );
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
    { };

    framenum++;

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}