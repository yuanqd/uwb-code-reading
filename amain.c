/*! ----------------------------------------------------------------------------
 *  @file    amain.c
 *  @brief   单侧双向测距发起者示例代码(SS TWR)
 *
 * 这是个简单代码示例，该程序在SS TWR距离测量交换中作为发起者角色，该程序发送一个"poll"帧(记录poll的TX时间戳),之后等待一个来自"DS TWR responder"的响应消息
 * 那部分示例代码与本程序一起完成消息交换，响应消息包含远方响应者的poll的时间戳（The response message contains the remote responder's time-stamps of poll）
 *  RX, 和响应TX.使用这些数据和本地的时间戳, (of poll TX and response RX),该示例程序计算出飞行时间的值（the time-of-flight over-the-air）,然后,两个设备间的估计距离写到LCD.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "lcd.h"
#include "port.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME "SS TWR INIT v1.1"

/* yuanqd:设置测距时间间隔，毫秒Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* 默认通信配置.这里使用EVK1000的模式看下面的NOTE 1. */
static dwt_config_t config = {
    2,               /* yuanqd:通道编号//Channel number. */
    DWT_PRF_64M,     /* yuanqd:脉冲重复频率//Pulse repetition frequency. */
    DWT_PLEN_128,    /* yuanqd:前导码长度.// Preamble length*/
    DWT_PAC8,        /* yuanqd:报头捕获数据块，是在DW1000模块数据接收过程中需要检测接收数据的一种机制。报头捕获数据块，是指在接收器里面，在报头检测过程中收集在一起的一组报头符号，可以通过改变DRX_TUNE2来设置该数据块的大小。（Preamble acquisition chunk size. Used in RX only. */
    9,               /* yuanqd:TX的前导码，只TX使用// preamble code. Used in TX only. */
    9,               /* yuanqd:RX的前导码，只RX使用// preamble code. Used in RX only. */
    0,               /* yuanqd:使用非标准的帧定界符(Start Frame Delimiter)(布尔值)//Use non-standard SFD (Boolean) */
    DWT_BR_6M8,      /* 数据速率//Data rate. */
    DWT_PHRMODE_STD, /* yuanqd:物理层报头模式//PHY header mode. */
    (129 + 8 - 8)    /* yuanqd:帧定界符超时(preamble length + 1 + SFD length - PAC size)//SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* yuanqd:64MHz的PRF的默认天线时延时，见下面的NOTE2。PRF（pulse repetition frequency）即脉冲重复频率。//Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* yuanqd:用于测距过程的帧//Frames used in the ranging process. See NOTE 3 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* yuanqd:消息公共部分的长度（包含函数代码，见NOTW 3）//Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* yuanqd:访问上面定义的帧的一些域（字段）//Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* yuanqd:帧序列号，每次传输递增//Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* yuanqd:用于存放接收到的响应信息的缓存//Buffer to store received response message.
 * 这个大小根据本示例代码准备处理的最长帧调整//Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* yuanqd:保持寄存器状态的copy用以引用，这样读取器就可以在断点时检查//Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* yuanqd:UWB微秒（uus）转设备时间单位的转换因子（参数）//UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs  and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* yuanqd:帧间延迟，单位为UWB的微秒，见后文NOTE 1//Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 140
/* yuanqd:接收响应过期时间，见后面NOTE 5//Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 210

/* yuanqd:光在空气中的速度//Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* yuanqd:保持计算获得的飞行时间和距离拷贝，读取器可以在断点查看//Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance;

/* yuanqd: 在LCD上显示测量距离的字符串（最大16个字符）//String used to display measured distance on LCD screen (16 characters maximum). */
char dist_str[16] = {0};

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn main()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int main(void)
{
    /* yuanqd: 初始化板子具体的硬件，并启动//Start with board specific hardware init. */
    peripherals_init();

    /* yuanqd:显示应用名到LCD上//Display application name on LCD. */
    lcd_display_str(APP_NAME);

    /* yuanqd:复位并初始化DW1000//Reset and initialise DW1000.
     * 对于初始化，DW1000时钟必须临时设置为晶振速度，初始化SPI速率后可以再增加到最佳性能//For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();
    dwt_initialise(DWT_LOADUCODE);
    spi_set_rate_high();

    /* yuanqd:配置DW1000.见下面的NOTE6//Configure DW1000. See NOTE 6 below. */
    dwt_configure(&config);

    /* yuanqd:应用默认的天线时延值，见后面的NOTE2//Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* yuanqd:设置期望的响应时延和超时时长。见NOTE 1和 NOTE 5//Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * yuanqd:本例中只是使用同样的时延和超时时常来处理一个进来的帧，这些值可以在这里一次性设置//As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* yuanqd:死循环，初始化测距交换//Loop forever initiating ranging exchanges. */
    while (1)
    {
        /*yuanqd:写帧数据到DW1000,并准备发送，见下面的NOTE 7.// Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

        /* yuanqd:开始发送，意味着期待一个响应，所以在帧发送后，并等待dwt_setrxaftertxdelay（）函数设置的延时时间过去后 ，接收机制会被自动使能//Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* yuanqd:我们假设发送的消息正确到达, 一个帧的接收或者错误超时的poll，见NOTE 8//We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        /* yuanqd:完成一个poll消息的传输后增加帧序列号（modulo 256）//Increment frame sequence number after transmission of the poll message (modulo 256). */
        frame_seq_nb++;

        if (status_reg & SYS_STATUS_RXFCG)
        {
            uint32 frame_len;

            /* yuanqd:从DW1000状态寄存器中清除好的RX帧事件//Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            /* yuanqd:接收到一个帧后，将之读入到本地缓存//A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* yuanqd:检查这个帧是从伙伴"SS TWR responder"发来的期望的回应//Check that the frame is the expected response from the companion "SS TWR responder" example.
             * yuanqd:作为帧的序列号域没有关联，显然是为了简化帧校验//As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                int32 rtd_init, rtd_resp;

                /* yuanqd:取回poll发送与接收响应时间戳，见下面的NOTE 9//Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
                poll_tx_ts = dwt_readtxtimestamplo32();
                resp_rx_ts = dwt_readrxtimestamplo32();

                /* yuanqd:获取嵌入到响应消息里的时间戳//Get timestamps embedded in response message. */
                resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                /* yuanqd:计算飞行时间与距离//Compute time of flight and distance. */
                rtd_init = resp_rx_ts - poll_tx_ts;
                rtd_resp = resp_tx_ts - poll_rx_ts;

                tof = ((rtd_init - rtd_resp) / 2.0) * DWT_TIME_UNITS;
                distance = tof * SPEED_OF_LIGHT;

                /* yuanqd:在LCD上显示计算获得的距离//Display computed distance on LCD. */
                sprintf(dist_str, "DIST: %3.2f m", distance);
                lcd_display_str(dist_str);
            }
        }
        else
        {
            /* yuanqd:清除DW1000状态寄存器上的RX错误事件//Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }

        /* yuanqd:在两次距离交换之间执行一个延时//Execute a delay between ranging exchanges. */
        deca_sleep(RNG_DELAY_MS);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn resp_msg_get_ts()
 *
 * @brief yuanqd:从响应消息里读取一个给定的时间戳的值。 在响应消息的时间戳字段里，越不重要的字节地址越低//Read a given timestamp value from the response message. In the timestamp fields of the response message, the
 *        least significant byte is at the lower address.
 *
 * @param  ts_field  yuanqd:时间戳字段的第一个字节指针，用来获得时间戳值//pointer on the first byte of the timestamp field to get
 *         ts  timestamp value
 *
 * @return none
 */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The single-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
 *    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
 *    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
 *    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the single-sided two-way ranging
 *    process.
 * 2. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 3. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
 *    following:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 4 below.
 *     - byte 7/8: source address, see NOTE 4 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10 -> 13: poll message reception timestamp.
 *     - byte 14 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 4. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    6.8M data rate used (around 200 祍).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 7. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *    subtraction.
 * 10. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
