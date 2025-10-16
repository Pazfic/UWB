/*! ----------------------------------------------------------------------------
 * @file    deca_table.c
 * @brief   HW specific definitions and functions for portability
 *
 * @author  Decawave
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @modified Path Partner 2018
 */

#include "dwt_common.h"

/******************************************************************************
 *
 *  ѡ������ģʽ  ( indexed 0 to 5 )
 *
 ******************************************************************************/

chConfig_t chConfig[6] = {
    // MODES 0
    {
        2,               // channel
        DWT_PRF_16M,     // prf
        DWT_BR_110K,     // datarate
        4,               // preambleCode
        DWT_PLEN_1024,   // preambleLength
        DWT_PAC32,       // pacSize
        1,               // non-standard SFD
        (1025 + 64 - 32) // SFD timeout    SFD length :64 for 110k (always)   8 for 6.81M  16 for 850k
    },                   // SFD timeout (preamble length + 1 + SFD length - PAC size)

    // MODES 1
    {
        2,              // channel
        DWT_PRF_16M,    // prf
        DWT_BR_850K,    // datarate
        4,              // preambleCode
        DWT_PLEN_256,   // preambleLength
        DWT_PAC16,      // pacSize
        1,              // non-standard SFD  ����
        (257 + 16 - 16) // SFD timeout
    },
    // MODES 2
    {
        2,            // channel
        DWT_PRF_16M,  // prf
        DWT_BR_6M8,   // datarate
        4,            // preambleCode
        DWT_PLEN_128, // preambleLength
        DWT_PAC8,     // pacSize
        0,            // non-standard SFD
        (129 + 8 - 8) // SFD timeout
    },
    // MODES 3
    {
        5,               // channel
        DWT_PRF_16M,     // prf
        DWT_BR_110K,     // datarate
        3,               // preambleCode
        DWT_PLEN_1024,   // preambleLength
        DWT_PAC32,       // pacSize
        1,               // non-standard SFD
        (1025 + 64 - 32) // SFD timeout
    },

    // MODES 4
    {
        5,              // channel
        DWT_PRF_16M,    // prf
        DWT_BR_850K,    // datarate
        3,              // preambleCode
        DWT_PLEN_256,   // preambleLength
        DWT_PAC16,      // pacSize
        1,              // non-standard SFD  ����
        (257 + 16 - 16) // SFD timeout
    },
    // MODES 5
    {
        5,            // channel
        DWT_PRF_16M,  // prf
        DWT_BR_6M8,   // datarate
        3,            // preambleCode
        DWT_PLEN_128, // preambleLength
        DWT_PAC8,     // pacSize
        0,            // non-standard SFD
        (129 + 8 - 8) // SFD timeout
    }};

// timeConfig_t timeconfig[6]=
//{
//
//				 {
//						150,
//						4200,
//						3900,
//						3500,
//						200,
//						9000
//				 },

//				 {
//						150,
//						2300,
//						2100,
//						2000,
//						300,
//						2500
//				 },
//				 {
//						1300,//�ж�
//						6500,
//						1700,
//						1500,
//						200,
//						6700
//				 },
//				 {
//						150,
//						4200,
//						3900,
//						3500,
//						200,
//						9000
//				 },
//				 {
//						150,
//						2300,
//						2100,
//						2000,
//						300,
//						2500
//				 },
//				 {
//						150,
//						1900,
//						1700,
//						1500,
//						400,
//						2500
//				 }

//};

// The table below specifies the default TX spectrum configuration parameters... this has been tuned for DW EVK hardware
// units
// the table is set for smart power - see below in the instance_config function how this is used when not using smart
// power
const tx_structr txSpectrumConfigr[8] = {
    // Channel 0 ----- this is just a place holder so the next array element is channel 1
    {0x0, // 0
     {{
          0x0, // 0
          0x0  // 0
      },
      {
          0x0, // 0
          0x0  // 0
      }

     }},
    // Channel 1
    {0xc9, // PG_DELAY
     {{
          // smartPowerEn=1
          0x15355575, // 16M
          0x07274767  // 64M
      },
      {
          // smartPowerEn=0
          0x75757575, // 16M
          0x67676767  // 64M
      }

     }

    },
    // Channel 2
    {0xc2, // PG_DELAY
     {{
          // smartPowerEn=1
          0x39393939, // 16M
          0x07274767  // 64M
      },
      {
          // smartPowerEn=0
          0x39393939, // 16M
          0x67676767  // 64M
      }

     }},
    // Channel 3
    {0xc5, // PG_DELAY
     {{
          // smartPowerEn=1
          0x0F2F4F6F, // 16M
          0x2B4B6B8B  // 64M
      },
      {
          // smartPowerEn=0
          0x6F6F6F6F, // 16M
          0x8B8B8B8B  // 64M
      }

     }},
    // Channel 4
    {0x95, // PG_DELAY
     {{
          // smartPowerEn=1
          0x1F1F3F5F, // 16M
          0x3A5A7A9A  // 64M
      },
      {
          // smartPowerEn=0
          0x5F5F5F5F, // 16M
          0x9A9A9A9A  // 64M
      }

     }},
    // Channel 5
    {0xc0, // PG_DELAY
     {{
          // smartPowerEn=1
          0x0E082848, // 16M
          0x25456585  // 64M
      },
      {
          // smartPowerEn=0
          0x4848484, // 16M
          0x85858585 // 64M
      }

     }},
    // Channel 6 ----- this is just a place holder so the next array element is channel 7
    {0x0, // 0
     {{
          0x0, // 0
          0x0  // 0
      },
      {
          0x0, // 0
          0x0  // 0
      }

     }},
    // Channel 7
    {0x93, // PG_DELAY
     {{
          // smartPowerEn=1
          0x32527292, // 16M
          0x5171B1D1  // 64M
      },
      {
          // smartPowerEn=0
          0x92929292, // 16M
          0xD1D1D1D1  // 64M
      }

     }}};

unsigned int deID = 0;

uint32 power = 0;

int MODES = 0;
int T_ID = 0;
int A_ID = 0;

int MODULE_ROLE;
// timeConfig_t tconfig;
void addressconfigure(int modes) {
    uint16 instAddress;

    instAddress = sys_para.uwbid;

    insaddress(instAddress);
}
// returns the use case / operational mode
int decarangingmode() {
    int mode = 0;

    if (sys_para.role == 0x01) {
        A_ID = sys_para.uwbid;
        MODULE_ROLE = 1;
    }
    if (sys_para.role == 0x02) {
        T_ID = sys_para.uwbid;
        MODULE_ROLE = 0;
    }

    if ((sys_para.channel == 0x02) && (sys_para.datarate == 0x01))
        mode = 0;
    else if ((sys_para.channel == 0x02) && (sys_para.datarate == 0x02))
        mode = 1;
    else if ((sys_para.channel == 0x02) && (sys_para.datarate == 0x03))
        mode = 2;
    else if ((sys_para.channel == 0x05) && (sys_para.datarate == 0x01))
        mode = 3;
    else if ((sys_para.channel == 0x05) && (sys_para.datarate == 0x02))
        mode = 4;
    else if ((sys_para.channel == 0x05) && (sys_para.datarate == 0x03))
        mode = 5;
    return mode;
}

int deca_configure(void) {
    int result;
    dwt_config_t config;
    dwt_txconfig_t txconfig;

    // reset the DW1000 by driving the RSTn line low
    reset_DW1000();

    port_stop_all_UWB(); /**< switch off all UWB and set callbacks to NULL */
    port_DisableEXT_IRQ();
    deID = dwt_readdevid();

    if (DWT_DEVICE_ID != deID) // if the read of device ID fails, the DW1000 could be asleep
    {
        port_SPIx_clear_chip_select(); // CS low
        HAL_Delay(1);                  // 200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
        port_SPIx_set_chip_select();   // CS high
        HAL_Delay(7);
        deID = dwt_readdevid();
        // SPI not working or Unsupported Device ID
        if (DWT_DEVICE_ID != deID) return (-1);
        // clear the sleep bit - so that after the hard reset below the DW does not go into sleep
        // dwt_softreset();
    }
    // spi_set_rate_low();
    result = dwt_initialise(DWT_LOADUCODE | DWT_READ_OTP_PID | DWT_READ_OTP_LID | DWT_READ_OTP_BAT |
                            DWT_READ_OTP_TMP);        // 初始化DW1000
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE); // 开启功率放大器
    dwt_setfinegraintxseq(0);
    if (result != DWT_SUCCESS) return (-1);
    port_set_dw1000_fastrate(); // 高速收发

    MODES = decarangingmode();
    config.chan = chConfig[MODES].channel;
    config.dataRate = chConfig[MODES].datarate;
    config.nsSFD = chConfig[MODES].nsSFD;
    config.phrMode = DWT_PHRMODE_STD;
    config.prf = chConfig[MODES].prf;
    config.rxPAC = chConfig[MODES].pacSize;
    config.sfdTO = chConfig[MODES].sfdTO;
    config.rxCode = chConfig[MODES].preambleCode;
    config.txCode = chConfig[MODES].preambleCode;
    config.txPreambLength = chConfig[MODES].preambleLength;

    dwt_configure(&config); // 配置DW1000

    dwt_configuresleep(DWT_LOADUCODE | DWT_PRESRV_SLEEP | DWT_CONFIG | DWT_TANDV,
                       DWT_WAKE_WK | DWT_WAKE_CS | DWT_SLP_EN); // 配置睡眠
    dwt_setleds(3);                                             // 设置dw1000收发指示LED
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE |
                         DWT_INT_RFSL | DWT_INT_SFDT,
                     1); // 设置DW1000的中断

    // power = txSpectrumConfigr[config.chan].txPwr[rate_ch][config.prf-DWT_PRF_16M];
    power = sys_para.gain1 + (sys_para.gain2 << 16);

    txconfig.power = power;                                  //
    txconfig.PGdly = txSpectrumConfigr[config.chan].PGdelay; //
    dwt_configuretxrf(&txconfig);                            //

    dwt_setrxantennadelay(RX_ANT_DLY); // 设置天线接收延迟
    dwt_settxantennadelay(TX_ANT_DLY); // 设置天线发送延迟

    addressconfigure(MODULE_ROLE);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    dwt_setcallbacks(twr_tx_tag_cb, twr_rx_cb, twr_rx_timeout_cb, twr_rx_error_cb); // 注册回调函数
    return (deID);
}
