#include <stdio.h>

#include "dwt_common.h"
#include "instance.h"
#include "main.h"
#include "tim.h"

#define MAXTAGNUM  128 // 定义最大标签数量
#define MINTAGSLOT 10  // 定义标签间隔时间
uint8_t mode = 0;      // 标签基站模式
uint8_t sendbuff[300]; // 发送buff
uint8_t index2 = 0;    // 标签收到基站response的时间戳填充位置
uint8_t Tarecrx = 0;   // 标签模式
uint8_t faild_rxtime = 0;
uint16_t addr8_t = 0;      // 基站短地址
uint16_t addr16_t = 0;     // 基站长地址
uint16_t Anch_addr = 1000; // 基站地址
int MAX_TAG_SIZE = 64;     // 标签个数
int N_Slot = 64;           // 时间片个数
int T_Slot = 20;           // 1个时间片的时间 ms
int T_Round = 64 * 20;     // 轮询的总时间
int GetT_ID = 0;           // 标签ID
int rangnum = 0;           // 测距次数不断加
int POLL_Value = 0;        // 预留电池
int aaddr, taddr;          // 基站标签地址
int contrlsendms = 10;     // 控制发送时间
int contrlbit = 0;         // 控制发送位
int POLL_count = 0;        // poll次数++
int response_num = 0;      // 标签接收到response包数量
int lastwaittime = 0;      // 标签应等待时间
int twl_address_index = 0;

/***************************************************************************************************/

// Debug日志 | ? - 有疑问，待解决 | ! - 正在解决 | * - 已解决

//* @Debug: Anchor无法进入回调的问题已经找出：在接收状态中状态机切换过度频繁，在触发中断加入接收回调之后回到
//*         原断点处，程序继续运行会导致状态机无法切换到正确的状态导致状态机死锁。Bug已修复。

//* @Debug: Anchor成功进入接收回调，也接收到了来自Tag的信息，也能正确解析POLL包信息，但是并没有进入CAND发送
//*         的状态；检查到能够进入等待时隙的休眠状态，但是休眠无法结束；发现Anchor模式下的时钟外部中断并没有
//*         编写休眠时间自减到0判断休眠结束的条件，导致Anchor的休眠时间和定时器溢出的时间一致。Bug已修复。

//* @Debug: Anchor的流程从接收POLL到发送CAND没有问题；单Tag对单Anchor的情况下，Tag从POLL广播到接收单个CAND
//*         后，计算SS-TWR的结果是错误的，需要检查是解析时间戳错误还是计算错误；是计算出的距离大于了2km，超出
//*         了测距的最大范围，需要检查SS-TWR计算出的ToF是否正确，以及POLL和CAND数据包中的时间戳数据是否正确；
//*         发现从数据包中读取时间戳的函数msg_get_ts和设置数据包时间戳的函数msg_set_ts的函数实现存在冲突：读
//*         取函数采用大端序，设置函数采用小端序，导致从数据包中解析出的时间戳是错误的。Anchor端计算发送时间的
//*         时候，天线发送延迟TX_ANT_DLY参与计算的形式有问题，Bug已被修复。
//?         奇怪了，我记得我没改过这两个函数，可能是在用Agent检查的时候被改了。

//? @Debug: Tag使用SS-TWR计算出的距离误差偏大，需要看看是SS-TWR的问题，还是计算错误；测距值的误差很大，但是
//?         变化值的误差却是合理的，这可能是什么导致的？会不会是计算过程存在问题？尝试计算DS-TWR看看测距误差
//?         如何；暂时将第一个SS-TWR的计算放弃，收到来自4个不同Anchor的CAND之后，向四个Anchor发送GRANT并等
//?         RESPONSE。

//! @Debug: 修改代码，将根据SS-TWR的范围测距逻辑删除，只要Tag接收到了CAND，它都会向CAND来源发送GRANT信息；
//!         Tag可以正常发出GRANT，Anchor也可以接收到GRANT，但是GRANT无法进入解析，查看是不是校验错误；wc
//!         是在任务循环中漏写了GRANT解析的状态机=_=，添加状态机查看情况；状态机添加，能够进入状态机，在回
//!         调中接收到了数据并解析到其功能码为GRANT，但是在解析处理函数中，功能码变为了0，检查接收到数据之
//!         后是否存在在切换过程中更改了功能码的情况；

/***************************************************************************************************/

// 基站的部署间距，和基站的响应范围，单位m
#define DISTRIBUTION_DIST   5                         // m
#define RESPONSE_RANGE      DISTRIBUTION_DIST * 1.414 // m
#define TIMESTAMP_LEN       5                         // 添加字段的长度，两个字节数为5的时间戳
#define MAX_RESP_ANCHOR_NUM 9                         // 最大响应的Anchor数量
#define DW_MASK_40          ((1ULL << 40) - 1ULL)     // 针对DW1000的40B时间戳的回绕处理掩码

#define POLL                0x81 // POLL功能码
#define CAND                0x71 // CAND功能码
#define GRANT               0x83 // GRANT功能码
#define RESPONSE            0x70 // RESPONSE功能码

#define POLL_TX_TS          2  // POLL信息中poll_tx_ts的偏移量
#define POLL_RX_TS          2  // CAND信息中poll_rx_ts的偏移量
#define CAND_TX_TS          6  // CAND信息中cand_tx_ts的偏移量
#define CAND_RX_TS          2  // GRANT信息中cand_rx_ts的偏移量
#define GRANT_TX_TS         6  // GRANT信息中grant_tx_ts的偏移量
#define GRANT_RX_TS         8  // RESPONSE信息中grant_rx_ts的偏移量
#define RESP_TX_TS          12 // RESPONSE信息中resp_tx_ts的偏移量
#define RANGE_NUM           1  // RangeNum的偏移量
#define RESP_SLEEP_CORR_L   2  // RESPONSE信息中sleep_correction的偏移量
#define RESP_SLEEP_CORR_H   3  // RESPONSE信息中sleep_correction的偏移量
#define RESP_TOF            4  // RESPONSE信息中ToF的偏移量

#define POLL_MSG_LEN        6  // Tag发送的POLL帧的长度-FCODE(1)+RANGE_NUM(1)+POLL_TX_TS(4)
#define CAND_MSG_LEN        10 // Anchor发送的CAND帧的长度-FCODE(1)+RANGE_NUM(1)+POLL_RX_TS(4)+CAND_TX_TS(4)
#define GRANT_MSG_LEN       10 // Tag发送的GRANT帧的长度-FCODE(1)+RANGE_NUM(1)+GRANT_TX_TS(4)+CAND_RX_TS(4)
// Anchor发送的RESPONSE帧的长度-FCODE(1)+RANGE_NUM(1)+SLEEP_CORRECTION(2)+TOF(4)+GRANT_RX_TS(4)+RESP_TX_TS(4)
#define RESPONSE_MSG_LEN    16
#define CRC_LEN             1 // CRC校验位长度

uint8_t anchor_group_id = 0x01; // Anchor标记组
uint8_t tag_group_id = 0x00;    // Tag标记组

/// 通过记录以下时间戳，Tag和Anchor都可以使用DS-TWR计算距离，单次轮询可以计算3次ToF(一次SS-TWR，两次DS-TWR)
/// Tag: ToF_ss = 1/2 x [(cand_rx_ts - poll_tx_ts) - (cand_tx_ts - poll_rx_ts)]
/// Anchor: ToF_ds1 = 1/4 x [(grant_rx_ts - cand_tx_ts) - (grant_tx_ts -
/// cand_rx_ts) + (cand_rx_ts - poll_tx_ts) - (cand_tx_ts - poll_rx_ts)] Tag:
/// ToF_ds2 = 1/4 x [(resp_rx_ts - grant_tx_ts) - (resp_tx_ts - grant_rx_ts) +
/// (grant_rx_ts - cand_tx_ts) - (grant_tx_ts - cand_rx_ts)]
/// Anchor在RESPONSE中装载ToF_ds1，Tag端就可以获得3次完整的测距信息，可以试着融合三次测距信息共12维数据，计算出更加精确的距离

static uint64_t poll_tx_ts;  // 发送poll时间
static uint64_t poll_rx_ts;  // 接收poll时间
static uint64_t cand_tx_ts;  // 发送cand时间
static uint64_t cand_rx_ts;  // 接收cand时间
static uint64_t grant_tx_ts; // 发送grant时间
static uint64_t grant_rx_ts; // 接收grant时间
static uint64_t resp_tx_ts;  // 发送response时间
static uint64_t resp_rx_ts;  // 接收response时间

// 32位时间戳，用于计算
static uint32_t poll_tx_ts32;  // 发送poll时间32位
static uint32_t poll_rx_ts32;  // 接收poll时间32位
static uint32_t cand_tx_ts32;  // 发送cand时间32位
static uint32_t cand_rx_ts32;  // 接收cand时间32位
static uint32_t grant_tx_ts32; // 发送grant时间32位
static uint32_t grant_rx_ts32; // 接收grant时间32位
static uint32_t resp_tx_ts32;  // 发送response时间32位
static uint32_t resp_rx_ts32;  // 接收response时间32位

// 记录CAND的接收时间戳
uint64_t cand_rx_ts_arr[MAX_ANCHOR_LIST_SIZE] = {0};
// 记录距离有效的CAND的源地址
uint16_t cand_anchor_addr[MAX_ANCHOR_LIST_SIZE] = {0};
// 有效的CAND源地址数量，在RX_CAND状态下增加，在TX_GRANT状态下减少
int8_t cand_valid_num = 0;

// SS-TWR得到的ToF的容器
uint64_t ToF_ss[MAX_ANCHOR_LIST_SIZE] = {0};
// Anchor的DS-TWR得到的ToF的容器
uint32_t ToF_ds1[MAX_ANCHOR_LIST_SIZE] = {0};
// Tag的DS-TWR得到的ToF的容器
uint64_t ToF_ds2[MAX_ANCHOR_LIST_SIZE] = {0};

// debug
double dist_ss = 0;
uint8_t RET_T = 0;
uint8_t RET_A = 0;
int8_t miss_flag = 0;
int8_t state_flag = 0;
int8_t cb_flag = 0;
uint64_t debug_ts = 0;

static uint64_t get_sys_timestamp_u64(void);
static void TA_Init_Handler(int module_mode);

static void TA_Txe_Handler(int module_mode);
static void TA_Rxe_Handler(int module_mode);
static void TA_Tx_Confirm_Handler(int module_mode);

static void TA_Rx_Poll_Handler(void);
static void TA_Tx_Poll_Handler(void);
static void TA_Rx_Cand_Handler(void);
static void TA_Tx_Cand_Handler(void);
static void TA_Rx_Grant_Handler(void);
static void TA_Tx_Grant_Handler(void);
static void TA_Rx_Response_Handler(void);
static void TA_Tx_Response_Handler(void);
static void TA_Feedback_Handler(void);

/**
 * @brief 从ToF计算出距离，单位为毫米
 * @param tof 飞行时间，单位为DW1000时间单位
 * @return 距离，单位为毫米。如果无效返回-1
 */
double instance_calc_distance(uint64_t tof) {
    double dis;
    double tofx = 0;

    // 转换为秒
    tofx = tof * DWT_TIME_UNITS;

    // 计算距离：时间(秒) × 光速(米/秒) × 1000 = 距离(毫米)
    dis = tofx * ((float)SPEED_OF_LIGHT) * 1000;

    // 有效性检查（以毫米为单位）
    if ((dis > 2000000.0) || (dis <= 0)) { // 2000000毫米 = 2000米
        if (dis > 2000000.0)
            miss_flag = 1;
        else if (dis <= 0)
            miss_flag = 2;
        dis = -1;
    } else if (dis > RESPONSE_RANGE * 1000) { // 转换为毫米
        miss_flag = 3;
        dis = -1;
    } else {
        miss_flag = 4;
    }

    return dis;
}

uint64_t dw_ts_diff(uint64_t ts1, uint64_t ts2) { return (ts1 - ts2) & DW_MASK_40; }

/***************************************************************************************************/

double distance[MAX_ANCHOR_LIST_SIZE] = {0};       // 存放距离
static uint64_t resp_rx_tag[MAX_ANCHOR_LIST_SIZE]; // 发送response 时间
// uint32 Response_tx_time=0;

event_data_t event;
event_data_t dw_event;
instance_data_t ins;
extern int Get_tag_inf;

static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);
static void msg_set_ts(uint8 *ts_field, uint64 ts);
static void msg_get_ts(const uint8 *ts_field, uint32_t *ts);

void insaddress(uint16_t address) // 获取标签ID，基站地址，设置包头帧
{
    // ins.instanceAddress16=address|0x8000;

    ins.instanceAddress16 = address | (anchor_group_id << 8);

    ins.panID = 0xdeca;
    // set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
    ins.msg_f.frameCtrl[0] = 0x1 | 0x40;

    // source/dest addressing modes and frame version
    ins.msg_f.frameCtrl[1] = 0x8 | 0x80;
    ins.msg_f.panID[0] = (ins.panID) & 0xff;
    ins.msg_f.panID[1] = ins.panID >> 8;
    ins.longTermRangeCount = 0;
    ins.msg_f.seqNum = 0;
    T_ID = sys_para.uwbid;
}

void testappruns(int modes) {
    int ret_T = 0;
    int ret_A = 0;
    int u = 0;
    switch (ins.nextState) {
        case TA_INIT: {
            // 初始化
            TA_Init_Handler(modes);
        } break;
        case TA_TXE_WAIT: {
            // 中间态，由TA_INIT/TA_RX_WAIT_DATA切换到发送数据的状态
            // 更新发送对应数据的标志位
            TA_Txe_Handler(modes);
        } break;
        case TA_TX_WAIT_CONF: {
            // 中间态，发送完数据之后的确认，切换到TA_RXE_WAIT或者TA_INIT
            // 发送结束后，更新接收特定数据的标志位
            TA_Tx_Confirm_Handler(modes);
        } break;
        case TA_RXE_WAIT: {
            // 中间态，由TA_INIT/TA_TX_WAIT_CONF切换到接收特定数据的状态
            // 开启收发器，设置接收超时
            TA_Rxe_Handler(modes);
        } break;
        case TA_RX_WAIT_POLL: {
            // Anchor 等待POLL状态
            TA_Rx_Poll_Handler();
        } break;
        case TA_TXPOLL_WAIT_SEND: {
            // Tag 发送POLL状态
            TA_Tx_Poll_Handler();
        } break;
        case TA_TXCAND_WAIT_SEND: {
            // Anchor 发送CAND状态
            TA_Tx_Cand_Handler();
        } break;
        case TA_RX_WAIT_CAND: {
            // Tag 等待CAND状态
            TA_Rx_Cand_Handler();
        } break;
        case TA_TXGRANT_WAIT_SEND: {
            // Tag 发送GRANT状态
            state_flag++;
            TA_Tx_Grant_Handler();
        } break;
        case TA_RX_WAIT_RESPONSE: {
            // Tag 等待RESPONSE状态
            TA_Rx_Response_Handler();
        } break;
        case TA_RX_WAIT_GRANT: {
            // Anchor 等待GRANT状态
            TA_Rx_Grant_Handler();
        } break;
        case TA_TXRESPONSE_WAIT_SEND: {
            // Anchor 发送RESPONSE状态
            TA_Tx_Response_Handler();
        } break;
        case TA_FEEDBACK_DATA: {
            // Tag反馈数据
            TA_Feedback_Handler();
        } break;
        case TA_SLEEP: {
            // 等待时隙，具体处理方法为等待已被设置的waittime在时钟中断中自减为0
            ins.testAppState = TA_SLEEP;
            ins.previousState = ins.testAppState;
        } break;
        case TA_SLEEP_DONE: {
            // 休眠结束，准备发送
            ins.testAppState = TA_SLEEP_DONE;
            ins.previousState = ins.testAppState;
            ins.nextState = TA_TXE_WAIT;
        } break;
        default:
            break;
    }
}

static void TA_Init_Handler(int module_mode) {
    switch (module_mode) {
        case ANCHOR: {
            ins.testAppState = TA_INIT;
            mode = ANCHOR;

            aaddr = (ins.instanceAddress16) & 0xFF; // 基站地址
            memcpy(ins.eui64, &ins.instanceAddress16, ADDR_BYTE_SIZE_S);
            dwt_seteui(ins.eui64);
            dwt_setpanid(ins.panID);

            ins.previousState = TA_INIT;
            ins.nextState = TA_RXE_WAIT; // 状态机切换: ANCHOR: INIT->RXE_WAIT
            ins.wait4ack = DWT_START_RX_IMMEDIATE;

            Anch_RecNt = 1;
            break;
        }

        case TAG: {
            ins.testAppState = TA_INIT;
            mode = TAG;

            taddr = sys_para.uwbid;
            memcpy(ins.eui64, &ins.instanceAddress16, ADDR_BYTE_SIZE_S);
            dwt_seteui(ins.eui64);
            dwt_setpanid(ins.panID);

            // 根据上一次收到的最近的Anchor的RESPONSE来计算自己的下一个时隙在什么时候
            if (waittime > 1) {
                ins.nextState = TA_SLEEP;
            } else if (waittime == 0) {
                if ((Tarecrx == 1) && (response_num == 0)) {
                    if (lastwaittime != 0 && faild_rxtime < 10) {
                        waittime = lastwaittime;
                        faild_rxtime++;
                    } else if (faild_rxtime >= 10 || lastwaittime == 0) {
                        lastwaittime = waittime = 1000;
                        faild_rxtime = 0;
                    }

                    // 上一次轮询没有收到回复，1000ms发送一次
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_SLEEP;
                } else if (Tarecrx == 2) {
                    // 上一次轮询有收到回复，进入下一次轮询
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_TXE_WAIT;
                } else if (Tarecrx == 0) {
                    waittime = T_ID * 200;
                    // 开机第一次发送等待时间
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_SLEEP;
                }
            }
            break;
        }
    }
}

/**
 * @brief 中间态处理函数，由TA_INIT/TA_RX_WAIT_DATA切换到发送特定数据的状态
 * @note  中间态不改变状态机的previousState
 * @param mode
 */
static void TA_Txe_Handler(int module_mode) {
    // Tag和Anchor切换到发送状态的中间态
    if (module_mode == TAG) {
        ins.testAppState = TA_TXE_WAIT;
        if (ins.previousState == TA_INIT || ins.previousState == TA_SLEEP_DONE) {
            // Tag 初始化结束(或者Tag初始化完毕后休眠结束)，准备发送POLL
            ins.nextState = TA_TXPOLL_WAIT_SEND;
        } else if (ins.previousState == TA_RX_WAIT_RESPONSE || ins.previousState == TA_RX_WAIT_CAND) {
            // 接收到4个CAND，或者发送GRANT之后接收到了RESPONSE，发送下一个GRANT
            ins.nextState = TA_TXGRANT_WAIT_SEND;
        }
    } else if (module_mode == ANCHOR) {
        ins.testAppState = TA_TXE_WAIT;
        if (ins.previousState == TA_SLEEP_DONE) {
            // 休眠结束，准备发送CAND
            ins.nextState = TA_TXCAND_WAIT_SEND;
        } else if (ins.previousState == TA_RX_WAIT_GRANT) {
            // 接收到GRANT，发送RESPONSE
            ins.nextState = TA_TXRESPONSE_WAIT_SEND;
        }
    }
}

/**
 * @brief 中间态，由发送特定数据结束后切换到TA_RXE_WAIT或者TA_INIT
 * @note  中间态不改变状态机的previousState
 * @param mode
 */
static void TA_Tx_Confirm_Handler(int module_mode) {
    if (module_mode == TAG) {
        // Tag 确认发送数据，切换到接收
        ins.testAppState = TA_TX_WAIT_CONF;
        ins.nextState = TA_RXE_WAIT;
    } else if (module_mode == ANCHOR) {
        ins.testAppState = TA_TX_WAIT_CONF;
        if (ins.previousState == TA_TXRESPONSE_WAIT_SEND) {
            // Anchor 确认发送RESPONSE，本轮结束，重新初始化
            ins.nextState = TA_INIT;
        } else if (ins.previousState == TA_TXCAND_WAIT_SEND) {
            // Anchor 确认发送CAND，等待GRANT
            ins.nextState = TA_RXE_WAIT;
        }
    }
}

/**
 * @brief 中间态，由TA_INIT或者发送特定数据之后切换到接收数据的状态，同时设置接收超时时间
 * @note  中间态不改变状态机的previousState
 * @param mode
 */
static void TA_Rxe_Handler(int module_mode) {
    if (module_mode == TAG) {
        ins.testAppState = TA_RXE_WAIT;
        if (ins.previousState == TA_TXPOLL_WAIT_SEND) {
            ins.nextState = TA_RX_WAIT_CAND; // 等待CAND信息
        } else if (ins.previousState == TA_TXGRANT_WAIT_SEND) {
            ins.nextState = TA_RX_WAIT_RESPONSE; // 等待RESPONSE消息
        }
    } else if (module_mode == ANCHOR) {
        ins.testAppState = TA_RXE_WAIT;
        if (ins.previousState == TA_TXCAND_WAIT_SEND) {
            ins.nextState = TA_RX_WAIT_GRANT;
        } else if (ins.previousState == TA_INIT) {
            // Anchor在初始化结束后立即开启接收
            if ((2 * T_Slot * 1100) > 65535) {
                dwt_setrxtimeout(65535);
            } else {
                dwt_setrxtimeout(2 * T_Slot * 1100);
            }
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            ins.nextState = TA_RX_WAIT_POLL;
        }
    }
}

static void TA_Rx_Poll_Handler(void) {
    // 状态机更新
    ins.testAppState = TA_RX_WAIT_POLL;
    // 检查是否更新接收信息
    if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0) {
        // 未接收到数据
        ins.previousState = ins.testAppState;
    } else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == POLL) {
        Anch_RecNt = 0;

        poll_rx_ts = get_rx_timestamp_u64(); // 获取POLL接收时间

        // 计算CRC校验
        uint8_t crc = 0;
        int r = 0;
        int crc_len = POLL_MSG_LEN;
        for (r = 0; r < crc_len; r++) crc += dw_event.msgu.rxmsg_ss.messageData[r];
        crc = (~crc) + 1;

        if (crc == dw_event.msgu.rxmsg_ss.messageData[POLL_MSG_LEN]) {
            // @Pazfic
            // 获取poll发送时间戳
            msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[POLL_TX_TS], &poll_tx_ts32);
            // debug
            twl_address_index = 0;
            taddr = GetT_ID =
                dw_event.msgu.rxmsg_ss.sourceAddr[0] + (dw_event.msgu.rxmsg_ss.sourceAddr[1] << 8); // 获取标签地址

            // 获取标签的ID(从1开始)
            if (sys_para.twlt == 0x0000) {
                if ((0 < GetT_ID) && (GetT_ID <= N_Slot))
                    twl_address_index = GetT_ID;
                else
                    twl_address_index = 0;
            }

            // 标签ID合法
            if (twl_address_index > 0) {
                // @Pazfic: 接收到POLL之后，发送CAND帧
                rangnum = ins.rangeNum = dw_event.msgu.rxmsg_ss.messageData[POLL_RNUM]; // 获取测距次数
                ins.msg_f.seqNum = dw_event.msgu.rxmsg_ss.seqNum++;                     // 增加序列号
                memcpy(&(ins.msg_f.destAddr[0]), &(dw_event.msgu.rxmsg_ss.sourceAddr[0]),
                       ADDR_BYTE_SIZE_S); // 装载目标地址
                ins.msg_f.sourceAddr[0] = ins.eui64[0];
                ins.msg_f.sourceAddr[1] = ins.eui64[1]; // 装载源地址
                msg_set_ts(&ins.msg_f.messageData[POLL_RX_TS],
                           poll_rx_ts);                     // 装载poll接收时间
                ins.msg_f.messageData[POLL_RNUM] = rangnum; // 装载测距次数
                ins.msg_f.messageData[FCODE] = CAND;        // 装载功能码

                uint8_t slot_idx = ins.instanceAddress16 % MAX_RESP_ANCHOR_NUM;
                waittime = slot_idx * 10 + 3; // 设置休眠时间，等待时隙
                contrlbit = 1;

                ins.previousState = ins.testAppState;
                ins.nextState = TA_SLEEP;
            } else if (twl_address_index == 0) {
                // 时隙不合理，回到接收
                ins.previousState = ins.testAppState;
            }
        } else {
            // 校验失败，回到接收
            ins.previousState = ins.testAppState;
        }

        dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
    } else {
        // 不为POLL，回到接收
        Anch_RecNt = 0; // 设置接收标识
        ins.previousState = ins.testAppState;

        dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
    }
    Anch_RecNt = 1;
}

static void TA_Tx_Poll_Handler(void) {
    // 配置poll发送并开启发送

    uint16_t frameLength = 0;
    uint8_t ret_T;
    frameLength = FRAME_CRTL_AND_ADDRESS_S + POLL_MSG_LEN + CRC_LEN + FRAME_CRC; // 设置poll包长度

    ins.testAppState = TA_TXPOLL_WAIT_SEND;
    ins.previousState = TA_TXE_WAIT;
    ins.msg_f.seqNum = ins.frameSN++;
    ins.msg_f.sourceAddr[0] = sys_para.uwbid & 0xff;
    ins.msg_f.sourceAddr[1] = (sys_para.uwbid >> 8) & 0xff; // 装载源地址
    ins.msg_f.destAddr[0] = 0xff;
    ins.msg_f.destAddr[1] = 0xff; // 设置广播
    ins.msg_f.messageData[POLL_RNUM] = ins.rangeNum++;
    ins.msg_f.messageData[FCODE] = POLL;
    // 等待响应数量为4
    ins.responseTO = 4;

    // @Pazfic: 计算发送时间戳
    uint32_t tx_ts = 0;
    uint64_t precise_tx_ts = get_sys_timestamp_u64();
    tx_ts = (precise_tx_ts + (1000 * UUS_TO_DWT_TIME)) >> 8;                // 延迟100us发送
    dwt_setdelayedtrxtime(tx_ts);                                           // 设置延迟发送时间戳
    precise_tx_ts = (((uint64_t)(tx_ts & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY; // 计算精确发送时间戳
    poll_tx_ts = precise_tx_ts;

    // 装载拓展帧的时间戳
    msg_set_ts(&ins.msg_f.messageData[POLL_TX_TS], poll_tx_ts);

    // 计算CRC
    ins.msg_f.messageData[POLL_MSG_LEN] = 0;
    int crc_len = POLL_MSG_LEN;
    int r = 0;
    for (r = 0; r < crc_len; r++) ins.msg_f.messageData[POLL_MSG_LEN] += ins.msg_f.messageData[r];
    ins.msg_f.messageData[POLL_MSG_LEN] = (~ins.msg_f.messageData[POLL_MSG_LEN]) + 1; // 设置crc位

    dwt_writetxdata(frameLength, (uint8_t *)&(ins.msg_f), 0); // 填充poll数据
    dwt_writetxfctrl(frameLength, 0, 1);                      // 控制发送

    dwt_setrxtimeout(ins.responseTO * 2200);
    dwt_setrxaftertxdelay(20);                                         // 在发送后20us开启接收
    ret_T = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED); // 延迟发送并在发送完成后开启接收
    RET_T = ret_T;

    if (ret_T == DWT_SUCCESS) {
        Tarecrx = 1;
        POLL_count++;
        response_num = 0;
        cand_valid_num = 0;

        // 清空CAND源地址容器
        memset(cand_anchor_addr, 0, sizeof(cand_anchor_addr));
        // 清空CAND接收时间戳容器
        memset(cand_rx_ts_arr, 0, sizeof(cand_rx_ts_arr));

        ins.previousState = ins.testAppState;
        ins.nextState = TA_TX_WAIT_CONF;
    } else {
        // 发送失败，关闭收发器，重新初始化
        dwt_forcetrxoff();
        ins.previousState = ins.testAppState;
        ins.nextState = TA_INIT;
    }
}

static void TA_Rx_Cand_Handler(void) {
    // 等待CAND
    ins.testAppState = TA_RX_WAIT_CAND;
    if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0) {
        ins.previousState = ins.testAppState;
    } else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == CAND) {
        Tag_RecNt = 0;
        // 计算CRC校验
        uint8_t crc = 0;
        uint8_t Anch_num = 0;
        int r = 0;
        int crc_len = CAND_MSG_LEN;
        for (r = 0; r < crc_len; r++) crc += dw_event.msgu.rxmsg_ss.messageData[r];
        crc = (~crc) + 1;

        Tagrxtime = 0;
        // 校验CRC
        if (dw_event.msgu.rxmsg_ss.messageData[CAND_MSG_LEN] == crc) {
            // 查看目标地址是否对应本设备
            if (T_ID == (dw_event.msgu.rxmsg_ss.destAddr[0] + (dw_event.msgu.rxmsg_ss.destAddr[1] << 8))) {
                // 记录接收时间
                cand_rx_ts = get_rx_timestamp_u64();
                // 获取CAND发送时间和POLL接收时间，记录在32位变量中
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[CAND_TX_TS], &cand_tx_ts32);
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[POLL_RX_TS], &poll_rx_ts32);

                //! @Debug：把SS-TWR测距的流程暂时删去
                // double rrt = (double)(cand_rx_ts32 - poll_tx_ts32);
                // double rpt = (double)(cand_tx_ts32 - poll_rx_ts32);

                // tof_ticks = (uint64_t)((rrt - rpt) / 2);

                // double dist = instance_calc_distance(tof_ticks); // 计算距离
                // if (dist > 0)                                    // 距离有效，记录源地址和距离
                // {
                //     // 记录地址、距离、和接收时间戳
                //     cand_rx_ts_arr[cand_valid_num] = cand_rx_ts;
                //     cand_anchor_addr[cand_valid_num] =
                //         dw_event.msgu.rxmsg_ss.sourceAddr[0] + (dw_event.msgu.rxmsg_ss.sourceAddr[1] << 8);
                //     ToF_ss[cand_valid_num] = tof_ticks;

                //     if (cand_valid_num < MAX_ANCHOR_LIST_SIZE) {
                //         // 未接收到满足定位需求的ToFs，继续接收
                //         cand_valid_num++; // 增加cand_valid_num
                //         ins.previousState = ins.testAppState;
                //         ins.nextState = TA_RX_WAIT_CAND;
                //     } else if (cand_valid_num == MAX_ANCHOR_LIST_SIZE) {
                //         // 接收到满足定位需求的ToFs，先通过串口反馈一次数据，再切换为发送GRANT
                //         dwt_forcetrxoff(); // 关闭收发器
                //         ins.previousState = ins.testAppState;
                //         ins.nextState = TA_FEEDBACK_DATA;
                //     }
                // } else {
                //     // 距离无效，放弃这段数据，继续接收
                //     ins.previousState = ins.testAppState;
                // }

                // 记录地址、距离、和接收时间戳
                cand_rx_ts_arr[cand_valid_num] = cand_rx_ts;
                cand_anchor_addr[cand_valid_num] =
                    dw_event.msgu.rxmsg_ss.sourceAddr[0] + (dw_event.msgu.rxmsg_ss.sourceAddr[1] << 8);
                cand_valid_num++;
                // 转换时间戳，记录到32位数据中
                poll_tx_ts32 = (uint32_t)poll_tx_ts;
                cand_rx_ts32 = (uint32_t)cand_rx_ts;

                if (cand_valid_num < MAX_ANCHOR_LIST_SIZE) {
                    // 接收到的CAND数量达不到4个，设置下一次接收接收超时时间。
                    dwt_setrxtimeout(2200 * (ins.responseTO - cand_valid_num));
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);

                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_RX_WAIT_CAND;
                } else {
                    // 接收到CAND数量达到4个，关闭收发器，准备进入发送CAND阶段
                    dwt_forcetrxoff();
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_TXE_WAIT;
                }

            } else {
                // 非本设备地址，放弃数据，继续接收
                ins.previousState = ins.testAppState;
            }
        } else {
            // 校验错误，放弃数据，继续接收
            ins.previousState = ins.testAppState;
        }
        dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
    } else {
        // 数据不为CAND，放弃数据，继续接收
        Tag_RecNt = 0;
        dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
        ins.previousState = ins.testAppState;
    }
    Tag_RecNt = 1;
}

static void TA_Tx_Cand_Handler(void) {
    ins.testAppState = TA_TXCAND_WAIT_SEND;

    uint16_t frameLength = 0;
    uint8_t ret_A = 0;
    int calcwaittime = 0;

    frameLength = FRAME_CRTL_AND_ADDRESS_S + CAND_MSG_LEN + CRC_LEN + FRAME_CRC; // 设置CAND包长度

    // 设置延迟发送
    uint32_t tx_ts = 0;
    uint64_t precise_tx_ts = get_sys_timestamp_u64();
    tx_ts = (precise_tx_ts + (1000 * UUS_TO_DWT_TIME)) >> 8;                // 延迟100us发送
    dwt_setdelayedtrxtime(tx_ts);                                           // 设置延迟发送时间戳
    precise_tx_ts = (((uint64_t)(tx_ts & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY; // 计算精确发送时间戳
    cand_tx_ts = precise_tx_ts;

    msg_set_ts(&ins.msg_f.messageData[CAND_TX_TS], cand_tx_ts); // 装载cand发送时间戳

    // 计算CRC
    ins.msg_f.messageData[CAND_MSG_LEN] = 0;
    int crc_len = CAND_MSG_LEN;
    for (int r = 0; r < crc_len; r++) ins.msg_f.messageData[CAND_MSG_LEN] += ins.msg_f.messageData[r];
    ins.msg_f.messageData[CAND_MSG_LEN] = (~ins.msg_f.messageData[CAND_MSG_LEN]) + 1; // 设置crc位

    dwt_writetxdata(frameLength, (uint8_t *)&(ins.msg_f), 0); // 填充CAND数据
    dwt_writetxfctrl(frameLength, 0, 1);                      // 控制发送

    dwt_setrxtimeout(8800);                                            // 发送CAND之后等待8800us
    dwt_setrxaftertxdelay(20);                                         // 发送后延迟20us开启接收
    ret_A = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED); // 延迟发送，发送完成开启接收
    RET_A = ret_A;

    // 判断是否发送成功，成功则进入发送确认，失败则回到初始化
    if (ret_A == DWT_SUCCESS) {
        ins.previousState = ins.testAppState;
        ins.nextState = TA_TX_WAIT_CONF;
    } else {
        dwt_forcetrxoff();
        ins.previousState = ins.testAppState;
        ins.nextState = TA_INIT;
    }
}

static void TA_Rx_Grant_Handler(void) {
    ins.testAppState = TA_RX_WAIT_GRANT;

    if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0) {
        ins.previousState = ins.testAppState;
    } else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == GRANT) {
        Anch_RecNt = 0;
        // 校验CRC
        uint8_t crc = 0;
        int r = 0;
        int crc_len = GRANT_MSG_LEN;
        for (r = 0; r < crc_len; r++) crc += dw_event.msgu.rxmsg_ss.messageData[r];
        crc = (~crc) + 1;

        if (dw_event.msgu.rxmsg_ss.messageData[GRANT_MSG_LEN] == crc) {
            // 检查目标地址是否与本设备一致
            if (ins.instanceAddress16 ==
                (dw_event.msgu.rxmsg_ss.destAddr[0] + (dw_event.msgu.rxmsg_ss.destAddr[1] << 8))) {
                uint64_t tof_ticks = 0;
                // 记录接收时间
                grant_rx_ts = get_rx_timestamp_u64();
                // 获取GRANT发送时间和CAND接收时间
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[GRANT_TX_TS], &grant_tx_ts32);
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[CAND_RX_TS], &cand_rx_ts32);

                // DS-TWR计算距离
                grant_rx_ts32 = (uint32_t)grant_rx_ts;
                cand_tx_ts32 = (uint32_t)cand_tx_ts;

                double rrt1 = (double)(cand_rx_ts32 - poll_tx_ts32);
                double rpt1 = (double)(cand_tx_ts32 - poll_rx_ts32);
                double rrt2 = (double)(grant_rx_ts32 - cand_tx_ts32);
                double rpt2 = (double)(cand_rx_ts32 - grant_tx_ts32);

                tof_ticks =
                    (int64_t)((rrt1 * rrt2 - rpt1 * rpt2) / (rrt1 + rrt2 + rpt1 + rpt2)); // DW官方给出的ToF计算式

                //! @Debug: 距离检查
                dist_ss = instance_calc_distance(tof_ticks); // 计算距离

                // 准备发送RESPONSE，预先装载一定量的数据
                rangnum = ins.rangeNum = dw_event.msgu.rxmsg_ss.messageData[RANGE_NUM]; // 获取测距次数
                ins.msg_f.seqNum = dw_event.msgu.rxmsg_ss.seqNum++;                     // 增加序列号
                memcpy(&(ins.msg_f.destAddr[0]), &(dw_event.msgu.rxmsg_ss.sourceAddr[0]),
                       ADDR_BYTE_SIZE_S); // 装载目标地址
                ins.msg_f.sourceAddr[0] = ins.eui64[0];
                ins.msg_f.sourceAddr[1] = ins.eui64[1];                       // 装载源地址
                msg_set_ts(&ins.msg_f.messageData[GRANT_RX_TS], grant_rx_ts); // 装载grant接收时间
                msg_set_ts(&ins.msg_f.messageData[RESP_TOF], tof_ticks);      // 装载测距距离
                ins.msg_f.messageData[RANGE_NUM] = rangnum;                   // 装载测距次数
                ins.msg_f.messageData[FCODE] = RESPONSE;                      // 装载功能码

                dwt_forcetrxoff(); // 关闭收发器
                ins.previousState = ins.testAppState;
                ins.nextState = TA_TXE_WAIT;
            } else {
                // 目标地址与本设备不一致，放弃数据，继续接收
                ins.previousState = ins.testAppState;
            }
        } else {
            // 校验失败，放弃数据，继续接收
            ins.previousState = ins.testAppState;
        }
        dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
    } else {
        // 不为GRANT数据，放弃数据，继续接收
        Anch_RecNt = 0;
        dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
        ins.previousState = ins.testAppState;
    }

    Anch_RecNt = 1;
}

static void TA_Tx_Grant_Handler(void) {
    ins.testAppState = TA_TXGRANT_WAIT_SEND;

    uint16_t frameLength = 0;
    uint8_t ret_T = 0;
    frameLength = FRAME_CRTL_AND_ADDRESS_S + GRANT_MSG_LEN + CRC_LEN + FRAME_CRC; // 设置Grant包长度

    ins.msg_f.messageData[FCODE] = GRANT;
    ins.msg_f.messageData[RANGE_NUM] = ins.rangeNum;
    ins.msg_f.sourceAddr[0] = sys_para.uwbid & 0xff;
    ins.msg_f.sourceAddr[1] = (sys_para.uwbid >> 8) & 0xff; // 装载源地址

    // 装载目标地址，从cand_anchor_addr中从后向前取出
    ins.msg_f.destAddr[0] = cand_anchor_addr[--cand_valid_num] & 0xff;
    ins.msg_f.destAddr[1] = (cand_anchor_addr[--cand_valid_num] >> 8) & 0xff;

    msg_set_ts(&ins.msg_f.messageData[CAND_RX_TS], cand_rx_ts); // 装载cand接收时间戳

    // 设置延迟发送
    uint32_t tx_ts = 0;
    uint64_t precise_tx_ts = get_sys_timestamp_u64();
    tx_ts = (precise_tx_ts + (1000 * UUS_TO_DWT_TIME)) >> 8;                // 延迟100us发送
    dwt_setdelayedtrxtime(tx_ts);                                           // 设置延迟发送时间戳
    precise_tx_ts = (((uint64_t)(tx_ts & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY; // 计算精确发送时间戳
    grant_tx_ts = precise_tx_ts;

    msg_set_ts(&ins.msg_f.messageData[GRANT_TX_TS], grant_tx_ts); // 装载GRANT发送时间戳
    // 计算CRC
    ins.msg_f.messageData[GRANT_MSG_LEN] = 0;
    int crc_len = GRANT_MSG_LEN;
    for (int r = 0; r < crc_len; r++) ins.msg_f.messageData[GRANT_MSG_LEN] += ins.msg_f.messageData[r];
    ins.msg_f.messageData[GRANT_MSG_LEN] = (~ins.msg_f.messageData[GRANT_MSG_LEN]) + 1; // 设置校验位

    dwt_writetxdata(frameLength, (uint8_t *)&(ins.msg_f), 0); // 填充GRANT数据
    dwt_writetxfctrl(frameLength, 0, 1);                      // 控制发送

    dwt_setrxtimeout(2200);                                            // 设置等待RESPONSE的时间为2200us
    dwt_setrxaftertxdelay(20);                                         // 设置发送完成后20us开启接收
    ret_T = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED); // 延迟发送，并设定发送完成后开启接收
    RET_T = ret_T;

    // 是否发送成功，成功则进行确认，失败则回到初始化阶段
    if (ret_T == DWT_SUCCESS) {
        ins.previousState = ins.testAppState;
        ins.nextState = TA_TX_WAIT_CONF;
    } else {
        dwt_forcetrxoff();
        ins.previousState = ins.testAppState;
        ins.nextState = TA_INIT;
    }
}

static void TA_Rx_Response_Handler(void) {
    ins.testAppState = TA_RX_WAIT_RESPONSE;
    if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0) {
        ins.previousState = ins.testAppState;
    } else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == RESPONSE) {
        Tag_RecNt = 0;
        uint8_t crc_jy = 0;
        uint8_t Anch_num = 0;
        int r = 0;
        int crc_len = RESPONSE_MSG_LEN; // crc校验长度
        for (r = 0; r < crc_len; r++) crc_jy += dw_event.msgu.rxmsg_ss.messageData[r];
        crc_jy = (~crc_jy) + 1; // 计算crc

        Tagrxtime = 0;
        if (dw_event.msgu.rxmsg_ss.messageData[CRC_BIT_RES] == crc_jy) {
            if (T_ID == dw_event.msgu.rxmsg_ss.destAddr[0] + (dw_event.msgu.rxmsg_ss.destAddr[1] << 8)) {
                Tag_RecNt = 0;
                Tarecrx = 2;
                resp_rx_ts = get_rx_timestamp_u64(); // 记录response接收时间
                uint32_t ToF_ds = 0;
                uint64_t tof_ticks = 0;

                // 获取RESPONSE中的两个时间戳
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[GRANT_RX_TS], &grant_rx_ts32);
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[RESP_TX_TS], &resp_tx_ts32);
                // 获取Anchor计算出的DS-TWR数据
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[TOFR], &ToF_ds);
                ToF_ds1[response_num] = ToF_ds;

                // Tag端计算一次DS-TWR
                grant_tx_ts32 = (uint32_t)grant_tx_ts;
                resp_rx_ts32 = (uint32_t)resp_rx_ts;

                double rrt1 = (double)(resp_rx_ts32 - grant_tx_ts32);
                double rpt1 = (double)(resp_tx_ts32 - grant_rx_ts32);
                double rrt2 = (double)(grant_rx_ts32 - cand_tx_ts32);
                double rpt2 = (double)(grant_tx_ts32 - cand_rx_ts32);

                // DW官方给出的DS-TWR计算公式
                tof_ticks = (int64_t)((rrt1 * rrt2 - rpt1 * rpt2) / (rrt1 + rrt2 + rpt1 + rpt2));

                dist_ss = instance_calc_distance(tof_ticks);

                // 保存Tag计算出的DS-TWR
                ToF_ds2[response_num] = tof_ticks;

                // 响应数量增加
                if (response_num < MAX_ANCHOR_LIST_SIZE) response_num++;

                if (response_num == 1) {
                    // 接收到最近的基站，更新休眠时间
                    lastwaittime = waittime = dw_event.msgu.rxmsg_ss.messageData[RESP_SLEEP_CORR_L] +
                                              (dw_event.msgu.rxmsg_ss.messageData[RESP_SLEEP_CORR_H] << 8);
                }

                if ((ins.responseTO - response_num) > 0) {
                    // 没有接收到来自4个基站的response，回到GRANT发送
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_TXE_WAIT;
                } else if (response_num == MAX_ANCHOR_LIST_SIZE) {
                    // 接收到的RESPONSE数量足够完成测距，反馈数据
                    POLL_count = 0;
                    dwt_forcetrxoff();
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_FEEDBACK_DATA;
                }

            } else {
                // 不对应本设备地址，回到接收
                ins.previousState = ins.testAppState;
            }
        } else {
            // CRC校验异常，回到接收
            ins.previousState = ins.testAppState;
        }
        dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
    } else {
        // 消息不为RESPONSE，回到接收
        ins.previousState = ins.testAppState;
        dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
    }

    Tag_RecNt = 1;
}

static void TA_Tx_Response_Handler(void) {
    ins.testAppState = TA_TXPOLL_WAIT_SEND;
    uint16_t frameLength = 0;
    uint8_t ret_A = 0;
    int calcwaittime = 0;

    frameLength = FRAME_CRTL_AND_ADDRESS_S + RESPONSE_MSG_LEN + CRC_LEN + FRAME_CRC; // 设置response包长度

    calcwaittime = ((T_Round + (twl_address_index * T_Slot)) * 10 - Anchpollingtime - 20); // 计算标签下次发送POLL的时间
    if ((calcwaittime > 0) && (calcwaittime < ((T_Round + (twl_address_index * T_Slot)) * 10))) {
        ins.msg_f.messageData[RESP_SLEEP_CORR_L] = calcwaittime & 0xFF;
        ins.msg_f.messageData[RESP_SLEEP_CORR_H] = (calcwaittime >> 8) & 0xFF; // 将休眠时间装载到response包
    } else {
        ins.msg_f.messageData[RESP_SLEEP_CORR_L] = 0;
        ins.msg_f.messageData[RESP_SLEEP_CORR_H] = 0;
    }

    // 配置延迟发送
    uint32_t tx_ts = 0;
    uint64_t precise_tx_ts = get_sys_timestamp_u64();
    tx_ts = (precise_tx_ts + (1000 * UUS_TO_DWT_TIME)) >> 8;                // 延迟100us发送
    dwt_setdelayedtrxtime(tx_ts);                                           // 设置延迟发送时间戳
    precise_tx_ts = (((uint64_t)(tx_ts & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY; // 计算精确发送时间戳
    resp_tx_ts = precise_tx_ts;

    msg_set_ts(&ins.msg_f.messageData[RESP_TX_TS], resp_tx_ts); // 装载response发送时间

    // 计算CRC
    ins.msg_f.messageData[ANCH_RESPONSE_MSG_LEN] = 0;
    int crc_len = ANCH_RESPONSE_MSG_LEN; // 计算CRC长度
    for (int r = 0; r < crc_len; r++) ins.msg_f.messageData[ANCH_RESPONSE_MSG_LEN] += ins.msg_f.messageData[r];
    ins.msg_f.messageData[ANCH_RESPONSE_MSG_LEN] = (~ins.msg_f.messageData[ANCH_RESPONSE_MSG_LEN]) + 1; // 设置crc位

    dwt_writetxdata(frameLength, (uint8_t *)&(ins.msg_f), 0); // 填充response数据
    dwt_writetxfctrl(frameLength, 0, 1);                      // 控制发送
    ret_A = dwt_starttx(DWT_START_TX_DELAYED);                // 延迟发送
    RET_A = ret_A;

    if (ret_A == DWT_SUCCESS) {
        // 发送成功，切换到确认
        ins.previousState = ins.testAppState;
        ins.nextState = TA_TX_WAIT_CONF;
    } else {
        // 发送失败，关闭收发器，重新初始化
        dwt_forcetrxoff();
        ins.previousState = ins.testAppState;
        ins.nextState = TA_INIT;
    }
}

/**
 * @brief 中间态，将数据反馈给上位机
 * @note  中间态不改变设备的previousState
 */
static void TA_Feedback_Handler() {
    ins.testAppState = TA_FEEDBACK_DATA;

    uint8_t u = 0;
    double dist[MAX_ANCHOR_LIST_SIZE] = {0};

    // if (ins.previousState == TA_RX_WAIT_CAND) {
    //     if (cand_valid_num > 0) {
    //         // 接收到4个CAND，拥有4个SS-TWR数据
    //         dist[0] = instance_calc_distance(ToF_ss[0]);
    //         dist[1] = instance_calc_distance(ToF_ss[1]);
    //         dist[2] = instance_calc_distance(ToF_ss[2]);
    //         dist[3] = instance_calc_distance(ToF_ss[3]);

    //         // 装载数据
    //         u = sprintf((uint8_t *)sendbuff[0], "t%x %08x %08x %08x %08x : %04x %04x %04x %04x\r\n", taddr, dist[0],
    //                     dist[1], dist[2], dist[3], cand_anchor_addr[0], cand_anchor_addr[1], cand_anchor_addr[2],
    //                     cand_anchor_addr[3]);
    //         // 通过串口发给上位机
    //         if (u > 0) {
    //             Usart1_DmaTx(sendbuff, u);
    //             Usart3_DmaTx(sendbuff, u);
    //             u = 0;
    //         }
    //         memset(sendbuff, 0, sizeof(sendbuff));
    //     }
    //     ins.nextState = TA_TXE_WAIT; // 切换到发送GRANT
    // }

    if (ins.previousState == TA_RX_WAIT_RESPONSE) {
        if (response_num > 0) {
            // 接收到4个RESPONSE，拥有4个由Anchor计算的DS-TWR数据和4个Tag计算的DS-TWR数据

            // 发送来自Anchor的DS-TWR数据
            dist[0] = instance_calc_distance(ToF_ds1[0]);
            dist[1] = instance_calc_distance(ToF_ds1[1]);
            dist[2] = instance_calc_distance(ToF_ds1[2]);
            dist[3] = instance_calc_distance(ToF_ds1[3]);

            // 装载数据
            u = sprintf((uint8_t *)sendbuff[0], "t%x %08x %08x %08x %08x : %04x %04x %04x %04x\r\n", taddr, dist[0],
                        dist[1], dist[2], dist[3], cand_anchor_addr[0], cand_anchor_addr[1], cand_anchor_addr[2],
                        cand_anchor_addr[3]);
            // 通过串口发给上位机
            if (u > 0) {
                Usart1_DmaTx(sendbuff, u);
                Usart3_DmaTx(sendbuff, u);
                u = 0;
            }

            // 清空缓冲区，
            memset(sendbuff, 0, sizeof(sendbuff));
            memset(dist, 0, sizeof(dist));

            // 发送来自Tag的DS-TWR数据
            dist[0] = instance_calc_distance(ToF_ds2[0]);
            dist[1] = instance_calc_distance(ToF_ds2[1]);
            dist[3] = instance_calc_distance(ToF_ds2[2]);
            dist[3] = instance_calc_distance(ToF_ds2[3]);

            // 装载数据
            u = sprintf((uint8_t *)sendbuff[0], "t%x %08x %08x %08x %08x : %04x %04x %04x %04x\r\n", taddr, dist[0],
                        dist[1], dist[2], dist[3], cand_anchor_addr[0], cand_anchor_addr[1], cand_anchor_addr[2],
                        cand_anchor_addr[3]);
            // 通过串口发给上位机
            if (u > 0) {
                Usart1_DmaTx(sendbuff, u);
                Usart3_DmaTx(sendbuff, u);
                u = 0;
            }
            memset(sendbuff, 0, sizeof(sendbuff));
        }
        ins.nextState = TA_INIT; // 本次轮询结束，重新初始化
    }
}

/**
 * @brief 发送完成回调
 */
void twr_tx_tag_cb(const dwt_cb_data_t *txd) {
    if (mode == TAG) {
        cb_flag++;
        if (ins.nextState != TA_TX_WAIT_CONF || ins.nextState != TA_RXE_WAIT) {
            // twr_rx_error_cb(txd);
        }
    } else if (mode == ANCHOR) {
        if (ins.nextState != TA_TX_WAIT_CONF || ins.nextState != TA_RXE_WAIT || ins.nextState != TA_INIT) {
            // twr_rx_error_cb(txd);
        }
    }
}

/**
 * @brief 接收正常回调
 */
void twr_rx_cb(const dwt_cb_data_t *rxd) {
    if (mode == TAG) {
        // 读取数据
        dwt_readrxdata((uint8_t *)&dw_event.msgu.frame[0], rxd->datalength, 0);

        // 接收到CAND或者RESPONSE，解析源地址
        if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == CAND ||
            dw_event.msgu.rxmsg_ss.messageData[FCODE] == RESPONSE) {
            addr16_t = dw_event.msgu.rxmsg_ss.sourceAddr[0] + (dw_event.msgu.rxmsg_ss.sourceAddr[1] << 8);
            aaddr = addr8_t = addr16_t % MAX_RESP_ANCHOR_NUM;
        }
    } else if (mode == ANCHOR) {
        // 读取数据
        dwt_readrxdata((uint8_t *)&dw_event.msgu.frame[0], rxd->datalength, 0);

        // 接收到POLL或者GRANT，解析源地址
        if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == POLL || dw_event.msgu.rxmsg_ss.messageData[FCODE] == GRANT) {
            addr16_t = dw_event.msgu.rxmsg_ss.sourceAddr[0] + (dw_event.msgu.rxmsg_ss.sourceAddr[1] << 8);

            if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == GRANT) cb_flag++;

            taddr = addr16_t;
        }
    }
}

/**
 * @brief 接收超时回调
 */
void twr_rx_timeout_cb(const dwt_cb_data_t *rxd) {
    if (mode == TAG) {
        Tagrxtime = 0;
        if (ins.previousState == TA_RX_WAIT_CAND) {
            if (cand_valid_num > 0) {
                // 接收到CAND的数量大于0，依旧发送GRANT
                ins.nextState = TA_TXE_WAIT;
            } else {
                ins.nextState = TA_INIT;
            }
        } else if (ins.previousState == TA_RX_WAIT_RESPONSE) {
            if (response_num > 0) {
                ins.nextState = TA_FEEDBACK_DATA;
            } else {
                ins.nextState = TA_INIT;
            }
        }
    } else if (mode == ANCHOR) {
        Anchrxtime = 0;
        ins.previousState = ins.testAppState;
        ins.nextState = TA_INIT;
    }
}

/**
 * @brief 接收异常回调
 */
void twr_rx_error_cb(const dwt_cb_data_t *rxd) {
    dwt_forcetrxoff();

    ins.previousState = ins.testAppState;
    ins.nextState = TA_INIT;
}
/*!
   ------------------------------------------------------------------------------------------------------------------
                获取TX时间戳
 */
static uint64_t get_tx_timestamp_u64(void) {
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*!
   ------------------------------------------------------------------------------------------------------------------
                获取RX时间戳
 */
static uint64 get_rx_timestamp_u64(void) {
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/**
 * @Pazfic: 获取系统时间，单位为DW_ticks，官方实现
 */
static uint64_t get_sys_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readsystime(ts_tab);
    for (i = 4; i >= 0; --i) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/**
 * @brief 设置时间戳数据
 * @param ts_field
 * @param ts
 */
static void msg_set_ts(uint8 *ts_field, uint64 ts) {
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        ts_field[i] = (uint8)ts;
        ts >>= 8;
    }
}

/*!
   ------------------------------------------------------------------------------------------------------------------
                final包数据读取
 */
static void msg_get_ts(const uint8 *ts_field, uint32_t *ts) {
    int i;
    *ts = 0;
    for (i = FINAL_MSG_TS_LEN - 1; i >= 0; i--) {
        *ts <<= 8;
        *ts |= ts_field[i];
    }
}
