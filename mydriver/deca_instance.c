#include <stdio.h>

#include "dwt_common.h"
#include "instance.h"
#include "main.h"
#include "tim.h"

#define MAXTAGNUM  128 // ��������ǩ����
#define MINTAGSLOT 10  // �����ǩ���ʱ��
uint8_t mode = 0;      // ��ǩ��վģʽ
uint8_t sendbuff[300]; // ����buff
uint8_t index2 = 0;    // ��ǩ�յ���վresponse��ʱ������λ��
uint8_t Tarecrx = 0;   // ��ǩģʽ
uint8_t faild_rxtime = 0;
uint16_t addr8_t = 0;      // ��վ�̵�ַ
uint16_t addr16_t = 0;     // ��վ����ַ
uint16_t Anch_addr = 1000; // ��վ��ַ
int MAX_TAG_SIZE = 64;     // ��ǩ����
int N_Slot = 64;           // ʱ��Ƭ����
int T_Slot = 20;           // 1��ʱ��Ƭ��ʱ�� ms
int T_Round = 64 * 20;     // ��ѯ����ʱ��
int GetT_ID = 0;           // ��ǩID
int rangnum = 0;           // ���������ϼ�
int POLL_Value = 0;        // Ԥ�����
int aaddr, taddr;          // ��վ��ǩ��ַ
int contrlsendms = 10;     // ���Ʒ���ʱ��
int contrlbit = 0;         // ���Ʒ���λ
int POLL_count = 0;        // poll����++
int response_num = 0;      // ��ǩ���յ�response������
int lastwaittime = 0;      // ��ǩӦ�ȴ�ʱ��
int twl_address_index = 0;

/***************************************************************************************************/

#include "iwdg.h"

// ��վ�Ĳ����࣬�ͻ�վ����Ӧ��Χ����λm
#define DISTRIBUTION_DIST   5                         // m
#define RESPONSE_RANGE      DISTRIBUTION_DIST * 1.414 // m
#define TIMESTAMP_LEN       5                         // ����ֶεĳ��ȣ������ֽ���Ϊ5��ʱ���
#define MAX_RESP_ANCHOR_NUM 9                         // �����Ӧ��Anchor����
#define DW_MASK_40          ((1ULL << 40) - 1ULL)     // ���DW1000��40Bʱ����Ļ��ƴ�������

#define POLL                0x81 // POLL������
#define CAND                0x71 // CAND������
#define GRANT               0x83 // GRANT������
#define RESPONSE            0x70 // RESPONSE������

#define POLL_TX_TS          2  // POLL��Ϣ��poll_tx_ts��ƫ����
#define POLL_RX_TS          2  // CAND��Ϣ��poll_rx_ts��ƫ����
#define CAND_TX_TS          7  // CAND��Ϣ��cand_tx_ts��ƫ����
#define CAND_RX_TS          2  // GRANT��Ϣ��cand_rx_ts��ƫ����
#define GRANT_TX_TS         7  // GRANT��Ϣ��grant_tx_ts��ƫ����
#define GRANT_RX_TS         9  // RESPONSE��Ϣ��grant_rx_ts��ƫ����
#define RESP_TX_TS          14 // RESPONSE��Ϣ��resp_tx_ts��ƫ����
#define RESP_SLEEP_CORR_L   2  // RESPONSE��Ϣ��sleep_correction��ƫ����
#define RESP_SLEEP_CORR_H   3  // RESPONSE��Ϣ��sleep_correction��ƫ����
#define RESP_TOF            4  // RESPONSE��Ϣ��ToF��ƫ����
#define RANGE_NUM           1  // RangeNum��ƫ����

#define POLL_MSG_LEN        7  // Tag���͵�POLL֡�ĳ���-FCODE(1)+RANGE_NUM(1)+POLL_TX_TS(5)
#define CAND_MSG_LEN        12 // Anchor���͵�CAND֡�ĳ���-FCODE(1)+RANGE_NUM(1)+POLL_RX_TS(5)+CAND_TX_TS(5)
#define GRANT_MSG_LEN       12 // Tag���͵�GRANT֡�ĳ���-FCODE(1)+RANGE_NUM(1)+GRANT_TX_TS(5)+CAND_RX_TS(5)
// Anchor���͵�RESPONSE֡�ĳ���-FCODE(1)+RANGE_NUM(1)+SLEEP_CORRECTION(2)+TOF(5)+GRANT_RX_TS(5)+RESP_TX_TS(5)
#define RESPONSE_MSG_LEN    19
#define CRC_LEN             1 // CRCУ��λ����

uint8_t anchor_group_id = 0x01; // Anchor�����
uint8_t tag_group_id = 0x00;    // Tag�����

/// ͨ����¼����ʱ�����Tag��Anchor������ʹ��DS-TWR������룬������ѯ���Լ���3��ToF(һ��SS-TWR������DS-TWR)
/// Tag: ToF_ss = 1/2 x [(cand_rx_ts - poll_tx_ts) - (cand_tx_ts - poll_rx_ts)]
/// Anchor: ToF_ds1 = 1/4 x [(grant_rx_ts - cand_tx_ts) - (grant_tx_ts -
/// cand_rx_ts) + (cand_rx_ts - poll_tx_ts) - (cand_tx_ts - poll_rx_ts)] Tag:
/// ToF_ds2 = 1/4 x [(resp_rx_ts - grant_tx_ts) - (resp_tx_ts - grant_rx_ts) +
/// (grant_rx_ts - cand_tx_ts) - (grant_tx_ts - cand_rx_ts)]
/// Anchor��RESPONSE��װ��ToF_ds1��Tag�˾Ϳ��Ի��3�������Ĳ����Ϣ�����������ں����β����Ϣ��12ά���ݣ���������Ӿ�ȷ�ľ���

uint64_t poll_tx_ts;  // ����pollʱ��
uint64_t poll_rx_ts;  // ����pollʱ��
uint64_t cand_tx_ts;  // ����candʱ��
uint64_t cand_rx_ts;  // ����candʱ��
uint64_t grant_tx_ts; // ����grantʱ��
uint64_t grant_rx_ts; // ����grantʱ��
uint64_t resp_tx_ts;  // ����responseʱ��
uint64_t resp_rx_ts;  // ����responseʱ��

// ��¼CAND�Ľ���ʱ���
uint64_t cand_rx_ts_arr[MAX_ANCHOR_LIST_SIZE] = {0};
// ��¼������Ч��CAND��Դ��ַ
uint16_t cand_anchor_addr[MAX_ANCHOR_LIST_SIZE] = {0};
// ��Ч��CANDԴ��ַ��������RX_CAND״̬�����ӣ���TX_GRANT״̬�¼���
uint8_t cand_valid_num = 0;

// SS-TWR�õ���ToF������
uint64_t ToF_ss[MAX_ANCHOR_LIST_SIZE] = {0};
// Anchor��DS-TWR�õ���ToF������
uint64_t ToF_ds1[MAX_ANCHOR_LIST_SIZE] = {0};
// Tag��DS-TWR�õ���ToF������
uint64_t ToF_ds2[MAX_ANCHOR_LIST_SIZE] = {0};

// debug
double dist_ss = 0;
uint8_t RET_T = 0;
uint8_t RET_A = 0;
int8_t miss_flag = 0;
int8_t state_flag = 0;
int8_t cb_flag = 0;

static uint64_t get_sys_ts_u64(void);
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

// �ӵ��β��õ���ToF������룬��λmm
double instance_calc_distance(uint64_t tof) {
    int i;
    double dis;
    double tofx = 0;
    tofx = (tof)*DWT_TIME_UNITS;
    dis = tofx * ((float)SPEED_OF_LIGHT) * 1000;
    if ((dis > 2000000.0) || (dis <= 0)) // ��Ч���
    {
        if (dis > 2000000.0)
            miss_flag = -1;
        else if (dis <= 0)
            miss_flag = -1;

        dis = -1;
    } else if (dis > RESPONSE_RANGE * 1000) // ������Ӧ��Χ
    {
        miss_flag = -1;
        dis = -1;
    } else {
        miss_flag = -1;
    }

    return dis;
}

uint64_t dw_ts_diff(uint64_t ts1, uint64_t ts2) { return (ts1 - ts2) & DW_MASK_40; }

/***************************************************************************************************/

double distance[MAX_ANCHOR_LIST_SIZE] = {0};       // ��ž���
static uint64_t resp_rx_tag[MAX_ANCHOR_LIST_SIZE]; // ����response ʱ��
// uint32 Response_tx_time=0;

event_data_t event;
event_data_t dw_event;
instance_data_t ins;
extern int Get_tag_inf;

static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);
static void msg_set_ts(uint8 *ts_field, uint64 ts);
static void msg_get_ts(const uint8 *ts_field, uint64 *ts);

void insaddress(uint16_t address) // ��ȡ��ǩID����վ��ַ�����ð�ͷ֡
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
            // ��ʼ��
            TA_Init_Handler(modes);
            state_flag += 2;
        } break;
        case TA_TXE_WAIT: {
            // �м�̬����TA_INIT/TA_RX_WAIT_DATA�л����������ݵ�״̬
            // ���·��Ͷ�Ӧ���ݵı�־λ
            TA_Txe_Handler(modes);
        } break;
        case TA_TX_WAIT_CONF: {
            // �м�̬������������֮���ȷ�ϣ��л���TA_RXE_WAIT����TA_INIT
            // ���ͽ����󣬸��½����ض����ݵı�־λ
            TA_Tx_Confirm_Handler(modes);
        } break;
        case TA_RXE_WAIT: {
            // �м�̬����TA_INIT/TA_TX_WAIT_CONF�л��������ض����ݵ�״̬
            // �����շ��������ý��ճ�ʱ
            TA_Rxe_Handler(modes);
        } break;
        case TA_RX_WAIT_POLL: {
            // Anchor �ȴ�POLL״̬
            TA_Rx_Poll_Handler();
            state_flag--;
        } break;
        case TA_TXPOLL_WAIT_SEND: {
            // Tag ����POLL״̬
            TA_Tx_Poll_Handler();
        } break;
        case TA_TXCAND_WAIT_SEND: {
            // Anchor ����CAND״̬
            TA_Tx_Cand_Handler();
        } break;
        case TA_RX_WAIT_CAND: {
            // Tag �ȴ�CAND״̬
            TA_Rx_Cand_Handler();
        } break;
        case TA_TXGRANT_WAIT_SEND: {
            // Tag ����GRANT״̬
            TA_Tx_Grant_Handler();
        } break;
        case TA_RX_WAIT_RESPONSE: {
            // Tag �ȴ�RESPONSE״̬
            TA_Rx_Response_Handler();
        } break;
        case TA_TXRESPONSE_WAIT_SEND: {
            // Anchor ����RESPONSE״̬
            TA_Tx_Response_Handler();
        } break;
        case TA_FEEDBACK_DATA: {
            // Tag��������
            TA_Feedback_Handler();
        } break;
        case TA_SLEEP: {
            // �ȴ�ʱ϶�����崦����Ϊ�ȴ��ѱ����õ�waittime��ʱ���ж����Լ�Ϊ0
            ins.testAppState = TA_SLEEP;
            ins.previousState = ins.testAppState;
        } break;
        case TA_SLEEP_DONE: {
            // ���߽�����׼������
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

            aaddr = (ins.instanceAddress16) & 0xFF; // ��վ��ַ
            memcpy(ins.eui64, &ins.instanceAddress16, ADDR_BYTE_SIZE_S);
            dwt_seteui(ins.eui64);
            dwt_setpanid(ins.panID);

            ins.previousState = TA_INIT;
            ins.nextState = TA_RXE_WAIT; // ״̬���л�: ANCHOR: INIT->RXE_WAIT

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

            // ������һ���յ��������Anchor��RESPONSE�������Լ�����һ��ʱ϶��ʲôʱ��
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

                    // ��һ����ѯû���յ��ظ���1000ms����һ��
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_SLEEP;
                } else if (Tarecrx == 2) {
                    // ��һ����ѯ���յ��ظ���������һ����ѯ
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_TXE_WAIT;
                } else if (Tarecrx == 0) {
                    waittime = T_ID * 200;
                    // ������һ�η��͵ȴ�ʱ��
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_SLEEP;
                }
            }
            break;
        }
    }
}

/**
 * @brief �м�̬����������TA_INIT/TA_RX_WAIT_DATA�л��������ض����ݵ�״̬
 * @note  �м�̬���ı�״̬����previousState
 * @param mode
 */
static void TA_Txe_Handler(int module_mode) {
    // Tag��Anchor�л�������״̬���м�̬
    if (module_mode == TAG) {
        ins.testAppState = TA_TXE_WAIT;
        if (ins.previousState == TA_INIT || ins.previousState == TA_SLEEP_DONE) {
            // Tag ��ʼ������(����Tag��ʼ����Ϻ����߽���)��׼������POLL
            ins.nextState = TA_TXPOLL_WAIT_SEND;
        } else if (ins.previousState == TA_RX_WAIT_RESPONSE) {
            // ���յ�RESPONSE��������һ��GRANT
            ins.nextState = TA_TXGRANT_WAIT_SEND;
        }
    } else if (module_mode == ANCHOR) {
        ins.testAppState = TA_TXE_WAIT;
        if (ins.previousState == TA_SLEEP_DONE) {
            // ���߽�����׼������CAND
            ins.nextState = TA_TXCAND_WAIT_SEND;
        } else if (ins.previousState == TA_RX_WAIT_GRANT) {
            // ���յ�GRANT������RESPONSE
            ins.nextState = TA_TXRESPONSE_WAIT_SEND;
        }
    }
}

/**
 * @brief �м�̬���ɷ����ض����ݽ������л���TA_RXE_WAIT����TA_INIT
 * @note  �м�̬���ı�״̬����previousState
 * @param mode
 */
static void TA_Tx_Confirm_Handler(int module_mode) {
    if (module_mode == TAG) {
        // Tag ȷ�Ϸ������ݣ��л�������
        ins.testAppState = TA_TX_WAIT_CONF;
        ins.nextState = TA_RXE_WAIT;
    } else if (module_mode == ANCHOR) {
        ins.testAppState = TA_TX_WAIT_CONF;
        if (ins.previousState == TA_TXRESPONSE_WAIT_SEND) {
            // Anchor ȷ�Ϸ���RESPONSE�����ֽ��������³�ʼ��
            ins.nextState = TA_INIT;
        } else if (ins.previousState == TA_TXCAND_WAIT_SEND) {
            // Anchor ȷ�Ϸ���CAND���ȴ�GRANT
            ins.nextState = TA_RXE_WAIT;
        }
    }
}

/**
 * @brief �м�̬����TA_INIT���߷����ض�����֮���л����������ݵ�״̬��ͬʱ���ý��ճ�ʱʱ��
 * @note  �м�̬���ı�״̬����previousState
 * @param mode
 */
static void TA_Rxe_Handler(int module_mode) {
    if (module_mode == TAG) {
        ins.testAppState = TA_RXE_WAIT;
        if (ins.previousState == TA_TXPOLL_WAIT_SEND) {
            // ȷ�Ϸ���POLL
            dwt_setrxtimeout(ins.responseTO * 2200); // ���õȴ�CAND��ʱ��Ϊ������Ӧ����*2200us
            ins.nextState = TA_RX_WAIT_CAND;         // �ȴ�CAND��Ϣ
        } else if (ins.previousState == TA_TXGRANT_WAIT_SEND) {
            // ȷ�Ϸ���GRANT
            dwt_setrxtimeout(2200);              // ���õȴ�RESPONSE��ʱ��Ϊ2200us
            ins.nextState = TA_RX_WAIT_RESPONSE; // �ȴ�RESPONSE��Ϣ
        }
        dwt_rxenable(DWT_START_RX_IMMEDIATE); // ���̿�������
    } else if (module_mode == ANCHOR) {
        ins.testAppState = TA_RXE_WAIT;
        if (ins.previousState == TA_TXCAND_WAIT_SEND) {
            // ȷ�Ϸ���CAND
            dwt_setrxtimeout(8800); // ���õȴ�GRANT��ʱ��Ϊ8800us
            ins.nextState = TA_RX_WAIT_GRANT;
        } else if (ins.previousState == TA_INIT) {
            // ��ʼ����������������
            if ((2 * T_Slot * 1100) > 65535) {
                dwt_setrxtimeout(65535);
            } else {
                dwt_setrxtimeout(2 * T_Slot * 1100);
            }
            ins.nextState = TA_RX_WAIT_POLL;
        }
        dwt_rxenable(DWT_START_RX_IMMEDIATE); // ���̿�������
    }
}

static void TA_Rx_Poll_Handler(void) {
    // ״̬������
    ins.testAppState = TA_RX_WAIT_POLL;
    // ����Ƿ���½�����Ϣ
    if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0) {
        // δ���յ�����
        ins.previousState = ins.testAppState;
        ins.nextState = TA_RX_WAIT_POLL;
    } else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == POLL) {
        Anch_RecNt = 0;

        poll_rx_ts = get_rx_timestamp_u64(); // ��ȡPOLL����ʱ��

        // ����CRCУ��
        uint8_t crc = 0;
        int r = 0;
        int crc_len = POLL_MSG_LEN;
        for (r = 0; r < crc_len; r++) crc += dw_event.msgu.rxmsg_ss.messageData[r];
        crc = (~crc) + 1;

        miss_flag++;
        if (crc == dw_event.msgu.rxmsg_ss.messageData[POLL_MSG_LEN]) {
            // @Pazfic
            // ��ȡpoll����ʱ���
            msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[POLL_TX_TS], &poll_tx_ts);
            // debug
            twl_address_index = 0;
            taddr = GetT_ID =
                dw_event.msgu.rxmsg_ss.sourceAddr[0] + (dw_event.msgu.rxmsg_ss.sourceAddr[1] << 8); // ��ȡ��ǩ��ַ

            // ��ȡ��ǩ��ID(��1��ʼ)
            if (sys_para.twlt == 0x0000) {
                if ((0 < GetT_ID) && (GetT_ID <= N_Slot))
                    twl_address_index = GetT_ID;
                else
                    twl_address_index = 0;
            }

            // ��ǩID�Ϸ�
            if (twl_address_index > 0) {
                // @Pazfic: ���յ�POLL֮�󣬷���CAND֡
                rangnum = ins.rangeNum = dw_event.msgu.rxmsg_ss.messageData[POLL_RNUM]; // ��ȡ������
                ins.msg_f.seqNum = dw_event.msgu.rxmsg_ss.seqNum++;                     // �������к�
                memcpy(&(ins.msg_f.destAddr[0]), &(dw_event.msgu.rxmsg_ss.sourceAddr[0]),
                       ADDR_BYTE_SIZE_S); // װ��Ŀ���ַ
                ins.msg_f.sourceAddr[0] = ins.eui64[0];
                ins.msg_f.sourceAddr[1] = ins.eui64[1]; // װ��Դ��ַ
                msg_set_ts(&ins.msg_f.messageData[POLL_RX_TS],
                           poll_rx_ts);                     // װ��poll����ʱ��
                ins.msg_f.messageData[POLL_RNUM] = rangnum; // װ�ز�����
                ins.msg_f.messageData[FCODE] = CAND;        // װ�ع�����

                uint8_t slot_idx = ins.instanceAddress16 % MAX_RESP_ANCHOR_NUM;
                waittime = slot_idx * 10 + 3; // ��������ʱ�䣬�ȴ�ʱ϶
                contrlbit = 1;

                ins.previousState = ins.testAppState;
                ins.nextState = TA_SLEEP;
            } else if (twl_address_index == 0) {
                // ʱ϶�������ص�����
                ins.previousState = ins.testAppState;
                ins.nextState = TA_RX_WAIT_POLL;
            }
        } else {
            // У��ʧ�ܣ��ص�����
            ins.previousState = ins.testAppState;
            ins.nextState = TA_RX_WAIT_POLL;
        }

        dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
    } else {
        // ��ΪPOLL���ص�����
        Anch_RecNt = 0; // ���ý��ձ�ʶ
        ins.previousState = ins.testAppState;
        ins.nextState = TA_RX_WAIT_POLL;

        dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
    }
    Anch_RecNt = 1;
}

static void TA_Tx_Poll_Handler(void) {
    // ����poll���Ͳ���������

    uint16_t frameLength = 0;
    uint8_t ret_T;
    frameLength = FRAME_CRTL_AND_ADDRESS_S + POLL_MSG_LEN + CRC_LEN + FRAME_CRC; // ����poll������

    ins.testAppState = TA_TXPOLL_WAIT_SEND;
    ins.previousState = TA_TXE_WAIT;
    ins.msg_f.seqNum = ins.frameSN++;
    ins.msg_f.sourceAddr[0] = sys_para.uwbid & 0xff;
    ins.msg_f.sourceAddr[1] = (sys_para.uwbid >> 8) & 0xff; // װ��Դ��ַ
    ins.msg_f.destAddr[0] = 0xff;
    ins.msg_f.destAddr[1] = 0xff; // ���ù㲥
    ins.msg_f.messageData[POLL_RNUM] = ins.rangeNum++;
    ins.msg_f.messageData[FCODE] = POLL;
    // �ȴ���Ӧ����Ϊ4
    ins.responseTO = 4;

    // @Pazfic: ���㷢��ʱ���
    poll_tx_ts = get_sys_ts_u64() + (1000ULL * (uint64_t)UUS_TO_DWT_TIME); // �ӳ�1ms����
    uint32_t delay = (uint32_t)(poll_tx_ts >> 8);
    dwt_setdelayedtrxtime(delay);                                        // �����ӳٷ���
    uint64_t precise_tx_ts = ((delay & 0xFFFFFFFEUL) << 8) + TX_ANT_DLY; // ������ʵ�ķ���ʱ��
    poll_tx_ts = precise_tx_ts;

    // װ����չ֡��ʱ���
    msg_set_ts(&ins.msg_f.messageData[POLL_TX_TS], poll_tx_ts);

    // ����CRC
    ins.msg_f.messageData[POLL_MSG_LEN] = 0;
    int crc_len = POLL_MSG_LEN;
    int r = 0;
    for (r = 0; r < crc_len; r++) ins.msg_f.messageData[POLL_MSG_LEN] += ins.msg_f.messageData[r];
    ins.msg_f.messageData[POLL_MSG_LEN] = (~ins.msg_f.messageData[POLL_MSG_LEN]) + 1; // ����crcλ

    dwt_writetxdata(frameLength, (uint8_t *)&(ins.msg_f), 0); // ���poll����
    dwt_writetxfctrl(frameLength, 0, 1);                      // ���Ʒ���
    ret_T = dwt_starttx(DWT_START_TX_DELAYED);                // �ӳٷ��Ͳ��ӳٽ���
    RET_T = ret_T;

    if (ret_T == DWT_SUCCESS) {
        Tarecrx = 1;
        POLL_count++;
        response_num = 0;
        cand_valid_num = 0;

        // ���CANDԴ��ַ����
        memset(cand_anchor_addr, 0, sizeof(cand_anchor_addr));
        // ���CAND����ʱ�������
        memset(cand_rx_ts_arr, 0, sizeof(cand_rx_ts_arr));

        ins.previousState = ins.testAppState;
        ins.nextState = TA_TX_WAIT_CONF;
    } else {
        // ����ʧ�ܣ��ر��շ��������³�ʼ��
        dwt_forcetrxoff();
        ins.previousState = ins.testAppState;
        ins.nextState = TA_INIT;
    }
}

static void TA_Rx_Cand_Handler(void) {
    // �ȴ�CAND
    ins.testAppState = TA_RX_WAIT_CAND;
    if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0) {
        ins.previousState = ins.testAppState;
        ins.nextState = TA_RX_WAIT_CAND;
    } else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == CAND) {
        Tag_RecNt = 0;
        // ����CRCУ��
        uint8_t crc = 0;
        uint8_t Anch_num = 0;
        int r = 0;
        int crc_len = CAND_MSG_LEN;
        for (r = 0; r < crc_len; r++) crc += dw_event.msgu.rxmsg_ss.messageData[r];
        crc = (~crc) + 1;

        Tagrxtime = 0;
        // У��CRC
        if (dw_event.msgu.rxmsg_ss.messageData[CAND_MSG_LEN] == crc) {
            // �鿴Ŀ���ַ�Ƿ��Ӧ���豸
            if (T_ID == (dw_event.msgu.rxmsg_ss.destAddr[0] + (dw_event.msgu.rxmsg_ss.destAddr[1] << 8))) {
                // ��¼����ʱ��
                cand_rx_ts = get_rx_timestamp_u64();
                // ��ȡCAND����ʱ���POLL����ʱ��
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[CAND_TX_TS], &cand_tx_ts);
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[POLL_RX_TS], &poll_rx_ts);

                // SS-TWR����һ�ξ���
                uint64_t rrt = dw_ts_diff(cand_rx_ts, poll_tx_ts);
                uint64_t rpt = dw_ts_diff(cand_tx_ts, poll_rx_ts);
                uint64_t tof_ticks = 0;
                if (rrt > rpt) {
                    tof_ticks = (rrt - rpt) >> 1;
                } else {
                    tof_ticks = 0;
                }

                dist_ss = instance_calc_distance(tof_ticks); // �������

                if (dist_ss > 0) // ������Ч����¼Դ��ַ�;���
                {
                    // ��¼��ַ�����롢�ͽ���ʱ���
                    cand_rx_ts_arr[cand_valid_num] = cand_rx_ts;
                    cand_anchor_addr[cand_valid_num] =
                        dw_event.msgu.rxmsg_ss.sourceAddr[0] + (dw_event.msgu.rxmsg_ss.sourceAddr[1] << 8);
                    ToF_ss[cand_valid_num] = tof_ticks;

                    if (cand_valid_num < MAX_ANCHOR_LIST_SIZE) {
                        // δ���յ����㶨λ�����ToFs����������
                        cand_valid_num++; // ����cand_valid_num
                        ins.previousState = ins.testAppState;
                        ins.nextState = TA_RX_WAIT_CAND;
                    } else if (cand_valid_num == MAX_ANCHOR_LIST_SIZE) {
                        // ���յ����㶨λ�����ToFs����ͨ�����ڷ���һ�����ݣ����л�Ϊ����GRANT
                        dwt_forcetrxoff(); // �ر��շ���
                        ins.previousState = ins.testAppState;
                        ins.nextState = TA_FEEDBACK_DATA;
                    }
                } else {
                    // ������Ч������������ݣ���������
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_RX_WAIT_CAND;
                }
            } else {
                // �Ǳ��豸��ַ���������ݣ���������
                ins.previousState = ins.testAppState;
                ins.nextState = TA_RX_WAIT_CAND;
            }
        } else {
            // У����󣬷������ݣ���������
            ins.previousState = ins.testAppState;
            ins.nextState = TA_RX_WAIT_CAND;
        }
    } else {
        // ���ݲ�ΪCAND���������ݣ���������
        Tag_RecNt = 0;
        ins.previousState = ins.testAppState;
        ins.previousState = TA_RX_WAIT_CAND;
    }
    Tag_RecNt = 1;
    dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
}

static void TA_Tx_Cand_Handler(void) {
    ins.testAppState = TA_TXCAND_WAIT_SEND;

    uint16_t frameLength = 0;
    uint8_t ret_A = 0;
    int calcwaittime = 0;

    frameLength = FRAME_CRTL_AND_ADDRESS_S + CAND_MSG_LEN + CRC_LEN + FRAME_CRC; // ����CAND������

    // �����ӳٷ���
    cand_tx_ts = get_sys_ts_u64() + (1000ULL * (uint64_t)UUS_TO_DWT_TIME); // �ӳ�100us����
    uint32_t delay = (uint32_t)(cand_tx_ts >> 8);
    dwt_setdelayedtrxtime(delay);                                                    // �����ӳٷ���ʱ���
    uint64_t precise_tx_ts = (uint64_t)(((delay & 0xFFFFFFFEUL) << 8) | TX_ANT_DLY); // ���㾫ȷ����ʱ���
    cand_tx_ts = precise_tx_ts;

    msg_set_ts(&ins.msg_f.messageData[CAND_TX_TS], cand_tx_ts); // װ��cand����ʱ���

    // ����CRC
    ins.msg_f.messageData[CAND_MSG_LEN] = 0;
    int crc_len = CAND_MSG_LEN;
    for (int r = 0; r < crc_len; r++) ins.msg_f.messageData[CAND_MSG_LEN] += ins.msg_f.messageData[r];
    ins.msg_f.messageData[CAND_MSG_LEN] = (~ins.msg_f.messageData[CAND_MSG_LEN]) + 1; // ����crcλ

    dwt_writetxdata(frameLength, (uint8_t *)&(ins.msg_f), 0); // ���CAND����
    dwt_writetxfctrl(frameLength, 0, 1);                      // ���Ʒ���
    ret_A = dwt_starttx(DWT_START_TX_DELAYED);                // �ӳٷ���
    RET_A = ret_A;

    // �ж��Ƿ��ͳɹ����ɹ�����뷢��ȷ�ϣ�ʧ����ص���ʼ��
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
        ins.nextState = TA_RX_WAIT_GRANT;
    } else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == GRANT) {
        Anch_RecNt = 0;
        // У��CRC
        uint8_t crc = 0;
        int r = 0;
        int crc_len = GRANT_MSG_LEN;
        for (r = 0; r < crc_len; r++) crc += dw_event.msgu.rxmsg_ss.messageData[r];
        crc = (~crc) + 1;

        if (dw_event.msgu.rxmsg_ss.messageData[GRANT_MSG_LEN] == crc) {
            // ���Ŀ���ַ�Ƿ��뱾�豸һ��
            if (ins.instanceAddress16 ==
                (dw_event.msgu.rxmsg_ss.destAddr[0] + (dw_event.msgu.rxmsg_ss.destAddr[1] << 8))) {
                // ��¼����ʱ��
                grant_rx_ts = get_rx_timestamp_u64();
                // ��ȡGRANT����ʱ���CAND����ʱ��
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[GRANT_TX_TS], &grant_tx_ts);
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[CAND_RX_TS], &cand_rx_ts);
                // DS-TWR�������
                uint64_t rrt1 = dw_ts_diff(cand_rx_ts, poll_tx_ts);
                uint64_t rrt2 = dw_ts_diff(grant_rx_ts, cand_tx_ts);
                uint64_t rpt1 = dw_ts_diff(cand_tx_ts, poll_rx_ts);
                uint64_t rpt2 = dw_ts_diff(cand_rx_ts, grant_tx_ts);
                uint64_t tof_ticks = 0;
                if (rrt1 > rpt1 && rrt2 > rpt2) {
                    // DS-TWR�������
                    tof_ticks = (((rrt1 - rpt1) + (rrt2 - rpt2)) >> 2);
                } else {
                    tof_ticks = 0;
                }

                // ׼������RESPONSE��Ԥ��װ��һ����������
                rangnum = ins.rangeNum = dw_event.msgu.rxmsg_ss.messageData[RANGE_NUM]; // ��ȡ������
                ins.msg_f.seqNum = dw_event.msgu.rxmsg_ss.seqNum++;                     // �������к�
                memcpy(&(ins.msg_f.destAddr[0]), &(dw_event.msgu.rxmsg_ss.sourceAddr[0]),
                       ADDR_BYTE_SIZE_S); // װ��Ŀ���ַ
                ins.msg_f.sourceAddr[0] = ins.eui64[0];
                ins.msg_f.sourceAddr[1] = ins.eui64[1]; // װ��Դ��ַ
                msg_set_ts(&ins.msg_f.messageData[GRANT_RX_TS],
                           grant_rx_ts);                    // װ��grant����ʱ��
                ins.msg_f.messageData[RANGE_NUM] = rangnum; // װ�ز�����
                ins.msg_f.messageData[FCODE] = RESPONSE;    // װ�ع�����

                dwt_forcetrxoff(); // �ر��շ���
                ins.previousState = ins.testAppState;
                ins.nextState = TA_TXE_WAIT;
            } else {
                // Ŀ���ַ�뱾�豸��һ�£��������ݣ���������
                ins.previousState = ins.testAppState;
                ins.nextState = TA_RX_WAIT_GRANT;
            }
        } else {
            // У��ʧ�ܣ��������ݣ���������
            ins.previousState = ins.testAppState;
            ins.nextState = TA_RX_WAIT_GRANT;
        }
    } else {
        // ��ΪGRANT���ݣ��������ݣ���������
        Anch_RecNt = 0;
        ins.previousState = ins.testAppState;
        ins.nextState = TA_RX_WAIT_GRANT;
    }

    Anch_RecNt = 1;
    dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
}

static void TA_Tx_Grant_Handler(void) {
    ins.testAppState = TA_TXGRANT_WAIT_SEND;

    uint16_t frameLength = 0;
    uint8_t ret_T = 0;
    frameLength = FRAME_CRTL_AND_ADDRESS_S + GRANT_MSG_LEN + CRC_LEN + FRAME_CRC; // ����Grant������

    ins.msg_f.messageData[FCODE] = GRANT;
    ins.msg_f.messageData[RANGE_NUM] = ins.rangeNum;
    ins.msg_f.sourceAddr[0] = sys_para.uwbid & 0xff;
    ins.msg_f.sourceAddr[1] = (sys_para.uwbid >> 8) & 0xff; // װ��Դ��ַ

    // װ��Ŀ���ַ����cand_anchor_addr�дӺ���ǰȡ��
    ins.msg_f.destAddr[0] = cand_anchor_addr[--cand_valid_num] & 0xff;
    ins.msg_f.destAddr[1] = (cand_anchor_addr[--cand_valid_num] >> 8) & 0xff;

    msg_set_ts(&ins.msg_f.messageData[CAND_RX_TS], cand_rx_ts); // װ��cand����ʱ���

    // �����ӳٷ���
    grant_tx_ts = get_sys_ts_u64() + (1000ULL * (uint64_t)UUS_TO_DWT_TIME); // �ӳ�1000us����
    uint32_t delay = (uint32_t)(grant_tx_ts >> 8);
    dwt_setdelayedtrxtime(delay);                                                  // �����ӳٷ���ʱ���
    uint64_t precise_tx_ts = (uint64_t)((delay & 0xFFFFFFFEUL) << 8) + TX_ANT_DLY; // ���㾫ȷ����ʱ���
    grant_tx_ts = precise_tx_ts;

    msg_set_ts(&ins.msg_f.messageData[GRANT_TX_TS], grant_tx_ts); // װ��GRANT����ʱ���
    // ����CRC
    ins.msg_f.messageData[GRANT_MSG_LEN] = 0;
    int crc_len = GRANT_MSG_LEN;
    for (int r = 0; r < crc_len; r++) ins.msg_f.messageData[GRANT_MSG_LEN] += ins.msg_f.messageData[r];
    ins.msg_f.messageData[GRANT_MSG_LEN] = (~ins.msg_f.messageData[GRANT_MSG_LEN]) + 1; // ����crcλ

    dwt_writetxdata(frameLength, (uint8_t *)&(ins.msg_f), 0); // ���GRANT����
    dwt_writetxfctrl(frameLength, 0, 1);                      // ���Ʒ���
    ret_T = dwt_starttx(DWT_START_TX_DELAYED);                // �ӳٷ���
    RET_T = ret_T;

    // �Ƿ��ͳɹ����ɹ������ȷ�ϣ�ʧ����ص���ʼ���׶�
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
        ins.nextState = TA_RX_WAIT_RESPONSE;
    } else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == RESPONSE) {
        Tag_RecNt = 0;
        uint8_t crc_jy = 0;
        uint8_t Anch_num = 0;
        int r = 0;
        int crc_len = RESPONSE_MSG_LEN; // crcУ�鳤��
        for (r = 0; r < crc_len; r++) crc_jy += dw_event.msgu.rxmsg_ss.messageData[r];
        crc_jy = (~crc_jy) + 1; // ����crc

        Tagrxtime = 0;
        if (dw_event.msgu.rxmsg_ss.messageData[CRC_BIT_RES] == crc_jy) {
            if (T_ID == dw_event.msgu.rxmsg_ss.destAddr[0] + (dw_event.msgu.rxmsg_ss.destAddr[1] << 8)) {
                Tag_RecNt = 0;
                Tarecrx = 2;
                resp_rx_ts = get_rx_timestamp_u64(); // ��¼response����ʱ��
                uint64_t ToF_ds = 0;

                // ��ȡRESPONSE�е�����ʱ���
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[GRANT_RX_TS], &grant_rx_ts);
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[RESP_TX_TS], &resp_tx_ts);
                // ��ȡAnchor�������DS-TWR����
                msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[TOFR], &ToF_ds);
                ToF_ds1[response_num] = ToF_ds;

                // Tag�˼���һ��DS-TWR
                uint64_t rrt1 = dw_ts_diff(resp_rx_ts, grant_tx_ts);
                uint64_t rpt1 = dw_ts_diff(resp_tx_ts, grant_rx_ts);
                uint64_t rrt2 = dw_ts_diff(grant_rx_ts, cand_tx_ts);
                uint64_t rpt2 = dw_ts_diff(grant_tx_ts, cand_rx_ts);
                uint64_t tof_ticks = 0;
                if (rrt1 > rpt1 && rrt2 > rpt2) {
                    // DS-TWR�������
                    tof_ticks = (((rrt1 - rpt1) + (rrt2 - rpt2)) >> 2);
                } else {
                    tof_ticks = 0;
                }
                // ����Tag�������DS-TWR
                ToF_ds2[response_num] = tof_ticks;

                // ��Ӧ��������
                if (response_num < MAX_ANCHOR_LIST_SIZE) response_num++;

                if (response_num == 1) {
                    // ���յ�����Ļ�վ����������ʱ��
                    lastwaittime = waittime = dw_event.msgu.rxmsg_ss.messageData[RESP_SLEEP_CORR_L] +
                                              (dw_event.msgu.rxmsg_ss.messageData[RESP_SLEEP_CORR_H] << 8);
                }

                if ((ins.responseTO - response_num) > 0) {
                    // û�н��յ�����4����վ��response���ص�GRANT����
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_TXE_WAIT;
                } else if (response_num == MAX_ANCHOR_LIST_SIZE) {
                    // ���յ���RESPONSE�����㹻��ɲ�࣬��������
                    POLL_count = 0;
                    dwt_forcetrxoff();
                    ins.previousState = ins.testAppState;
                    ins.nextState = TA_FEEDBACK_DATA;
                }

            } else {
                // ����Ӧ���豸��ַ���ص�����
                ins.previousState = ins.testAppState;
                ins.nextState = TA_RX_WAIT_RESPONSE;
            }
        } else {
            // CRCУ���쳣���ص�����
            ins.previousState = ins.testAppState;
            ins.nextState = TA_RX_WAIT_RESPONSE;
        }
    } else {
        // ��Ϣ��ΪRESPONSE���ص�����
        ins.previousState = ins.testAppState;
        ins.nextState = TA_RX_WAIT_RESPONSE;
    }

    Tag_RecNt = 1;
    dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
}

static void TA_Tx_Response_Handler(void) {
    ins.testAppState = TA_TXPOLL_WAIT_SEND;
    uint16_t frameLength = 0;
    uint8_t ret_A = 0;
    int calcwaittime = 0;

    frameLength = FRAME_CRTL_AND_ADDRESS_S + RESPONSE_MSG_LEN + CRC_LEN + FRAME_CRC; // ����response������

    calcwaittime = ((T_Round + (twl_address_index * T_Slot)) * 10 - Anchpollingtime - 20); // �����ǩ�´η���POLL��ʱ��
    if ((calcwaittime > 0) && (calcwaittime < ((T_Round + (twl_address_index * T_Slot)) * 10))) {
        ins.msg_f.messageData[RESP_SLEEP_CORR_L] = calcwaittime & 0xFF;
        ins.msg_f.messageData[RESP_SLEEP_CORR_H] = (calcwaittime >> 8) & 0xFF; // ������ʱ��װ�ص�response��
    } else {
        ins.msg_f.messageData[RESP_SLEEP_CORR_L] = 0;
        ins.msg_f.messageData[RESP_SLEEP_CORR_H] = 0;
    }

    // �����ӳٷ���
    resp_tx_ts = get_sys_ts_u64() + (1000ULL * (uint64_t)UUS_TO_DWT_TIME); // �ӳ�1000us����
    uint32_t delay = (uint32_t)(resp_tx_ts >> 8);
    dwt_setdelayedtrxtime(delay);
    uint64_t precise_tx_ts = ((delay & 0xFFFFFFFEUL) << 8) + TX_ANT_DLY; // ������ʵ�ķ���ʱ��
    resp_tx_ts = precise_tx_ts;

    msg_set_ts(&ins.msg_f.messageData[RESP_TX_TS], resp_tx_ts); // װ��response����ʱ��

    // ����CRC
    ins.msg_f.messageData[ANCH_RESPONSE_MSG_LEN] = 0;
    int crc_len = ANCH_RESPONSE_MSG_LEN; // ����CRC����
    for (int r = 0; r < crc_len; r++) ins.msg_f.messageData[ANCH_RESPONSE_MSG_LEN] += ins.msg_f.messageData[r];
    ins.msg_f.messageData[ANCH_RESPONSE_MSG_LEN] = (~ins.msg_f.messageData[ANCH_RESPONSE_MSG_LEN]) + 1; // ����crcλ

    dwt_writetxdata(frameLength, (uint8_t *)&(ins.msg_f), 0); // ���response����
    dwt_writetxfctrl(frameLength, 0, 1);                      // ���Ʒ���
    ret_A = dwt_starttx(DWT_START_TX_DELAYED);                // �ӳٷ��Ͳ��ӳٽ���
    RET_A = ret_A;

    if (ret_A == DWT_SUCCESS) {
        // ���ͳɹ����л���ȷ��
        ins.previousState = ins.testAppState;
        ins.nextState = TA_TX_WAIT_CONF;
    } else {
        // ����ʧ�ܣ��ر��շ��������³�ʼ��
        dwt_forcetrxoff();
        ins.previousState = ins.testAppState;
        ins.nextState = TA_INIT;
    }
}

/**
 * @brief �м�̬�������ݷ�������λ��
 * @note  �м�̬���ı��豸��previousState
 */
static void TA_Feedback_Handler() {
    ins.testAppState = TA_FEEDBACK_DATA;

    uint8_t u = 0;
    double dist[MAX_ANCHOR_LIST_SIZE] = {0};
    if (ins.previousState == TA_RX_WAIT_CAND) {
        if (cand_valid_num > 0) {
            // ���յ�4��CAND��ӵ��4��SS-TWR����
            dist[0] = instance_calc_distance(ToF_ss[0]);
            dist[1] = instance_calc_distance(ToF_ss[1]);
            dist[2] = instance_calc_distance(ToF_ss[2]);
            dist[3] = instance_calc_distance(ToF_ss[3]);

            // װ������
            u = sprintf((uint8_t *)sendbuff[0], "t%x %08x %08x %08x %08x : %04x %04x %04x %04x\r\n", taddr, dist[0],
                        dist[1], dist[2], dist[3], cand_anchor_addr[0], cand_anchor_addr[1], cand_anchor_addr[2],
                        cand_anchor_addr[3]);
            // ͨ�����ڷ�����λ��
            if (u > 0) {
                Usart1_DmaTx(sendbuff, u);
                Usart3_DmaTx(sendbuff, u);
                u = 0;
            }
            memset(sendbuff, 0, sizeof(sendbuff));
        }
        ins.nextState = TA_TXE_WAIT; // �л�������GRANT
    } else if (ins.previousState == TA_RX_WAIT_RESPONSE) {
        if (response_num > 0) {
            // ���յ�4��RESPONSE��ӵ��4����Anchor�����DS-TWR���ݺ�4��Tag�����DS-TWR����

            // ��������Anchor��DS-TWR����
            dist[0] = instance_calc_distance(ToF_ds1[0]);
            dist[1] = instance_calc_distance(ToF_ds1[1]);
            dist[2] = instance_calc_distance(ToF_ds1[2]);
            dist[3] = instance_calc_distance(ToF_ds1[3]);

            // װ������
            u = sprintf((uint8_t *)sendbuff[0], "t%x %08x %08x %08x %08x : %04x %04x %04x %04x\r\n", taddr, dist[0],
                        dist[1], dist[2], dist[3], cand_anchor_addr[0], cand_anchor_addr[1], cand_anchor_addr[2],
                        cand_anchor_addr[3]);
            // ͨ�����ڷ�����λ��
            if (u > 0) {
                Usart1_DmaTx(sendbuff, u);
                Usart3_DmaTx(sendbuff, u);
                u = 0;
            }

            // ��ջ�������
            memset(sendbuff, 0, sizeof(sendbuff));
            memset(dist, 0, sizeof(dist));

            // ��������Tag��DS-TWR����
            dist[0] = instance_calc_distance(ToF_ds2[0]);
            dist[1] = instance_calc_distance(ToF_ds2[1]);
            dist[3] = instance_calc_distance(ToF_ds2[2]);
            dist[3] = instance_calc_distance(ToF_ds2[3]);

            // װ������
            u = sprintf((uint8_t *)sendbuff[0], "t%x %08x %08x %08x %08x : %04x %04x %04x %04x\r\n", taddr, dist[0],
                        dist[1], dist[2], dist[3], cand_anchor_addr[0], cand_anchor_addr[1], cand_anchor_addr[2],
                        cand_anchor_addr[3]);
            // ͨ�����ڷ�����λ��
            if (u > 0) {
                Usart1_DmaTx(sendbuff, u);
                Usart3_DmaTx(sendbuff, u);
                u = 0;
            }
            memset(sendbuff, 0, sizeof(sendbuff));
        }
        ins.nextState = TA_INIT; // ������ѯ���������³�ʼ��
    }
}

// TODO: �޸����µļ����ص�����
/**
 * @brief ������ɻص�
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
 * @brief ���������ص�
 */
void twr_rx_cb(const dwt_cb_data_t *rxd) {
    if (mode == TAG) {
        // ��ȡ����
        dwt_readrxdata((uint8_t *)&dw_event.msgu.frame[0], rxd->datalength, 0);
        cb_flag++;

        // ���յ�CAND����RESPONSE������Դ��ַ
        if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == CAND ||
            dw_event.msgu.rxmsg_ss.messageData[FCODE] == RESPONSE) {
            addr16_t = dw_event.msgu.rxmsg_ss.sourceAddr[0] + (dw_event.msgu.rxmsg_ss.sourceAddr[1] << 8);
            aaddr = addr8_t = addr16_t % MAX_RESP_ANCHOR_NUM;
        }
    } else if (mode == ANCHOR) {
        // ��ȡ����
        dwt_readrxdata((uint8_t *)&dw_event.msgu.frame[0], rxd->datalength, 0);
        cb_flag++;

        // ���յ�POLL����GRANT������Դ��ַ
        if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == POLL || dw_event.msgu.rxmsg_ss.messageData[FCODE] == GRANT) {
            addr16_t = dw_event.msgu.rxmsg_ss.sourceAddr[0] + (dw_event.msgu.rxmsg_ss.sourceAddr[1] << 8);
            taddr = addr16_t;
        }
    }
}

/**
 * @brief ���ճ�ʱ�ص�
 */
void twr_rx_timeout_cb(const dwt_cb_data_t *rxd) {
    if (mode == TAG) {
        Tagrxtime = 0;
        if (ins.previousState == TA_RX_WAIT_CAND) {
            if (cand_valid_num > 0) {
                ins.nextState = TA_FEEDBACK_DATA;
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
 * @brief �����쳣�ص�
 */
void twr_rx_error_cb(const dwt_cb_data_t *rxd) {
    cb_flag++;
    dwt_forcetrxoff();

    ins.previousState = ins.testAppState;
    ins.nextState = TA_INIT;
}
/*!
   ------------------------------------------------------------------------------------------------------------------
                ��ȡTXʱ���
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
                ��ȡRXʱ���
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
/*!
   ------------------------------------------------------------------------------------------------------------------
                final����������
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
                final�����ݶ�ȡ
 */
static void msg_get_ts(const uint8 *ts_field, uint64 *ts) {
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

/**
 * @Pazfic: ��ȡϵͳʱ�䣬��λΪDW_ticks
 */
static uint64_t get_sys_ts_u64(void) {
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
