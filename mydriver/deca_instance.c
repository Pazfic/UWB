#include "main.h"
#include "instance.h"
#include "dwt_common.h"
#include <stdio.h>
#include "tim.h"

#define MAXTAGNUM 128  // ��������ǩ����
#define MINTAGSLOT 10  // �����ǩ���ʱ��
uint8_t mode = 0;	   // ��ǩ��վģʽ
uint8_t sendbuff[300]; // ����buff
uint8_t index2 = 0;	   // ��ǩ�յ���վresponse��ʱ������λ��
uint8_t Tarecrx = 0;   // ��ǩģʽ
uint8_t faild_rxtime = 0;
uint16_t addr8_t = 0;	   // ��վ�̵�ַ
uint16_t addr16_t = 0;	   // ��վ����ַ
uint16_t Anch_addr = 1000; // ��վ��ַ
int MAX_TAG_SIZE = 64;	   // ��ǩ����
int N_Slot = 64;		   // ʱ��Ƭ����
int T_Slot = 20;		   // 1��ʱ��Ƭ��ʱ�� ms
int T_Round = 64 * 20;	   // ��ѯ����ʱ��
int GetT_ID = 0;		   // ��ǩID
int rangnum = 0;		   // ���������ϼ�
int POLL_Value = 0;		   // Ԥ�����
int aaddr, taddr;		   // ��վ��ǩ��ַ
int contrlsendms = 10;	   // ���Ʒ���ʱ��
int contrlbit = 0;		   // ���Ʒ���λ
int POLL_count = 0;		   // poll����++
int responsenum = 0;	   // ��ǩ���յ�response������
int lastwaittime = 0;	   // ��ǩӦ�ȴ�ʱ��
int twl_address_index = 0;

// @Pazfic: �洢���յ���response��Դ��ַ
uint16_t response_src_address[MAX_ANCHOR_LIST_SIZE] = {0};
// @Pazfic: ���Anchor���
uint8_t anchor_group_id = 0x70;

double distance[MAX_ANCHOR_LIST_SIZE] = {0};	   // ��ž���
static uint64_t poll_tx_tag = 0;				   // ����pollʱ��
static uint64_t resp_rx_tag[MAX_ANCHOR_LIST_SIZE]; // ����response ʱ��
static uint64_t final_tx_tag = 0;				   // ����final ʱ��
// uint32 Response_tx_time=0;
uint64_t poll_rx_ts = 0;  // ����pollʱ��
uint64_t resp_tx_ts = 0;  // ����response ʱ��
uint64_t final_rx_ts = 0; // ����finalʱ��

event_data_t event;
event_data_t dw_event;
instance_data_t ins;

static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);
static void msg_set_ts(uint8 *ts_field, uint64 ts);
static void msg_get_ts(const uint8 *ts_field, uint32 *ts);
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

void instance_calcranges(uint32_t *array, uint16_t size, uint8_t *mask) // ��tofת��Ϊ���뵥λΪm
{
	int i;
	int dis;
	uint32_t array1[MAX_ANCHOR_LIST_SIZE];
	double tofx = 0;
	for (i = 0; i < size; i++)
	{
		array1[i] = array[i];
		tofx = (array1[i]) * DWT_TIME_UNITS;
		dis = tofx * ((float)SPEED_OF_LIGHT) * 1000;
		if ((dis > 2000000.0) || (dis <= 0))
		{
			// clear mask
			*mask &= ~(0x1 << i);
			distance[i] = 0;
		}
		else if ((dis < 2000000.0) && (dis > 0))
			distance[i] = dis;
		dis = 0;
	}
}

extern int Get_tag_inf;

void testappruns(int modes)
{
	int ret_T = 0;
	int ret_A = 0;
	int u = 0;
	switch (ins.nextState)
	{
	case TA_INIT: // ��ʼ��
	{

		switch (modes)
		{

		case ANCHOR:
		{

			aaddr = (ins.instanceAddress16) & 0xFF;
			memcpy(ins.eui64, &ins.instanceAddress16, ADDR_BYTE_SIZE_S);
			dwt_seteui(ins.eui64);

			dwt_setpanid(ins.panID);
			ins.previousState = TA_INIT;
			ins.testAppState = TA_INIT;
			ins.nextState = TA_RXE_WAIT;
			// report_result=0;
			Anch_RecNt = 1;
			ins.wait4ack = DWT_START_RX_IMMEDIATE;
			mode = ANCHOR;
			break;
		}

		case TAG:
		{
			taddr = sys_para.uwbid;
			memcpy(ins.eui64, &ins.instanceAddress16, ADDR_BYTE_SIZE_S);
			dwt_seteui(ins.eui64);
			dwt_setpanid(ins.panID);
			ins.previousState = TA_INIT;
			ins.testAppState = TA_INIT;

			if (waittime > 1)
			{
				ins.nextState = TA_SLEEP;
			}
			else if (waittime == 0)
			{

				if ((Tarecrx == 1) && (responsenum == 0)) // ��һ����ѯû���յ��ظ���1000ms����һ��
				{

					ins.nextState = TA_SLEEP;
					if (lastwaittime != 0 && faild_rxtime < 10)
					{
						waittime = lastwaittime;
						faild_rxtime++;
					}
					else if (faild_rxtime >= 10 || lastwaittime == 0)
					{
						lastwaittime = waittime = 1000;
						faild_rxtime = 0;
					}
				}
				else if (Tarecrx == 2) // ��һ����ѯ���յ��ظ�
				{
					ins.nextState = TA_TXE_WAIT;
				}
				else if (Tarecrx == 0) // ������һ�η��͵ȴ�ʱ��
				{
					waittime = T_ID * 200;
					ins.nextState = TA_SLEEP;
				}
				mode = TAG;
			}
			break;
		}
		}
		break;
	}

	case TA_SLEEP: // �ȴ�
	{

		ins.previousState = TA_INIT;
		ins.testAppState = TA_SLEEP;

		break;
	}
	case TA_SLEEP_DONE: // ʱ�䵽���뷢��
	{
		ins.nextState = TA_TXE_WAIT;
		ins.previousState = TA_SLEEP;
		ins.testAppState = TA_SLEEP_DONE;
		break;
	}
	case TA_TXE_WAIT: // �ȴ�����
	{
		ins.previousState = TA_INIT;
		ins.testAppState = TA_TXE_WAIT;
		if (waittime <= 0)
		{
			ins.nextState = TA_TXPOLL_WAIT_SEND;
		}
		break;
	}
	case TA_TXPOLL_WAIT_SEND: // ����poll���Ͳ���������
	{
		uint16_t frameLength = 0;
		frameLength = TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC; // ����poll������
		ins.testAppState = TA_TXPOLL_WAIT_SEND;
		ins.previousState = TA_TXE_WAIT;
		ins.msg_f.seqNum = ins.frameSN++;
		ins.msg_f.sourceAddr[0] = sys_para.uwbid & 0xff;		// ����ID��Դ��ַ
		ins.msg_f.sourceAddr[1] = (sys_para.uwbid >> 8) & 0xff; //
		ins.msg_f.destAddr[0] = 0xff;							// ���ù㲥
		ins.msg_f.destAddr[1] = 0xff;
		ins.msg_f.messageData[POLL_RNUM] = ins.rangeNum++;
		ins.msg_f.messageData[FCODE] = 0x81;
		//			ins.msg_f.messageData[BATTERY_VALUE0]=ADC_Value&0xff;
		//			ins.msg_f.messageData[BATTERY_VALUE1]=(ADC_Value>>8)&0xff;
		//			ins.msg_f.messageData[SOS_CODE]=ins.sos_state;
		//			ins.sos_state=0;
		ins.responseTO = MAX_ANCHOR_LIST_SIZE;
		dwt_setrxaftertxdelay(20);				 // ���÷��ͺ�20us��������
		dwt_setrxtimeout(1100 * ins.responseTO); // ���ý��ճ�ʱʱ��

		dwt_writetxdata(frameLength, (uint8_t *)&(ins.msg_f), 0);			 // ���poll����
		dwt_writetxfctrl(frameLength, 0, 1);								 // ���Ʒ���
		ret_T = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED); // �����������Ͳ��ӳٽ���
		if (ret_T == DWT_SUCCESS)											 // �ж��Ƿ��ͳɹ�
		{
			ins.nextState = TA_TX_WAIT_CONF;
			Tarecrx = 1;
			POLL_count++;
			responsenum = 0;

			// @Pazfic: ���responseԴ��ַ����
			memset(response_src_address, 0, sizeof(response_src_address));
		}
		break;
	}
	case TA_TXRESPONSE_WAIT_SEND: // 4
	{
		uint16_t frameLength = 0;
		int calcwaittime = 0;
		dwt_setrxaftertxdelay(20); // ���÷��ͺ�~100us��������

		frameLength = ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC; // ����response������

		calcwaittime = ((T_Round + (twl_address_index * T_Slot)) * 10 - Anchpollingtime - 20); // �����ǩ�´η���ʱ��
		if ((calcwaittime > 0) && (calcwaittime < ((T_Round + (twl_address_index * T_Slot)) * 10)))
		{
			ins.msg_f.messageData[RES_TAG_SLP0] = calcwaittime & 0xFF; // ��ʱ��װ�ؽ�response��
			ins.msg_f.messageData[RES_TAG_SLP1] = (calcwaittime >> 8) & 0xFF;
		}
		else
		{
			ins.msg_f.messageData[RES_TAG_SLP0] = 0;
			ins.msg_f.messageData[RES_TAG_SLP1] = 0;
		}

		ins.msg_f.messageData[CRC_BIT_RES] = 0;
		for (int r = 0; r < 10; r++)
			ins.msg_f.messageData[CRC_BIT_RES] += ins.msg_f.messageData[r];

		ins.msg_f.messageData[CRC_BIT_RES] = (~ins.msg_f.messageData[CRC_BIT_RES]) + 1; // ����crcλ
		dwt_writetxdata(frameLength, (uint8_t *)&(ins.msg_f), 0);						// ���response����
		dwt_writetxfctrl(frameLength, 0, 1);											// ���Ʒ���
		ret_A = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);			// �����������Ͳ��ӳٽ���
		if (ret_A == DWT_SUCCESS)														// �ж��Ƿ��ͳɹ�
		{
			ins.nextState = TA_TX_WAIT_CONF;
		}
		else
		{

			dwt_forcetrxoff();
			ins.testAppState = TA_INIT;
			ins.nextState = ins.testAppState;
			ins.previousState = ins.testAppState;
		}

		break;
	}
	case TA_TXFINAL_WAIT_SEND:
	{

		uint16_t frameLength = 0;
		uint32_t final_tx_time;

		ins.testAppState = TA_TXFINAL_WAIT_SEND;
		ins.previousState = TA_RX_WAIT_DATA;
		poll_tx_tag = get_tx_timestamp_u64(); // ��ȡpoll����ʱ��
		final_tx_time = (poll_tx_tag + (6000 * UUS_TO_DWT_TIME)) >> 8;

		dwt_setdelayedtrxtime(final_tx_time); // ����final����ʱ��

		final_tx_tag = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY; // ��ȡfinal����ʱ��+�����ӳ�

		ins.msg_f.sourceAddr[0] = sys_para.uwbid & 0xff; // ����ID��Դ��ַ
		ins.msg_f.sourceAddr[1] = (sys_para.uwbid >> 8) & 0xff;
		ins.msg_f.destAddr[0] = 0xff; // ���ù㲥
		ins.msg_f.destAddr[1] = 0xff;
		ins.msg_f.messageData[POLL_RNUM] = ins.rangeNum;
		ins.msg_f.seqNum = ins.frameSN = dw_event.msgu.rxmsg_ss.seqNum++;
		msg_set_ts(&ins.msg_f.messageData[PTXT], poll_tx_tag);	// װ��poll����ʱ��
		msg_set_ts(&ins.msg_f.messageData[FTXT], final_tx_tag); // װ��response����ʱ��

		ins.msg_f.messageData[FCODE] = 0x82;
		memset(resp_rx_tag, 0, sizeof(resp_rx_tag));
		frameLength = TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC; // ����final������

		dwt_writetxdata(frameLength, (uint8_t *)&(ins.msg_f), 0);
		dwt_writetxfctrl(frameLength, 0, 1);

		ret_T = dwt_starttx(DWT_START_TX_DELAYED); // �����ӳٷ���

		if (ret_T == DWT_SUCCESS)
		{

			Tag_RecNt = 1;
			ins.nextState = TA_TX_WAIT_CONF;
		}
		else
		{
			dwt_forcetrxoff();
			// dwt_rxreset();
			ins.testAppState = TA_INIT;
			ins.nextState = ins.testAppState;
			ins.previousState = ins.testAppState;
		}

		break;
	}
	case TA_TX_WAIT_CONF: // 6
	{
		if (mode == TAG)
		{
			if (ins.testAppState == TA_TXPOLL_WAIT_SEND)
				ins.previousState = TA_TXPOLL_WAIT_SEND;

			if (ins.testAppState == TA_TXFINAL_WAIT_SEND)
			{
				int valid = 0;
				int rangeTime = 0;
				int l;
				ins.previousState = TA_TXFINAL_WAIT_SEND;
				instance_calcranges(&ins.Tof[0], MAX_ANCHOR_LIST_SIZE, &ins.rxResponseMask);
				memset(ins.Tof, 0, sizeof(ins.Tof));

				valid = ins.rxResponseMask; // ��ȡ���յ���վ����
				l = (ins.longTermRangeCount++) & 0xFFFF;
				rangeTime = HAL_GetTick() & 0xffffffff; // ��ȡsystickʱ��

				if (ins.rxResponseMask > 0)
				{

					u = sprintf((char *)&sendbuff[0], "mc %02x %08x %08x %08x %08x %04x %02x %08x %c%x: %04x %04x %04x 	%04x\r\n",
								valid, (int)((distance[0] * 0.9335) - 67.8345),
								(int)((distance[1] * 0.9335) - 67.8345),
								(int)((distance[2] * 0.9335) - 67.8345),
								(int)((distance[3] * 0.9335) - 67.8345),
								l, ins.rangeNum, rangeTime, 't', taddr,
								response_src_address[0], response_src_address[1],
								response_src_address[2], response_src_address[3]); // ������
				}

				ins.rxResponseMask = addr8_t = addr16_t = 0;
				if (u > 0)
				{

					Usart1_DmaTx(sendbuff, u); // �������ݰ�
					Usart3_DmaTx(sendbuff, u);
					u = 0;
				}

				memset(distance, 0, sizeof(distance));
			}
			ins.testAppState = TA_TX_WAIT_CONF;
		}
		else
		{
			ins.previousState = TA_TXRESPONSE_WAIT_SEND;
			ins.testAppState = TA_TX_WAIT_CONF;
		}

		break;
	}

	case TA_RXE_WAIT: // 7
	{
		if (mode == TAG)
		{
			ins.previousState = TA_TX_WAIT_CONF;
			ins.testAppState = TA_RXE_WAIT;
			ins.nextState = TA_RX_WAIT_DATA;
		}
		else
		{
			ins.testAppState = TA_RXE_WAIT;

			if (ins.previousState == TA_INIT)
			{
				if ((2 * T_Slot * 1100) > 65535) // �����ճ�ʱʱ�����65535
					dwt_setrxtimeout(65535);
				else
					dwt_setrxtimeout(2 * T_Slot * 1100);
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
			}
			ins.nextState = TA_RX_WAIT_DATA;
		}
	}

	case TA_RX_WAIT_DATA: // 8
	{
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (mode == TAG)
		{
			if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0)
				Tag_RecNt = 1;																				  //
			else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0x70 /* &&((addr16_t&0x8000) == 0x8000) */) // �ж�response
			{
				uint8_t crc_jy = 0;
				uint8_t Anch_num = 0;
				int r = 0;
				for (r = 0; r < 10; r++)
					crc_jy += dw_event.msgu.rxmsg_ss.messageData[r];

				crc_jy = (~crc_jy) + 1; // ����crc
				Tagrxtime = 0;
				if (dw_event.msgu.rxmsg_ss.messageData[CRC_BIT_RES] == crc_jy) // У��crc
				{
					if (T_ID == dw_event.msgu.rxmsg_ss.destAddr[0] + (dw_event.msgu.rxmsg_ss.destAddr[1] << 8)) // �ж�ID�Ƿ�Ϊ�Լ�
					{
						responsenum++;
						Tag_RecNt = 0;

						Tarecrx = 2;
						resp_rx_tag[addr8_t] = get_rx_timestamp_u64(); // ��ȡ��Ӧ��վλ��response����ʱ��
						if (responsenum == 1)						   // ���յ���һ����վ
						{
							for (Anch_num = 0; Anch_num < MAX_ANCHOR_LIST_SIZE; Anch_num++)
							{
								if (resp_rx_tag[Anch_num] > 0)
								{
									Anch_addr = Anch_num;
									break;
								}
							}
						}
						//							if(addr16_t == (0x8000|Anch_addr))//���ջ�վ�ĵȴ�ʱ��
						//							{
						//
						//								lastwaittime=waittime=dw_event.msgu.rxmsg_ss.messageData[RES_TAG_SLP0]+(dw_event.msgu.rxmsg_ss.messageData[RES_TAG_SLP1]<<8);
						//
						//							}
						// @Pazfic:
						if ((addr16_t & 0x00FF) == Anch_addr)
						{
							lastwaittime = waittime = dw_event.msgu.rxmsg_ss.messageData[RES_TAG_SLP0] + (dw_event.msgu.rxmsg_ss.messageData[RES_TAG_SLP1] << 8);
						}

						POLL_count = 0;

						if ((ins.responseTO - addr8_t - 1) > 0) // ������һ��վ�Ľ��ճ�ʱʱ��
						{
							dwt_setrxtimeout(1100 * (ins.responseTO - addr8_t - 1));

							dwt_rxenable(DWT_START_RX_IMMEDIATE);
							ins.nextState = TA_RX_WAIT_DATA;
						}
						else if ((ins.responseTO - addr8_t - 1) == 0)
						{
							ins.nextState = TA_TXFINAL_WAIT_SEND;
							dwt_forcetrxoff();
						}
						memcpy(&(ins.Tof[addr8_t]), &(dw_event.msgu.rxmsg_ss.messageData[TOFR]), 4); // ���ջ�վ��Ӧλ��tof
						msg_set_ts(&ins.msg_f.messageData[index2], resp_rx_tag[addr8_t]);			 // ��response����ʱ�������Ӧfinal��λ
					}
					else
					{
						ins.nextState = TA_INIT;
					}
				}
				else
				{
					ins.nextState = TA_INIT;
				}
				dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
				Tag_RecNt = 1;
			}
			else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] != 0x70) // ���յ���������¿�ʼ
			{
				Tag_RecNt = 1;
				ins.nextState = TA_INIT;
				dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
			}

			ins.previousState = TA_RXE_WAIT;
			ins.testAppState = TA_RX_WAIT_DATA;
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		else
		{

			if ((Sendcontrltime - contrlsendms >= 0) && (contrlbit == 1))
			{
				contrlbit = 0;
				contrlsendms = 0;

				dwt_forcetrxoff();

				dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;
				ins.nextState = TA_TXRESPONSE_WAIT_SEND;
			}
			if ((Anchpollingtime > 0) && (Anchpollingtime < 20) && (Get_tag_inf == 1))
			{

				uint8_t retxx = 0;
				uint16_t frameLength = 0;
				int r = 0;
				dwt_forcetrxoff();
				ins.msg_f.messageData[MAX_TAG_NUM_L] = sys_para.TAG_NUM & 0xFF;
				ins.msg_f.messageData[MAX_TAG_NUM_H] = (sys_para.TAG_NUM >> 8) & 0xFF;
				ins.msg_f.messageData[TAG_SLOTTIME_L] = sys_para.TAG_SLOT & 0xFF;
				ins.msg_f.messageData[TAG_SLOTTIME_H] = (sys_para.TAG_SLOT >> 8) & 0xFF;
				ins.msg_f.messageData[FCODE] = 0x69;
				ins.msg_f.messageData[CRC_BIT] = 0;
				for (r = 0; r < 6; r++)
					ins.msg_f.messageData[CRC_BIT] += ins.msg_f.messageData[r];
				ins.msg_f.messageData[CRC_BIT] = (~ins.msg_f.messageData[CRC_BIT]) + 1; // ����crc

				frameLength = 7 + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;

				dwt_writetxdata(frameLength, (uint8_t *)&(ins.msg_f), 0);
				dwt_writetxfctrl(frameLength, 0, 1);

				retxx = dwt_starttx(DWT_START_TX_IMMEDIATE);
				HAL_Delay(10);
				retxx = dwt_starttx(DWT_START_TX_IMMEDIATE);
				if (retxx == DWT_SUCCESS)
				{
					HAL_Delay(10);
					NVIC_SystemReset();
				}
			}
			if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0)
			{
				Anch_RecNt = 1;
			}

			else
			{
				Anch_RecNt = 0;
				if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0x81) // �ж�poll��
				{
					int r = 0;
					poll_rx_ts = get_rx_timestamp_u64();
					Anch_RecNt = 0;
					twl_address_index = 0;
					taddr = GetT_ID = dw_event.msgu.rxmsg_ss.sourceAddr[0] + (dw_event.msgu.rxmsg_ss.sourceAddr[1] << 8);
					if (sys_para.twlt == 0x0000)
					{
						if ((0 < GetT_ID) && (GetT_ID <= N_Slot))
							twl_address_index = GetT_ID;
						else
							twl_address_index = 0;
					}
					else if (sys_para.twlt == 0x0001)
					{
						for (r = 0; r < 4; r++)
						{
							if (GetT_ID == twl_address[r])
							{
								twl_address_index = r + 1;
								break;
							}
						}
					}

					if (twl_address_index > 0)
					{
						rangnum = ins.rangeNum = dw_event.msgu.rxmsg_ss.messageData[POLL_RNUM];

						// POLL_Value=dw_event.msgu.rxmsg_ss.messageData[BATTERY_VALUE0]+(dw_event.msgu.rxmsg_ss.messageData[BATTERY_VALUE1]<<8);

						// fill in respones message
						ins.msg_f.seqNum = dw_event.msgu.rxmsg_ss.seqNum++;

						memcpy(&(ins.msg_f.destAddr[0]), &(dw_event.msgu.rxmsg_ss.sourceAddr[0]), ADDR_BYTE_SIZE_S);
						memcpy(&(ins.msg_f.messageData[TOFR]), &ins.TOF[twl_address_index], 4);
						ins.msg_f.sourceAddr[0] = ins.eui64[0];
						ins.msg_f.sourceAddr[1] = ins.eui64[1];

						ins.msg_f.messageData[FCODE] = 0x70;

						//							if((ins.instanceAddress16 &0x8000)== 0x8000)//
						//							{
						//								if(ins.instanceAddress16 == 0x8000)
						//								{
						//									contrlsendms=3;
						//								}
						//								else
						//								{
						//									contrlsendms=aaddr*10+3;
						//								}
						//								contrlbit =1;
						//
						//								dwt_forcetrxoff();
						//								if((ins.instanceAddress16) != 0x8000)
						//								{
						//
						//									//dwt_setrxtimeout(T_Slot*1000);
						//									dwt_rxenable(DWT_START_RX_IMMEDIATE);
						//								}
						//								Sendcontrltime=0;
						//								ins.nextState=TA_RX_WAIT_DATA;

						// @Pazfic
						if ((ins.instanceAddress16 & 0x00FF) == 0x0000)
						{
							contrlsendms = 3;
						}
						else
						{
							contrlsendms = aaddr * 10 + 3;
						}
						contrlbit = 1;
						dwt_forcetrxoff();
						if ((ins.instanceAddress16 & 0x00FF) != 0x0000)
						{
							dwt_rxenable(DWT_START_RX_IMMEDIATE);
						}
						Sendcontrltime = 0;
						ins.nextState = TA_RX_WAIT_DATA;
					}
					else if (twl_address_index == 0)
						ins.nextState = TA_INIT;

					// dw_event.msgu.rxmsg_ss.messageData[FCODE] =0;
				}
				else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0x70) // �ж�response��
				{
					uint8_t crc_jy = 0;
					int r = 0;
					for (r = 0; r < 10; r++)
						crc_jy += dw_event.msgu.rxmsg_ss.messageData[r];

					crc_jy = (~crc_jy) + 1;
					if (dw_event.msgu.rxmsg_ss.messageData[CRC_BIT_RES] == crc_jy)
					{
						memcpy(&(ins.TOFArray[dw_event.msgu.rxmsg_ss.sourceAddr[0]]), &(dw_event.msgu.rxmsg_ss.messageData[TOFR]), 4);
						Anch_RecNt = 0;
						ins.rxResponseMask |= (0x1 << (dw_event.msgu.rxmsg_ss.sourceAddr[0]));

						ins.nextState = TA_RX_WAIT_DATA;
						if ((aaddr - (dw_event.msgu.rxmsg_ss.sourceAddr[0])) == 1)
						{
							if (contrlbit != 1)
							{
								// dwt_setrxtimeout(T_Slot*1000);
								dwt_rxenable(DWT_START_RX_IMMEDIATE);
							}
						}
						else
						{
							// dwt_setrxtimeout(T_Slot*1000);
							dwt_rxenable(DWT_START_RX_IMMEDIATE);
						}
					}
					else
						ins.nextState = TA_INIT;
				}
				else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0x82) // �ж�final��
				{
					uint64_t tof_dtu;
					uint32 poll_tx_ts = 0, resp_rx_ts = 0, final_tx_ts = 0;

					uint32 resp_tx_ts32 = 0, poll_rx_ts32 = 0, final_rx_ts32 = 0;
					int valid = 0;
					int rangeTime = 0;
					double Ra, Rb, Da, Db;
					int l = 0;

					uint8_t index = 0;

					if (GetT_ID == (dw_event.msgu.rxmsg_ss.sourceAddr[0] + (dw_event.msgu.rxmsg_ss.sourceAddr[1] << 8)))
					{
						index = RRXT0 + 5 * aaddr;
						Anch_RecNt = 0;
						ins.nextState = TA_INIT;
						// GetT_ID=taddr=dw_event.msgu.rxmsg_ss.sourceAddr[0]+(dw_event.msgu.rxmsg_ss.sourceAddr[1]<<8);

						resp_tx_ts = get_tx_timestamp_u64();
						final_rx_ts = get_rx_timestamp_u64(); //???final???????T6

						msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[PTXT], &poll_tx_ts); //??????????��??T1??T4??T5
						msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[index], &resp_rx_ts);
						msg_get_ts(&dw_event.msgu.rxmsg_ss.messageData[FTXT], &final_tx_ts);

						resp_tx_ts32 = (uint32_t)resp_tx_ts;
						poll_rx_ts32 = (uint32_t)poll_rx_ts;
						final_rx_ts32 = (uint32_t)final_rx_ts;

						Ra = (double)(resp_rx_ts - poll_tx_ts);							// Tround1 = T4 - T1
						Rb = (double)(final_rx_ts32 - resp_tx_ts32);					// Tround2 = T6 - T3
						Da = (double)(final_tx_ts - resp_rx_ts);						// Treply2 = T5 - T4
						Db = (double)(resp_tx_ts32 - poll_rx_ts32);						// Treply1 = T3 - T2
						tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db)); //?????
						ins.TOFArray[(ins.instanceAddress16) & 0xff] = tof_dtu;
						instance_calcranges(&ins.TOFArray[0], MAX_ANCHOR_LIST_SIZE, &ins.rxResponseMask);
						memset(ins.TOFArray, 0, sizeof(ins.TOFArray));
						//								memset(sendbuff,0,sizeof(sendbuff));

						if (distance[aaddr] > 0)
						{
							ins.TOF[twl_address_index] = tof_dtu;
						}
						else
							distance[aaddr] = 0;

						valid = ins.rxResponseMask;
						l = (ins.longTermRangeCount++) & 0xFFFF;
						;
						rangeTime = HAL_GetTick() & 0xffffffff;

						if (ins.rxResponseMask > 0)
						{

							//									u = sprintf((char*)&sendbuff[0], "mc %02x %08x %08x %08x %08x %04x %02x %08x %c%x:%x\r\n",
							//																										valid, (int)distance[0],(int)distance[1],(int)distance[2],(int)distance[3]
							//																											,l, ins.rangeNum, rangeTime, 'a', taddr, aaddr);//�����ݰ�

							u = sprintf((char *)&sendbuff[0], "mc %02x %08x %08x %08x %08x %04x %02x %08x %c%x:%x\r\n",
										valid, (int)((distance[0] * 0.9335) - 67.8345),
										(int)((distance[1] * 0.9335) - 67.8345),
										(int)((distance[2] * 0.9335) - 67.8345),
										(int)((distance[3] * 0.9335) - 67.8345),
										l, ins.rangeNum, rangeTime, 'a', taddr, aaddr); // ������
						}
						ins.rxResponseMask = 0;
						ins.rxResponseMask = (0x1 << (aaddr));
						memset(distance, 0, sizeof(distance));

						if (u > 0)
						{
							Usart3_DmaTx(sendbuff, u);
							Usart1_DmaTx(sendbuff, u);

							u = 0;
						}

						memset(distance, 0, sizeof(distance));
					}
					else
					{
						dwt_rxenable(DWT_START_RX_IMMEDIATE);
					}
					ins.wait4ack = DWT_START_RX_IMMEDIATE;
					dwt_setrxtimeout(0);
					// dw_event.msgu.rxmsg_ss.messageData[FCODE] =0;
				}
				else if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0x69) // �ж�sped��
				{
					uint8_t crc_jy = 0;

					int r = 0;
					for (r = 0; r < 6; r++)
						crc_jy += dw_event.msgu.rxmsg_ss.messageData[r];
					crc_jy = (~crc_jy) + 1;
					if (dw_event.msgu.rxmsg_ss.messageData[CRC_BIT] == crc_jy)
					{
						FLAH_BUFF0[14] = (dw_event.msgu.rxmsg_ss.messageData[MAX_TAG_NUM_H] << 8) + dw_event.msgu.rxmsg_ss.messageData[MAX_TAG_NUM_L];
						FLAH_BUFF0[15] = (dw_event.msgu.rxmsg_ss.messageData[TAG_SLOTTIME_H] << 8) + dw_event.msgu.rxmsg_ss.messageData[TAG_SLOTTIME_L];
						if ((MAXTAGNUM >= FLAH_BUFF0[14]) && (MINTAGSLOT <= FLAH_BUFF0[15]))
						{
							app_flash_write(); // ��??Flash
							HAL_Delay(10);
							NVIC_SystemReset(); //??��
						}
					}
					else
						ins.nextState = TA_INIT;
				}
				else
					ins.nextState = TA_INIT;

				ins.previousState = TA_RXE_WAIT;
				ins.testAppState = TA_RX_WAIT_DATA;
				dw_event.msgu.rxmsg_ss.messageData[FCODE] = 0;

				Anch_RecNt = 1;
				Anchrxtime = 0;
			}
		}
		break;
	}
	default:
		break;
	}
}

// dwt_setcallbacks(twr_tx_tag_cb, twr_rx_cb, twr_rx_timeout_cb, twr_rx_error_cb);
void twr_tx_tag_cb(const dwt_cb_data_t *txd)
{
	if (mode == TAG)
	{
		if (ins.previousState == TA_TXPOLL_WAIT_SEND)
			ins.nextState = TA_RXE_WAIT;
		else if (ins.previousState == TA_TXFINAL_WAIT_SEND)
		{

			ins.nextState = TA_INIT;
		}
		else
		{
			twr_rx_error_cb(txd);
		}
	}
	else
	{
		if (ins.testAppState == TA_TX_WAIT_CONF)
		{
			ins.nextState = TA_RXE_WAIT;
		}
	}
}

void twr_rx_cb(const dwt_cb_data_t *rxd)
{
	if (mode == TAG)
	{

		dwt_readrxdata((uint8_t *)&dw_event.msgu.frame[0], rxd->datalength, 0); //???????????
		if (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0x70)
		{
			addr16_t = dw_event.msgu.rxmsg_ss.sourceAddr[0] + (dw_event.msgu.rxmsg_ss.sourceAddr[1] << 8);
			aaddr = addr8_t = addr16_t & 0xff;

			response_src_address[aaddr] = addr16_t;

			index2 = RRXT0 + 5 * addr8_t;
			ins.rxResponseMask |= (0x1 << (addr8_t));
		}

		if ((dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0x81) || (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0x82) || (dw_event.msgu.rxmsg_ss.messageData[FCODE] == 0x69))
		{
			dwt_forcetrxoff();
			ins.nextState = TA_INIT;
		}
	}
	else
	{
		dwt_readrxdata((uint8_t *)&dw_event.msgu.frame[0], rxd->datalength, 0); // Read Data Frame

		if ((dw_event.msgu.rxmsg_ss.messageData[FCODE] != 0x81) && (dw_event.msgu.rxmsg_ss.messageData[FCODE] != 0x70) &&
			(dw_event.msgu.rxmsg_ss.messageData[FCODE] != 0x82) && (dw_event.msgu.rxmsg_ss.messageData[FCODE] != 0x69))
		{
			dwt_forcetrxoff();
			// dwt_rxreset();

			ins.nextState = TA_INIT;
		}
	}
}

void twr_rx_timeout_cb(const dwt_cb_data_t *rxd)
{
	if (mode == TAG)
	{
		Tagrxtime = 0;

		if (responsenum > 0)
			ins.nextState = TA_TXFINAL_WAIT_SEND;
		else if (responsenum == 0)
			ins.nextState = TA_INIT;
	}
	else
	{
		//		dwt_rxreset();
		Anchrxtime = 0;

		//	dwt_forcetrxoff();
		ins.testAppState = TA_INIT;
		ins.nextState = TA_INIT;
		ins.previousState = TA_INIT;
	}
}
void twr_rx_error_cb(const dwt_cb_data_t *rxd)
{
	dwt_forcetrxoff();
	// dwt_rxreset();
	ins.testAppState = TA_INIT;
	ins.nextState = ins.testAppState;
	ins.previousState = ins.testAppState;
}
/*! ------------------------------------------------------------------------------------------------------------------
		��ȡTXʱ���
 */
static uint64_t get_tx_timestamp_u64(void)
{
	uint8 ts_tab[5];
	uint64_t ts = 0;
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
		��ȡRXʱ���
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
		final����������
 */
static void msg_set_ts(uint8 *ts_field, uint64 ts)
{
	int i;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++)
	{
		ts_field[i] = (uint8)ts;
		ts >>= 8;
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
		final�����ݶ�ȡ
 */
static void msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
	int i;
	*ts = 0;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++)
	{
		*ts += ts_field[i] << (i * 8);
	}
}
