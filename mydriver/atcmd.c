#include "atcmd.h"

#include "instance.h"
#include "stdio.h"
#include "string.h"


uint8 Sendbuff[400];
uint8 n = 0; // Sendbuff index

void Load_Set_from_U(uint8 *RXbuff);
void Load_Set_from_P(uint8 *RXbuff);
void Load_SetTCP(uint8 *RXbuff);
void Load_tagnum(uint8 *RXbuff);

void Display_Searching(void) {
    int n = 0;
    n += sprintf((char *)&Sendbuff[n], ">> Searching Anchors & Tags...\r\n");
    Usart3_DmaTx(Sendbuff, n);
    Usart1_DmaTx(Sendbuff, n);
}

void Display_ATCMDInfo(void) {
    int n = 0;
    n = sprintf((char *)&Sendbuff[0], ">> Please configure the module...:AT+QSET\r\n");
    Usart3_DmaTx(Sendbuff, n);
    Usart1_DmaTx(Sendbuff, n);
}

uint8 Check_cmd(uint8 *RXbuff) {
    //	u8 i=0;
    if (strstr((char *)RXbuff, "AT+ACMD=") != NULL) // 0x01
    {
        if ((((RXbuff[8] <= '9') && (RXbuff[8] >= '0')) || ((RXbuff[8] <= 'F') && (RXbuff[8] >= 'A'))) &&
            (((RXbuff[9] <= '9') && (RXbuff[9] >= '0')) || ((RXbuff[9] <= 'F') && (RXbuff[9] >= 'A'))) &&
            (((RXbuff[10] <= '9') && (RXbuff[10] >= '0')) || ((RXbuff[10] <= 'F') && (RXbuff[10] >= 'A'))) &&
            (((RXbuff[11] <= '9') && (RXbuff[11] >= '0')) || ((RXbuff[11] <= 'F') && (RXbuff[11] >= 'A'))) &&
            (RXbuff[12] == ',') &&
            (((RXbuff[13] <= '9') && (RXbuff[13] >= '0')) || ((RXbuff[13] <= 'F') && (RXbuff[13] >= 'A'))) &&
            (((RXbuff[14] <= '9') && (RXbuff[14] >= '0')) || ((RXbuff[14] <= 'F') && (RXbuff[14] >= 'A'))) &&
            (((RXbuff[15] <= '9') && (RXbuff[15] >= '0')) || ((RXbuff[15] <= 'F') && (RXbuff[15] >= 'A'))) &&
            (((RXbuff[16] <= '9') && (RXbuff[16] >= '0')) || ((RXbuff[16] <= 'F') && (RXbuff[16] >= 'A'))))
            return ACMD;
        else
            return EROR;
    }
    if (strstr((char *)RXbuff, "AT+TCPS=") != NULL) // 0x02
    {
        int t = 0;
        int x = 0;
        for (t = 8; t <= 55; t++) {
            if (((RXbuff[t] >= 'A') && (RXbuff[t] <= 'F')) || ((RXbuff[t] >= '0') && (RXbuff[t] <= '9'))) {
                x++;
            }
        }
        if (x != 48)
            return EROR;
        else if (x == 48)
            return TCPS;
    }
    if (strstr((char *)RXbuff, "AT+SPED=") != NULL) // 0x03
    {
        if ((RXbuff[11] == '-') && (RXbuff[8] <= '9') && (RXbuff[8] >= '0') && (RXbuff[9] <= '9') &&
            (RXbuff[9] >= '0') && (RXbuff[10] <= '9') && (RXbuff[10] >= '0') && (RXbuff[12] <= '9') &&
            (RXbuff[12] >= '0') && (RXbuff[13] <= '9') && (RXbuff[13] >= '0') && (RXbuff[14] <= '9') &&
            (RXbuff[14] >= '0') && (RXbuff[15] <= '9') && (RXbuff[15] >= '0'))
            return SPED;
        else
            return EROR;
    }
    if (strstr((char *)RXbuff, "AT+INF?") != NULL) // 0x04
        return INFO;
    if (strstr((char *)RXbuff, "AT+VBAT?") != NULL) // 0x05
        return VBAT;
    if (strstr((char *)RXbuff, "AT+TWLT=") != NULL) // 0x06
    {
        if (RXbuff[8] == '1') {
            int t1 = 0;
            int x1 = 0;
            for (t1 = 0; t1 < 5; t1++) {
                if ((RXbuff[9 + 5 * t1] == ',') &&
                    (((RXbuff[10 + 5 * t1] <= '9') && (RXbuff[10 + 5 * t1] >= '0')) ||
                     ((RXbuff[10 + 5 * t1] <= 'F') && (RXbuff[10 + 5 * t1] >= 'A'))) &&
                    (((RXbuff[11 + 5 * t1] <= '9') && (RXbuff[11 + 5 * t1] >= '0')) ||
                     ((RXbuff[11 + 5 * t1] <= 'F') && (RXbuff[11 + 5 * t1] >= 'A'))) &&
                    (((RXbuff[12 + 5 * t1] <= '9') && (RXbuff[12 + 5 * t1] >= '0')) ||
                     ((RXbuff[12 + 5 * t1] <= 'F') && (RXbuff[12 + 5 * t1] >= 'A'))) &&
                    (((RXbuff[13 + 5 * t1] <= '9') && (RXbuff[13 + 5 * t1] >= '0')) ||
                     ((RXbuff[13 + 5 * t1] <= 'F') && (RXbuff[13 + 5 * t1] >= 'A'))))
                    x1++;
            }
            if (x1 != 5)
                return EROR;
            else if (x1 == 5)
                return TWLT;
        } else if (RXbuff[8] == '0')
            return TWLT;
    }
    if (strstr((char *)RXbuff, "AT+BAUD=") != NULL) // 0x07
        return BAUD;
    if (strstr((char *)RXbuff, "AT+STAR") != NULL) // 0x08
        return STAR;
    if (strstr((char *)RXbuff, "AT+TCPE=") != NULL) // 0x09
        return TCPE;
    if (strstr((char *)RXbuff, "AT+RSET") != NULL) // 0x10
        return RSET;

    if (strstr((char *)RXbuff, "AT+BEEP?") != NULL) return BEEp;
    if (strstr((char *)RXbuff, "AT+BEEP=") != NULL) return BEEP;
    if (strstr((char *)RXbuff, "AT+UCHG=") != NULL) return UCHG;
    if (strstr((char *)RXbuff, "AT+LIAP=") != NULL) return LIAP;
    if (strstr((char *)RXbuff, "AT+UCHG?") != NULL) return UCHg;
    if (strstr((char *)RXbuff, "AT+BAUD?") != NULL) return BAUd;
    if (strstr((char *)RXbuff, "AT+GAIN?") != NULL) return GAIn;

    if (strstr((char *)RXbuff, "AT+GAIN=") != NULL) {
        if ((((RXbuff[8] <= '9') && (RXbuff[8] >= '0')) || ((RXbuff[8] <= 'F') && (RXbuff[8] >= 'A'))) &&
            (((RXbuff[9] <= '9') && (RXbuff[9] >= '0')) || ((RXbuff[9] <= 'F') && (RXbuff[9] >= 'A'))) &&
            (((RXbuff[10] <= '9') && (RXbuff[10] >= '0')) || ((RXbuff[10] <= 'F') && (RXbuff[10] >= 'A'))) &&
            (((RXbuff[11] <= '9') && (RXbuff[11] >= '0')) || ((RXbuff[11] <= 'F') && (RXbuff[11] >= 'A'))) &&
            (((RXbuff[12] <= '9') && (RXbuff[12] >= '0')) || ((RXbuff[12] <= 'F') && (RXbuff[12] >= 'A'))) &&
            (((RXbuff[13] <= '9') && (RXbuff[13] >= '0')) || ((RXbuff[13] <= 'F') && (RXbuff[13] >= 'A'))) &&
            (((RXbuff[14] <= '9') && (RXbuff[14] >= '0')) || ((RXbuff[14] <= 'F') && (RXbuff[14] >= 'A'))) &&
            (((RXbuff[15] <= '9') && (RXbuff[15] >= '0')) || ((RXbuff[15] <= 'F') && (RXbuff[15] >= 'A'))))
            return GAIN;
        else
            return EROR;
    }

    if (strstr((char *)RXbuff, "AT+QSET=") != NULL) {
        //		int i;
        //		printf("%x %x %x",RXbuff[12],RXbuff[13],RXbuff[14]);
        if (RXbuff[8] == 'F' /*|| RXbuff[8] == 'S' || RXbuff[8] == 'M'*/)
            if (RXbuff[9] == '2' || RXbuff[9] == '5')
                if (RXbuff[10] == '-')
                    if (RXbuff[11] == 'A' || RXbuff[11] == 'T') {
                        if ((((RXbuff[12] <= '9') && (RXbuff[12] >= '0')) ||
                             ((RXbuff[12] <= 'F') && (RXbuff[12] >= 'A'))) &&
                            (((RXbuff[13] <= '9') && (RXbuff[13] >= '0')) ||
                             ((RXbuff[13] <= 'F') && (RXbuff[13] >= 'A'))) &&
                            (((RXbuff[14] <= '9') && (RXbuff[14] >= '0')) ||
                             ((RXbuff[14] <= 'F') && (RXbuff[14] >= 'A'))) &&
                            (((RXbuff[15] <= '9') && (RXbuff[15] >= '0')) ||
                             ((RXbuff[15] <= 'F') && (RXbuff[15] >= 'A')))) {
                            if ((RXbuff[11] == 'A' && RXbuff[15] >= '4') || (RXbuff[11] == 'T' && RXbuff[15] == '0'))

                                return EROR;
                            return QSET;
                        }

                        //						if(RXbuff[12] <='9' && RXbuff[12] >='0')
                        //						{
                        //								if(RXbuff[11] == 'A' &&
                        //RXbuff[12] >= '4') 									return EROR; 								return QSET;
                        //						}
                    }
    }
    return EROR;
}

int Get_tag_inf = 0;

void app_reply_atcmd_excute(uint8 cmd_value, uint8 *cmd_buf) {
    //	if(cmd_value ==STAR)//AT+STAR
    //	{
    //		n =	sprintf((char*)&cmd_buf[0],"OK+STAR\r\n");
    //	}

    if (cmd_value == RSET) // AT+REST
    {
        n = sprintf((char *)&cmd_buf[0], "OK+RSET\r\n");
        FLAH_BUFF0[0] = 0x0000;
    }
}
void app_reply_atcmd_search(uint8 cmd_value, uint8 *cmd_buf) {
    //	if(cmd_value == BEEp)//AT+BEEP?

    //		n =	sprintf((char*)&cmd_buf[0],"OK+BEEP=%d\r\n",sys_para.beep);

    if (cmd_value == INFO) // AT+INF?
    {
        display_deviceinfo(&sys_para, INFO_VERSION | INFO_ENET | INFO_UWB | INFO_OTHER); // Show UWB parameters
    }

    //	if(cmd_value ==VBAT)//AT+VBAT?
    //	{
    ////		ADC_Value=read_adcvalue();

    ////		n =	sprintf((char*)&cmd_buf[0], "\r\nOK+VBAT= %.2fV \r\n",convert_adc_to_vbat(ADC_Value) );

    //	}

    if (cmd_value == BAUd) // AT+BAUD?

        n = sprintf((char *)&cmd_buf[0], "OK+BAUD=%dbps\r\n", baudrate);

    if (cmd_value == GAIn) // AT+GAIN?
        n = sprintf((char *)&cmd_buf[0], "OK+GAIN=0x%4X%4X\r\n", sys_para.gain1, sys_para.gain2);

    //	if(cmd_value == UCHg)//AT+UCHG?
    //		n =	sprintf((char*)&cmd_buf[0],"OK+UCHG=%d\r\n",sys_para.uchg);
}
void app_reply_atcmd_setting(uint8 cmd_value, uint8 *cmd_buf, uint8 *temp_rx_buf) {
    //////////////////////////////////////////////////////////////////
    //	if(cmd_value == LIAP)//AT+LIAP=x
    //	{
    //		if(temp_rx_buf[8]=='0')
    //		{
    //			n =	sprintf((char*)&cmd_buf[0],"OK+LIAP=0\r\n");
    //			FLAH_BUFF1[1]=0x0000;
    //		}
    //		if(temp_rx_buf[8]=='1')
    //		{
    //			n =	sprintf((char*)&cmd_buf[0],"OK+LIAP=1\r\n");
    //			FLAH_BUFF1[1]=0xAAAA;
    //		}
    //	}
    //////////////////////////////////////////////////////////////////
    //	if(cmd_value == BEEP)//AT+BEEP=x
    //	{
    //		if(temp_rx_buf[8]=='0')
    //		{
    //			n =	sprintf((char*)&cmd_buf[0],"OK+BEEP=0\r\n");
    //			FLAH_BUFF0[8]=0x00;
    //		}
    //		if(temp_rx_buf[8]=='1')
    //		{
    //			n =	sprintf((char*)&cmd_buf[0],"OK+BEEP=1\r\n");
    //			FLAH_BUFF0[8]=0x01;
    //		}
    //	}
    //////////////////////////////////////////////////////////////////
    //	if(cmd_value == BAUD)//AT+BAUD=x
    //	{
    //		if(temp_rx_buf[8]=='0')
    //		{
    //			n =	sprintf((char*)&cmd_buf[0],"OK+BAUD=0\r\n");
    //			FLAH_BUFF0[5]=0x00;
    //		}
    //		if(temp_rx_buf[8]=='1')
    //		{
    //			n =	sprintf((char*)&cmd_buf[0],"OK+BAUD=1\r\n");
    //			FLAH_BUFF0[5]=0x01;
    //		}
    //		if(temp_rx_buf[8]=='2')
    //		{
    //			n =	sprintf((char*)&cmd_buf[0],"OK+BAUD=2\r\n");
    //			FLAH_BUFF0[5]=0x02;
    //		}
    //
    //	}

    //////////////////////////////////////////////////////////////////
    if (cmd_value == SPED) // AT+SPED=xxxx-xxx
    {
        if (sys_para.role == 0x02)
            n = sprintf((char *)&cmd_buf[0], "\r\nThis is a tag\r\n");
        else if (sys_para.role == 0x01)
            n = sprintf((char *)&cmd_buf[0], "OK+SPED\r\n");

        Load_Set_from_P(temp_rx_buf);
    }
    //////////////////////////////////////////////////////////////////
    if (cmd_value == QSET) // AT+QSET=
    {
        n = sprintf((char *)&cmd_buf[0], "OK+QSET\r\n");

        Load_Set_from_U(temp_rx_buf);
    }

    //////////////////////////////////////////////////////////////////
    if (cmd_value == GAIN) // AT+GAIN=xxxxxxxx
    {
        char buf[2] = {0};
        int sys_pa[2];
        n = sprintf((char *)&cmd_buf[0], "OK+GAIN=0x%c%c%c%c%c%c%c%c\r\n", temp_rx_buf[8], temp_rx_buf[9],
                    temp_rx_buf[10], temp_rx_buf[11], temp_rx_buf[12], temp_rx_buf[13], temp_rx_buf[14],
                    temp_rx_buf[15]);

        memcpy(buf, &temp_rx_buf[8], 4);
        sscanf((char *)buf, "%x", &(sys_pa[0]));
        memcpy(buf, &temp_rx_buf[12], 4);
        sscanf((char *)buf, "%x", &(sys_pa[1]));
        FLAH_BUFF0[6] = sys_pa[0];
        FLAH_BUFF0[7] = sys_pa[1];
    }
    //////////////////////////////////////////////////////////////////
    //	if(cmd_value == UCHG)//AT+UCHG=x
    //	{
    //		if(temp_rx_buf[8]=='0')
    //		{
    //			n =	sprintf((char*)&cmd_buf[0],"OK+UCHG=0\r\n");
    //			FLAH_BUFF0[12]=0x00;
    //		}
    //		if(temp_rx_buf[8]=='1')
    //		{
    //			n =	sprintf((char*)&cmd_buf[0],"OK+UCHG=1\r\n");
    //			FLAH_BUFF0[12]=0x01;
    //		}
    //
    //	}
    //////////////////////////////////////////////////////////////////
    //	if(cmd_value == TCPE)//AT+TCPE=x
    //	{
    //		if(temp_rx_buf[8]=='0')
    //		{
    //			FLAH_BUFF0[24]=0x0000;
    //			n =	sprintf((char*)&cmd_buf[0],"OK+TCPE=0\r\n");

    //		}
    //		if(temp_rx_buf[8]=='1')
    //		{
    //			FLAH_BUFF0[24]=0x0001;
    //			n =	sprintf((char*)&cmd_buf[0],"OK+TCPE=1\r\n");
    //		}
    //	}
    //////////////////////////////////////////////////////////////////
    //	if(cmd_value == TCPS)//AT+TCPS=
    //	{
    //		n =	sprintf((char*)&cmd_buf[0],"OK+TCPS\r\n");
    //		Load_SetTCP(temp_rx_buf);
    //	}
    //////////////////////////////////////////////////////////////////
    //	if(cmd_value==TWLT)//AT+TWLT=
    //	{
    //		if((temp_rx_buf[8]=='1')&&(temp_rx_buf[13]=='4'))
    //		{
    //			n =	sprintf((char*)&cmd_buf[0],"OK+TWLT=1\r\n");
    //			FLAH_BUFF0[48]=0x0001;
    //			Load_tagnum(temp_rx_buf);

    //		}
    //		if(temp_rx_buf[8]=='0')
    //		{
    //			n =	sprintf((char*)&cmd_buf[0],"OK+TWLT=0\r\n");
    //			FLAH_BUFF0[48]=0x0000;
    //		}
    //
    //	}
    //////////////////////////////////////////////////////////////////
    //	if(cmd_value == ACMD)//AT+ACMD
    //	{
    //		char buf[2]={0};
    //		int sys_pa[2];
    //		n =	sprintf((char*)&cmd_buf[0],"OK+ACMD\r\n");
    //
    //		memcpy(buf,&temp_rx_buf[8],4);
    //		sscanf((char* )buf,"%x",&(sys_pa[0]));
    //

    //		memcpy(buf,&temp_rx_buf[13],4);
    //		sscanf((char* )buf,"%x",&(sys_pa[1]));
    ////		air_cmd[0]=sys_pa[0];
    ////		air_cmd[1]=sys_pa[1];
    //
    ////		send_tagcmd=1;

    //	}
}

void Process_USART_ATCMD(uint8 *temp_rx_buf, uint8 *temp_tx_buf) {
    uint8_t usart_cmd_value = 0;

    if (USART_RX_STA & 0x8000) // �ж��Ƿ����������
    {
        USART_RX_STA = 0;

        usart_cmd_value = Check_cmd(temp_rx_buf);
        //////////////////////////////////////////////////////////////////
        if ((usart_cmd_value & 0x40) == 0x40) {
            app_reply_atcmd_excute(usart_cmd_value, temp_tx_buf);
            printf("%s", temp_tx_buf);
            memset(temp_rx_buf, 0, 100); // Clear the buffer
            if (usart_cmd_value == RSET) {
                app_flash_write(); // д��Flash
                HAL_Delay(200);
            }
            NVIC_SystemReset(); // ��λ
        }
        if ((usart_cmd_value & 0x10) == 0x10) {
            app_reply_atcmd_search(usart_cmd_value, temp_tx_buf);
            if (usart_cmd_value != INFO) {
                printf("%s", temp_tx_buf);
            }
            memset(temp_rx_buf, 0, 100); // Clear the buffer
        }

        if ((usart_cmd_value & 0x20) == 0x20) {
            app_reply_atcmd_setting(usart_cmd_value, temp_tx_buf, temp_rx_buf);
            if (usart_cmd_value == ACMD) {
                printf("%s", temp_tx_buf);
                memset(temp_rx_buf, 0, 100); // Clear the buffer
            } else if (usart_cmd_value == SPED) {
                printf("%s", temp_tx_buf);
                memset(temp_rx_buf, 0, 100); // Clear the buffer
                app_flash_write();           // д��Flash
            } else {
                printf("%s", temp_tx_buf);
                memset(temp_rx_buf, 0, 100); // Clear the buffer
                app_flash_write();           // д��Flash
                HAL_Delay(200);
                NVIC_SystemReset(); // ��λ
            }
        }
    }
}
void Process_USART3_ATCMD(uint8 *temp_rx_buf, uint8 *temp_tx_buf) {
    uint8_t usart_cmd_value = 0;

    if (USART3_RX_STA & 0x8000) // �ж��Ƿ����������
    {
        USART3_RX_STA = 0;
        usart_cmd_value = Check_cmd(temp_rx_buf);
        //////////////////////////////////////////////////////////////////
        if ((usart_cmd_value & 0x40) == 0x40) {
            app_reply_atcmd_excute(usart_cmd_value, temp_tx_buf);
            USART3_Send_PartialCommand(temp_tx_buf);
            memset(temp_rx_buf, 0, 100); // Clear the buffer
            if (usart_cmd_value == RSET) {
                app_flash_write(); // д��Flash
                HAL_Delay(200);
            }
            NVIC_SystemReset(); // ��λ
        }
        if ((usart_cmd_value & 0x10) == 0x10) {
            app_reply_atcmd_search(usart_cmd_value, temp_tx_buf);
            if (usart_cmd_value != INFO) {
                USART3_Send_PartialCommand(temp_tx_buf);
            }
            memset(temp_rx_buf, 0, 100); // Clear the buffer
        }

        if ((usart_cmd_value & 0x20) == 0x20) {
            app_reply_atcmd_setting(usart_cmd_value, temp_tx_buf, temp_rx_buf);
            if (usart_cmd_value == ACMD) {
                USART3_Send_PartialCommand(temp_tx_buf);
                memset(temp_rx_buf, 0, 100); // Clear the buffer
            } else if (usart_cmd_value == SPED) {
                USART3_Send_PartialCommand(temp_tx_buf);
                memset(temp_rx_buf, 0, 100); // Clear the buffer
                app_flash_write();           // д��Flash
            } else {
                USART3_Send_PartialCommand(temp_tx_buf);
                memset(temp_rx_buf, 0, 100); // Clear the buffer
                app_flash_write();           // д��Flash
                HAL_Delay(200);
                NVIC_SystemReset(); // ��λ
            }
        }
    }
}
/*
void Process_USB_ATCMD(uint8 *temp_rx_buf,uint8 *temp_tx_buf)
{

        uint8 usb_cmd_value=0;



        if(USB_USART_RX_STA&0x8000)	   //�ж��Ƿ����������
        {
                USB_USART_RX_STA = 0;

                usb_cmd_value = Check_cmd(temp_rx_buf);
//////////////////////////////////////////////////////////////////
                if((usb_cmd_value&0x40) == 0x40)
                {
                        app_reply_atcmd_excute(usb_cmd_value, temp_tx_buf);
                        USB_TxWrite(temp_tx_buf, n);
                        memset(temp_rx_buf,0,100);// Clear the buffer
                        if(usb_cmd_value == RSET)
                        {
                                app_flash_write();//д��Flash
                                HAL_Delay(200);
                        }
                        NVIC_SystemReset(); //��λ
                }
                if((usb_cmd_value&0x10) == 0x10)
                {

                        app_reply_atcmd_search(usb_cmd_value,temp_tx_buf);
                        if(usb_cmd_value != INFO)
                        {
                                USB_TxWrite(temp_tx_buf, n);

                        }
                        memset(temp_rx_buf,0,100);// Clear the buffer

                }

                if((usb_cmd_value&0x20) == 0x20)
                {
                        app_reply_atcmd_setting(usb_cmd_value,temp_tx_buf,temp_rx_buf);
                        if(usb_cmd_value == ACMD)
                        {
                                USB_TxWrite(temp_tx_buf, n);
                                memset(temp_rx_buf,0,100);// Clear the buffer
                        }
                        else if(usb_cmd_value == SPED)
                        {
                                USB_TxWrite(temp_tx_buf, n);
                                memset(temp_rx_buf,0,100);// Clear the buffer
                                app_flash_write();//д��Flash
                        }
                        else
                        {
                                USB_TxWrite(temp_tx_buf, n);
                                memset(temp_rx_buf,0,100);// Clear the buffer
                                app_flash_write();//д��Flash
                                HAL_Delay(200);
                                NVIC_SystemReset(); //��λ
                        }

                }

        }
}
void Process_LAN8742_ATCMD(uint8 *temp_rx_buf,uint8 *temp_tx_buf)
{
        uint8 len=0;

        uint8 LAN8742_cmd_value=0;

        len = sizeof(temp_rx_buf);

        if( len >0 )
        {

                LAN8742_cmd_value = Check_cmd(temp_rx_buf);
//////////////////////////////////////////////////////////////////
                if((LAN8742_cmd_value&0x40) == 0x40)
                {
                        app_reply_atcmd_excute(LAN8742_cmd_value, temp_tx_buf);
                        tcp_client_usersent(tcppcb);
                        memset((char*)&tcp_client_sendbuf,0,sizeof(tcp_client_sendbuf));
                        memset(temp_rx_buf,0,100);// Clear the buffer
                        if(LAN8742_cmd_value == RSET)
                        {
                                app_flash_write();//д��Flash
                                HAL_Delay(200);
                        }
                        NVIC_SystemReset(); //��λ
                }
                if((LAN8742_cmd_value&0x10) == 0x10)
                {

                        app_reply_atcmd_search(LAN8742_cmd_value,temp_tx_buf);
                        if(LAN8742_cmd_value != INFO)
                        {
                                tcp_client_usersent(tcppcb);
                                memset((char*)&tcp_client_sendbuf,0,sizeof(tcp_client_sendbuf));
                        }
                        memset(temp_rx_buf,0,100);// Clear the buffer

                }

                if((LAN8742_cmd_value&0x20) == 0x20)
                {
                        app_reply_atcmd_setting(LAN8742_cmd_value,temp_tx_buf,temp_rx_buf);
                        if(LAN8742_cmd_value == ACMD)
                        {
                                tcp_client_usersent(tcppcb);
                                memset((char*)&tcp_client_sendbuf,0,sizeof(tcp_client_sendbuf));
                                memset(temp_rx_buf,0,100);// Clear the buffer
                        }
                        else if(LAN8742_cmd_value == SPED)
                        {
                                tcp_client_usersent(tcppcb);
                                memset((char*)&tcp_client_sendbuf,0,sizeof(tcp_client_sendbuf));
                                memset(temp_rx_buf,0,100);// Clear the buffer
                                app_flash_write();//д��Flash
                        }
                        else
                        {
                                tcp_client_usersent(tcppcb);
                                memset((char*)&tcp_client_sendbuf,0,sizeof(tcp_client_sendbuf));
                                memset(temp_rx_buf,0,100);// Clear the buffer
                                app_flash_write();//д��Flash
                                HAL_Delay(200);
                                NVIC_SystemReset(); //��λ
                        }

                }
        }
}*/

//////////////////////////////////////////////////////////////////
void Load_Set_from_U(uint8_t *RXbuff) {
    char buf[4] = {0};
    int sys_pa;
    // int getid=0;
    if (RXbuff[8] == 'F') sys_para.datarate = 0x03;
    //	else if(RXbuff[8] == 'S')
    //		sys_para.datarate = 0x01;
    //	else if(RXbuff[8] == 'M')
    //		sys_para.datarate = 0x02;

    if (RXbuff[9] == '5')
        sys_para.channel = 0x05;
    else if (RXbuff[9] == '2')
        sys_para.channel = 0x02;

    if (RXbuff[11] == 'A')
        sys_para.role = 0x01;
    else if (RXbuff[11] == 'T')
        sys_para.role = 0x02;

    // Load Address
    memcpy(buf, &RXbuff[12], 4);
    sscanf((char *)buf, "%x", &(sys_pa));
    //	getid=(RXbuff[12]-'0')*16*16*16 + (RXbuff[13]-'0')*16*16+(RXbuff[14]-'0')*16+(RXbuff[15]-'0');
    sys_para.uwbid = sys_pa;
    sys_para.flag = 0xAAAA;

    FLAH_BUFF0[0] = 0xAAAA;
    FLAH_BUFF0[2] = sys_para.role;
    FLAH_BUFF0[3] = sys_para.channel;
    FLAH_BUFF0[4] = sys_para.datarate;
    FLAH_BUFF1[0] = sys_para.uwbid;
}
//////////////////////////////////////////////////////////////////
void Load_Set_from_P(uint8_t *RXbuff) {
    //	{
    sys_para.TAG_NUM = (RXbuff[8] - '0') * 100 + (RXbuff[9] - '0') * 10 + (RXbuff[10] - '0');
    sys_para.TAG_SLOT =
        (RXbuff[12] - '0') * 1000 + (RXbuff[13] - '0') * 100 + (RXbuff[14] - '0') * 10 + (RXbuff[15] - '0');
    FLAH_BUFF0[14] = sys_para.TAG_NUM;
    FLAH_BUFF0[15] = sys_para.TAG_SLOT;
    Get_tag_inf = 1;

    //	}
}
//////////////////////////////////////////////////////////////////
void Load_tagnum(uint8_t *RXbuff) {
    uint8_t buf[10] = {0};
    int t = 0;
    int sys_pa[4];
    for (t = 0; t < 4; t++) {
        memcpy(buf, &RXbuff[15 + t * 5], 4);
        sscanf((char *)buf, "%x", &(sys_pa[t]));
        FLAH_BUFF0[49 + t] = sys_pa[t];
    }
}
//////////////////////////////////////////////////////////////////
void Load_SetTCP(uint8_t *RXbuff) {
    //	uint8_t buf[10] = {0};
    //	int sys_pa[12];
    //	int t=0;
    //
    //	sys_para.tcpset = 0xABCD;
    //	for(t=0;t<=11;t++)
    //	{
    //		memcpy(buf,&RXbuff[8+t*4],4);
    //		sscanf((char* )buf,"%x",&(sys_pa[t]));
    //		FLAH_BUFF0[26+t]=sys_pa[t];
    //	}
    //	FLAH_BUFF0[25] = sys_para.tcpset;
}
void Process_ATCMD(void) {
    Process_USART_ATCMD(USART_RX_BUF, USART_TX_BUF);
    Process_USART3_ATCMD(USART3_RX_BUF, USART3_TX_BUF);
}

void display_deviceinfo(System_Para_TypeDef *syspara, uint16_t type) {
    int n = 0;
    unsigned char cRevisionDate[12] = __DATE__;

    if ((type & INFO_VERSION) == INFO_VERSION) {
        n += sprintf((char *)&Sendbuff[n], "\r\n\r\n>> COPYRIGHT 2022 YCHIOT LTD\r\n\r\n");
        n += sprintf((char *)&Sendbuff[n], ">> HW Version:   Mini5 V1.0\r\n");
        n += sprintf((char *)&Sendbuff[n], ">> SW Version:   01.01.00\r\n");
        n += sprintf((char *)&Sendbuff[n], ">> Build Date:   %s\r\n", cRevisionDate);
        // n+=	sprintf((char*)&Sendbuffusb[n],   ">> MCU UniqueID:
        // %08x-%08x-%08x\r\n",*(uint32_t*)(0x1FFF7A10),*(uint32_t*)(0x1FFF7A14),*(uint32_t*)(0x1FFF7A18) ); n+=
        // sprintf((char*)&Sendbuffusb[n],   ">> Start Number: %d\r\n",syspara->start_count );
        n += sprintf((char *)&Sendbuff[n], ">> Baudrate:     %dbps\r\n", baudrate);
    }
    if ((type & INFO_UWB) == INFO_UWB) {
        if (syspara->role == 0x02)
            n += sprintf((char *)&Sendbuff[n], ">> UWB Mode:     Tag %d\r\n", syspara->uwbid);
        else if (syspara->role == 0x01)
            n += sprintf((char *)&Sendbuff[n], ">> UWB Mode:     Anchor %d\r\n", syspara->uwbid);

        if (syspara->datarate == 0x03)
            n += sprintf((char *)&Sendbuff[n], ">> UWB DataRate: 6.81Mbps\r\n");
        else if (syspara->datarate == 0x01)
            n += sprintf((char *)&Sendbuff[n], ">> UWB DataRate: 110Kbps\r\n");
        else if (syspara->datarate == 0x02)
            n += sprintf((char *)&Sendbuff[n], ">> UWB DataRate: 850Kbps\r\n");

        if (syspara->channel == 0x05)
            n += sprintf((char *)&Sendbuff[n], ">> UWB Channel:  5\r\n\r\n");
        else if (syspara->channel == 0x02)
            n += sprintf((char *)&Sendbuff[n], ">> UWB Channel:  2\r\n\r\n");
    }

    if (((type & INFO_OTHER) == INFO_OTHER) && (sys_para.role == 0x01)) {
        n += sprintf((char *)&Sendbuff[n], ">> SYS_TAG_NUM:  %d\r\n", syspara->TAG_NUM);
        n += sprintf((char *)&Sendbuff[n], ">> SYS_TAG_SLOT: %d\r\n\r\n", syspara->TAG_SLOT);
    }
    //	USB_TxWrite(Sendbuffusb, n);
    //	if(tcp_client_flag&1<<5)
    //	{
    //		memcpy((uint8 *)&tcp_client_sendbuf,(uint8*)Sendbuffusb,n);
    //		tcp_client_usersent(tcppcb);
    //		memset((char*)&tcp_client_sendbuf,0,sizeof(tcp_client_sendbuf));

    //	}
    USART3_Send_PartialCommand(Sendbuff);
    printf("%s", Sendbuff);
    n = 0;
    HAL_Delay(100);
}
