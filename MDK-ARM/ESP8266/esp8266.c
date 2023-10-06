#include "esp8266.h"
#include "tim.h"
#include "usart.h"


/*

��ESP8266��������
cmd:���͵������ַ���
ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
waittime:�ȴ�ʱ��(��λ:10ms)
����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)
       1,����ʧ��
	
*/

uint8_t esp8266_send_cmd(uint8_t *cmd, uint8_t *ack, uint16_t waittime)
{
	uint8_t res = 0;
	USART3_RX_STA = 0;
	memset(USART3_RX_BUF,0,USART3_MAX_RECV_LEN);
	u3_printf("%s\r\n",cmd);
	if(waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)	//�ȴ�����ʱ
		{
			Delay_ms(10);
			if(USART3_RX_STA&0X8000)//���յ��ڴ���Ӧ����
			{
				if(esp8266_check_cmd(ack))
				{
					u1_printf("ack:%s\r\n",(uint8_t*)ack);
					break;//�õ���Ч���� 
				}
				USART3_RX_STA=0;
			} 
		}
		if(waittime==0)
			res=1; 
	}
	return res;
}


/*

��ESP8266����ָ������
data:���͵�����(����Ҫ��ӻس���)
ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
waittime:�ȴ�ʱ��(��λ:10ms)
����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)
	
*/

uint8_t atk_8266_send_data(uint8_t *data, uint8_t *ack, uint16_t waittime)
{
	uint8_t res=0; 
	USART3_RX_STA=0;
	u3_printf("%s",data);	//��������
	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)	//�ȴ�����ʱ
		{
			Delay_ms(10);
			if(USART3_RX_STA&0X8000)//���յ��ڴ���Ӧ����
			{
				if(esp8266_check_cmd(ack))break;//�õ���Ч���� 
				USART3_RX_STA=0;
			} 
		}
		if(waittime==0)
			res=1; 
	}
	return res;
}


/*

ATK-ESP8266���������,�����յ���Ӧ��
str:�ڴ���Ӧ����
����ֵ:0,û�еõ��ڴ���Ӧ����
    ����,�ڴ�Ӧ������λ��(str��λ��)
	
*/

uint8_t* esp8266_check_cmd(uint8_t *str)
{
	
	char *strx=0;
	if(USART3_RX_STA&0X8000)		//���յ�һ��������
	{ 
		u1_printf("%s\r\n",(char*)USART3_RX_BUF);
		USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//��ӽ�����
		
		strx=strstr((const char*)USART3_RX_BUF,(const char*)str);
	} 
	return (uint8_t*)strx;
}


/*

ATK-ESP8266�˳�͸��ģʽ
����ֵ:0,�˳��ɹ�;
       1,�˳�ʧ��	

*/

uint8_t esp8266_quit_trans(void)
{
	while((USART3->SR&0X40)==0);	//�ȴ����Ϳ�
	USART3->DR='+';      
	HAL_Delay(15);					//���ڴ�����֡ʱ��(10ms)
	while((USART3->SR&0X40)==0);	//�ȴ����Ϳ�
	USART3->DR='+';      
	HAL_Delay(15);					//���ڴ�����֡ʱ��(10ms)
	while((USART3->SR&0X40)==0);	//�ȴ����Ϳ�
	USART3->DR='+';      
	HAL_Delay(500);					//�ȴ�500ms
	return esp8266_send_cmd("AT","OK",20);//�˳�͸���ж�.
}


uint8_t esp8266_Connect_IOTServer(void)
{
	u1_printf("׼������ģ��\r\n");
	HAL_Delay(100);
	
	
	esp8266_send_cmd("AT","OK",50);
	u1_printf("׼���˳�͸��ģʽ\n");
	if(esp8266_quit_trans())
	{
		u1_printf("�˳�͸��ģʽʧ�ܣ�׼������\r\n");
		return 6;
	}else u1_printf("�˳�͸��ģʽ�ɹ�\r\n");
	
	
	
	u1_printf("׼���رջ���\r\n");
	if(esp8266_send_cmd("ATE0","OK",50))
	{
		u1_printf("�رջ���ʧ��׼������\r\n");
		return 1;
	}else u1_printf("�رջ��Գɹ�\r\n");
	
	u1_printf("��ѯģ���Ƿ�����\r\n");
	if(esp8266_send_cmd("AT","OK",50))
	{
		u1_printf("ģ�鲻����׼������\r\n");
		return 1;
	}else u1_printf("���ò�ѯ���߳ɹ�\r\n");
	
	
	
	
	u1_printf("׼������STAģʽ\r\n");
	if(esp8266_send_cmd("AT+CWMODE=1","OK",50))
	{
		u1_printf("����STAģʽʧ��׼������\r\n");
		return 1;
	}else u1_printf("����STAģʽ�ɹ�\r\n");
	
	u1_printf("׼������\r\n");
	if(esp8266_send_cmd("AT+RST","OK",50))
	{
		u1_printf("����ʧ�ܣ�׼������\r\n");
		return 2;
	}else u1_printf("�����ɹ����ȴ�����\r\n");
	
	Delay_ms(1000);
	Delay_ms(1000);
	Delay_ms(1000);
	Delay_ms(1000);
	
	
	u1_printf("׼��ȡ���Զ�����\r\n");
	if(esp8266_send_cmd("AT+CWAUTOCONN=0","OK",50))
	{
		u1_printf("ȡ���Զ�����ʧ�ܣ�׼������\r\n");
		return 3;
	}else u1_printf("ȡ���Զ����ӳɹ�\r\n");
	
	u1_printf("׼������·����\r\n");
	if(esp8266_Connect_AP())
	{
		u1_printf("����·����ʧ�ܣ��ȴ�����\r\n");
		return 4;
	}else u1_printf("����·�����ɹ�\r\n");
	
	Delay_ms(1000);
	Delay_ms(1000);
	
	
	u1_printf("׼������DHCP\r\n");
	if(esp8266_send_cmd("AT+CWDHCP=1,1","OK",100))
	{
		u1_printf("����DHCPʧ�ܣ�׼������\r\n");
		return 7;
	}else u1_printf("����DHCP�ɹ�\r\n");
	
	
	u1_printf("����Ϊ�رն�·����\r\n");
	if(esp8266_send_cmd("AT+CIPMUX=0","OK",100))
	{
		u1_printf("�رն�·����ʧ�ܣ�׼������\r\n");
		return 7;
	}else u1_printf("���ùرն�·���ӳɹ�\r\n");
	
	
	u1_printf("׼�����ӷ�����\r\n");
	if(esp8266_Connect_Server())
	{
		u1_printf("���ӷ�����ʧ�ܣ��ȴ�����\r\n");
		return 8;
	}else u1_printf("���ӷ������ɹ�\r\n");
	
	u1_printf("׼���˳�͸��ģʽ\n");
	if(esp8266_quit_trans())
	{
		u1_printf("�˳�͸��ģʽʧ�ܣ�׼������\r\n");
		return 6;
	}else u1_printf("�˳�͸��ģʽ�ɹ�\r\n");
	
	u1_printf("����Ϊ͸��ģʽ\r\n");
	if(esp8266_send_cmd("AT+CIPMODE=1","OK",50))
	{
		u1_printf("����͸��ʧ�ܣ�׼������\r\n");
		return 6;
	}else u1_printf("����͸���ɹ�\r\n");

	
	
	
	u1_printf("���ÿ���͸��ģʽ\r\n");
	if(esp8266_send_cmd("AT+CIPSEND","OK",1000))
	{
		u1_printf("����͸��ʧ�ܣ�׼������\r\n");
		return 9;
	}else u1_printf("����͸���ɹ�\r\n");
	
	
	
	while(1){
		Delay_ms(1000);
		//esp8266_send_cmd("AT+CIPSEND","OK",200);  //����ָ�����ȵ�����
		u3_printf("This is a message\r\n");
	}
//	return 0;
}


/*

�ͻ������ô���
����ֵ:0,���ӳɹ�;
       ����,����ʧ��	

*/

uint8_t esp8266_client_config(void)
{
	u1_printf("׼������esp8266�ͻ���ģ��\r\n");
	HAL_Delay(100);
	
	esp8266_send_cmd("AT","OK",50); //���WIFIģ���Ƿ�����
	
	u1_printf("׼������STAģʽ\r\n");
	if(esp8266_send_cmd("AT+CWMODE=1","OK",50)) //1.Stationģʽ 2.Apģʽ 3.����1.2����ģʽ
	{
		u1_printf("����STAģʽʧ��׼������\r\n");
		return 1;
	}else u1_printf("����STAģʽ�ɹ�\r\n");
	
	u1_printf("׼������\r\n");
	if(esp8266_send_cmd("AT+RST","OK",50)) //������λ
	{
		u1_printf("����ʧ�ܣ�׼������\r\n");
		return 2;
	}else u1_printf("�����ɹ����ȴ�����\r\n");
	Delay_ms(4000);
	
	u1_printf("׼������WIFI\r\n");
	if(esp8266_Connect_AP()) //����WIFI
	{
		u1_printf("����WIFIʧ�ܣ��ȴ�����\r\n");
		return 3;
	}else u1_printf("����WIFI�ɹ�\r\n");
	Delay_ms(2000);
	
	u1_printf("׼�����ӷ�����\r\n");
	if(esp8266_Connect_Server()) //���ӷ�����
	{
		u1_printf("���ӷ�����ʧ�ܣ��ȴ�����\r\n");
		return 4;
	}else u1_printf("���ӷ������ɹ�\r\n");
	
	return 0;
}


/*

���������ô���
����ֵ:0,���ӳɹ�;
       ����,����ʧ��	

*/

uint8_t esp8266_server_config(void)
{
	u1_printf("׼������esp8266������ģ��\r\n");
	HAL_Delay(100);
	
	esp8266_send_cmd("AT","OK",50); //���WIFIģ���Ƿ�����
	
	u1_printf("׼������APģʽ\r\n");
	if(esp8266_send_cmd("AT+CWMODE=2","OK",50)) //1.Stationģʽ2.Apģʽ3.����1.2����ģʽ
	{
		u1_printf("����APģʽʧ��׼������\r\n");
		return 1;
	}else u1_printf("����APģʽ�ɹ�\r\n");
	
	u1_printf("׼������\r\n");
	if(esp8266_send_cmd("AT+RST","OK",50)) //������λ��ģʽ��Ч
	{
		u1_printf("����ʧ�ܣ�׼������\r\n");
		return 2;
	}else u1_printf("�����ɹ����ȴ�����\r\n");
	Delay_ms(4000);
	
	u1_printf("׼������������\r\n");
	if(esp8266_send_cmd("AT+CIPMUX=0","OK",50)) //0�������ӣ�1��������
	{
		u1_printf("����������ʧ�ܣ��ȴ�����\r\n");
		return 3;
	}else u1_printf("���������ӳɹ�\r\n");
	
	u1_printf("׼������������\r\n");
	if(esp8266_Set_PORTNUM()) //����������serverָ��	
	{
		u1_printf("����������ʧ�ܣ��ȴ�����\r\n");
		return 4;
	}else u1_printf("�����������ɹ�\r\n");
	
	return 0;
}


/*

���÷������˿�
����ֵ:0,���ӳɹ�;
       1,����ʧ��	

*/

uint8_t esp8266_Set_PORTNUM()
{
	uint8_t i=10;
	char *p = (char*)malloc(50);
	sprintf((char*)p,"AT+CIPSERVER=1,%s",PORTNUM);
	while(esp8266_send_cmd(p,"OK",300)&i)
	{
		u1_printf("���÷������˿�ʧ�ܣ�������������\r\n");
		i--;
	}
	free(p);
	if(i)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


/*

����WIFI
����ֵ:0,���ӳɹ�;
       1,����ʧ��	

*/

uint8_t esp8266_Connect_AP()
{
	uint8_t i=10;
	char *p = (char*)malloc(50);
	sprintf((char*)p,"AT+CWJAP=\"%s\",\"%s\"",SSID,PASS);
	while(esp8266_send_cmd(p,"WIFI GOT IP",300)&i)
	{
		u1_printf("����APʧ�ܣ�������������\r\n");
		i--;
	}
	free(p);
	if(i)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}



/*

���ӷ�������IP��ַ�Ͷ˿�
����ֵ:0,���ӳɹ�;
       1,����ʧ��	

*/

uint8_t esp8266_Connect_Server()
{
	uint8_t i=10;
	char *p = (char*)malloc(50);
	sprintf((char*)p,"AT+CIPSTART=\"TCP\",\"%s\",%s",IPBUF,PORTNUM);
	while(esp8266_send_cmd(p,"CONNECT",300)&i)
	{
		u1_printf("���ӷ�����ʧ�ܣ�������������\r\n");
		i--;
	}
	free(p);
	if(i)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


